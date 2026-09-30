/* Integrated actuator bench with one owner for fault state and all outputs. */
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stm32_ll_tim.h>
#include "motor_control.h"
#include "brake.h"
#include "pi_link.h"
#include "servo.h"
#include "blinkers.h"
#include "encoder.h"
#include "system_state.h"
#include "test_points.h"

static K_MUTEX_DEFINE(output_lock);
static struct system_state state;
static bool braking = true, servo_ready, have_light_request;
static uint8_t last_light_event;
static uint16_t duty, accepted;
static int16_t steering;

static bool initialize_modules(void)
{
    int motor_rc = motor_init();
    int brake_rc = motor_brake_all();
    int light_rc = blinkers_init();
    int encoder_rc = encoders_init();
    int servo_rc = servo_prepare();
    servo_ready = servo_rc == 0;
    return motor_rc == 0 && brake_rc == 0 && light_rc == 0 && servo_rc == 0 &&
           (encoder_rc == 0 || encoder_rc == -EALREADY);
}

static void brake_locked(void)
{
    if (motor_brake_all()) system_state_output_failed(&state);
    braking = true;
    duty = 0; /* Propulsion duty; EN is HIGH while dynamically braking. */
}
static void fail_safe_locked(void)
{
    brake_locked();
    if (servo_ready && servo_get_last_pulse_us() && servo_center()) system_state_output_failed(&state);
    if (blinkers_get_mode() != BLINKER_MODE_HAZARD && blinkers_set_error(true)) system_state_output_failed(&state);
    have_light_request = false;
}

static bool well_formed(const cmd_payload_t *cmd, bool plausible)
{
    return plausible && (cmd->flags & 0xFCU) == 0x80U &&
        cmd->steering_raw >= -100 && cmd->steering_raw <= 100 &&
        cmd->throttle_raw >= 0 && cmd->throttle_raw <= 100 &&
        cmd->brake_raw >= 0 && cmd->brake_raw <= 100;
}
bool pi_link_command_is_fresh(const cmd_payload_t *cmd, bool plausible)
{
    return well_formed(cmd, plausible) && !(cmd->flags & 1U);
}

static void apply_command_locked(const cmd_payload_t *cmd)
{
    if (cmd->brake_raw > 0) brake_locked();
    else {
        direction_t dir = cmd->throttle_raw ? FORWARD : FREE;
        int a = sys_motor_set(LEFT_MOTOR, cmd->throttle_raw, dir);
        int b = sys_motor_set(RIGHT_MOTOR, cmd->throttle_raw, dir);
        if (a || b) system_state_output_failed(&state);
        else { duty = cmd->throttle_raw; braking = false; }
    }
    if (system_state_reason(&state) != SYS_OK) { fail_safe_locked(); return; }
    bool enable_servo = (cmd->buttons & 4U) != 0;
    int servo_rc = enable_servo ? servo_set_steering(cmd->steering_raw) : servo_disable();
    if (servo_rc) { system_state_output_failed(&state); fail_safe_locked(); return; }
    steering = cmd->steering_raw;
    uint8_t event = cmd->buttons >> 8;
    if (!have_light_request || event != last_light_event) {
        if (blinkers_set_mode((enum blinker_mode)(cmd->buttons & 3U))) system_state_output_failed(&state);
        last_light_event = event;
        have_light_request = true;
    }
    /* Repeated control packets must not undo auto-cancel or restart blink phase. */
    if (blinkers_update_steering(steering)) system_state_output_failed(&state);
    if (system_state_reason(&state) != SYS_OK) fail_safe_locked();
}

void pi_link_on_command_received(const cmd_payload_t *cmd, bool plausible)
{
    k_mutex_lock(&output_lock, K_FOREVER);
    if (!well_formed(cmd, plausible)) {
        system_state_bad_input(&state);
    } else {
        uint8_t test_counter = (cmd->buttons >> 3) & 31U;
        bool sync = (cmd->flags & 2U) != 0 || !state.button_synced;
        if (sync) system_state_sync_button(&state, test_counter);
        if (cmd->flags & 1U) {
            system_state_link_lost(&state);
        } else {
            accepted++;
            system_state_valid(&state);
            bool recover = !sync && system_state_button(&state, test_counter, (uint64_t)k_uptime_get());
            if (recover && (!state.hardware_ready || state.output_fault)) {
                system_state_reinitialized(&state, initialize_modules());
                fail_safe_locked();
            }
        }
    }
    if (system_state_reason(&state) == SYS_OK) apply_command_locked(cmd);
    else fail_safe_locked();
    k_mutex_unlock(&output_lock);
}
void pi_link_on_link_restored(void) { /* A transport event alone cannot release outputs. */ }
void pi_link_on_link_lost(void)
{
    k_mutex_lock(&output_lock, K_FOREVER);
    system_state_link_lost(&state);
    fail_safe_locked();
    k_mutex_unlock(&output_lock);
}
void pi_link_on_invalid_frame(void)
{
    k_mutex_lock(&output_lock, K_FOREVER);
    system_state_bad_input(&state);
    fail_safe_locked();
    k_mutex_unlock(&output_lock);
}

void pi_link_customize_heartbeat(hb_payload_t *hb)
{
    k_mutex_lock(&output_lock, K_FOREVER);
    enum system_reason reason = system_state_reason(&state);
    /* State and outputs are captured together; this is not the current-sensor format. */
    hb->zone_state = reason == SYS_OK ? ZONE_STATE_NORMAL :
        reason == SYS_STARTUP ? ZONE_STATE_INIT : reason == SYS_SELF_TEST ? ZONE_STATE_LOCAL_FAIL :
        reason == SYS_LINK_LOST ? ZONE_STATE_LINK_LOST : ZONE_STATE_SYS_ERROR;
    hb->bus_degraded = 0xB3;
    hb->current_motor_l = duty | ((uint16_t)state.test_counter << 8);
    hb->current_motor_r = (braking ? 1U : 0U) |
        ((uint16_t)blinkers_get_mode() << 1) |
        (servo_get_last_pulse_us() ? 8U : 0U) |
        (blinkers_turn_crossed() ? 16U : 0U) | (state.self_test ? 32U : 0U) |
        (((GPIOC->ODR >> 6) & 15U) << 8) | ((uint16_t)reason << 12);
    hb->current_servo = (uint16_t)servo_get_last_pulse_us();
    k_mutex_unlock(&output_lock);
}

static int status_cmd(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    k_mutex_lock(&output_lock, K_FOREVER);
    struct system_state snapshot = state;
    uint16_t current_duty = duty, count = accepted;
    bool current_brake = braking;
    int16_t current_steering = steering;
    uint32_t period3 = LL_TIM_GetAutoReload(TIM3)+1;
    uint32_t left = LL_TIM_OC_GetCompareCH1(TIM3), right = LL_TIM_OC_GetCompareCH2(TIM3);
    uint32_t port_b = GPIOB->ODR, port_c = GPIOC->ODR;
    uint32_t period1 = LL_TIM_GetAutoReload(TIM1)+1, compare1 = LL_TIM_OC_GetCompareCH1(TIM1);
    uint32_t pulse = servo_get_last_pulse_us();
    unsigned lights = blinkers_get_mode(), crossed = blinkers_turn_crossed();
    k_mutex_unlock(&output_lock);
    struct test_point_counts marks = test_points_counts();
    shell_print(sh, "Test points: CMD_RX=%u PWM_SET=%u errors=%u PC10=%u PC11=%u",
                marks.command_rx, marks.pwm_set, marks.errors,
                !!(GPIOC->ODR & BIT(10)), !!(GPIOC->ODR & BIT(11)));
    /* Console logging must not block command handling while holding output_lock. */
    shell_print(sh, "Actuator bench: duty=%u brake=%u fault=%u accepted=%u received=%u rejected=%u",
                current_duty, current_brake, system_state_reason(&snapshot) != SYS_OK, count,
                pi_link_rx_valid_count(), pi_link_rx_crc_fail_count());
    shell_print(sh, "State=%s ready=%u self_test=%u input_error=%u output_error=%u test_ack=%u timeout_age_ms=%u",
                system_reason_name(system_state_reason(&snapshot)), snapshot.hardware_ready, snapshot.self_test,
                snapshot.bad_input, snapshot.output_fault, snapshot.test_counter, pi_link_last_timeout_age_ms());
    shell_print(sh, "TIM3: period=%u left_compare=%u right_compare=%u; PB12..15=%u%u%u%u",
                period3, left, right, !!(port_b & BIT(12)), !!(port_b & BIT(13)),
                !!(port_b & BIT(14)), !!(port_b & BIT(15)));
    shell_print(sh, "Steering=%d pulse_us=%u; TIM1 period=%u compare=%u; lights=%u crossed=%u PC6..9=%u%u%u%u",
                current_steering, pulse, period1, compare1, lights, crossed,
                !!(port_c & BIT(6)), !!(port_c & BIT(7)), !!(port_c & BIT(8)), !!(port_c & BIT(9)));
    return 0;
}
static int inject_output_fault(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    k_mutex_lock(&output_lock, K_FOREVER);
    system_state_output_failed(&state);
    fail_safe_locked();
    k_mutex_unlock(&output_lock);
    shell_print(sh, "Simulated output fault latched; double-press requests hardware reinitialization.");
    return 0;
}
SHELL_STATIC_SUBCMD_SET_CREATE(bench_commands,
    SHELL_CMD(output_fault, NULL, "Simulate output failure for state testing", inject_output_fault),
    SHELL_CMD(status, NULL, "Read motor, steering and lighting output registers", status_cmd),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(bench, &bench_commands, "Integrated actuator test", NULL);


int main(void)
{
    system_state_init(&state, initialize_modules());
    fail_safe_locked();
    pi_link_init();
    printk("ACTUATOR TEST with unified fault state; startup brake/hazards, servo PWM off.\n");
    printk("Self-test: press -> fail now; second press 50..500 ms later -> recovery request.\n");
    printk("Use bench status / encoder status. UART watchdog: 60 ms threshold + periodic check.\n");
    return 0;
}
