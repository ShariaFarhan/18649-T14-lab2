/* Real wheel -> Pi -> raw UART -> steering. Motor outputs stay disabled. */
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio.h>
#include <stm32_ll_tim.h>
#include "pi_link.h"
#include "servo.h"
#include "wheel_mapping.h"
#include "wheel_buttons.h"
#include "blinkers.h"
#ifdef LAB2_WHEEL_DUAL_DRIVE
#include "dual_drive.h"
#define ALLOWED_FLAGS (FLAG_UDP_STALE | FLAG_DUAL_ENABLE)
#else
#define ALLOWED_FLAGS FLAG_UDP_STALE
#endif

static K_MUTEX_DEFINE(steering_lock);
static bool initialized, active, failed;
static uint8_t zone = ZONE_STATE_INIT;
static int32_t raw_steering = WHEEL_RAW_CENTER;
static int16_t steering_percent;
static struct wheel_buttons buttons;
static bool lights_ready;

static void safe_locked(uint8_t reason)
{
#ifdef LAB2_WHEEL_DUAL_DRIVE
    dual_drive_fail_safe();
#endif
    if (initialized && active && servo_center()) failed = true;
    if (lights_ready && blinkers_get_mode() != BLINKER_MODE_OFF &&
        blinkers_set_mode(BLINKER_MODE_OFF)) failed = true;
    wheel_buttons_reset(&buttons);
    steering_percent = 0;
    zone = failed ? ZONE_STATE_SYS_ERROR : reason;
}
bool pi_link_command_is_fresh(const cmd_payload_t *cmd, bool plausible)
{
    return plausible && !(cmd->flags & ~ALLOWED_FLAGS) && !(cmd->flags & FLAG_UDP_STALE);
}
void pi_link_on_command_received(const cmd_payload_t *cmd, bool plausible)
{
    k_mutex_lock(&steering_lock, K_FOREVER);
    int16_t percent;
    if (!plausible || (cmd->flags & ~ALLOWED_FLAGS) || wheel_steering_percent(cmd->steering_raw,&percent)) {
        safe_locked(ZONE_STATE_SYS_ERROR);
    } else if (cmd->flags & FLAG_UDP_STALE) {
        safe_locked(active ? ZONE_STATE_LINK_LOST : ZONE_STATE_INIT);
    } else if (!initialized || failed) {
        safe_locked(ZONE_STATE_SYS_ERROR);
    } else {
#ifdef LAB2_WHEEL_DUAL_DRIVE
        /* Apply brake / motor input before steering and light work. */
        if(dual_drive_command(cmd->throttle_raw,cmd->brake_raw,cmd->flags & FLAG_DUAL_ENABLE)) {
            safe_locked(ZONE_STATE_SYS_ERROR);
            k_mutex_unlock(&steering_lock);
            return;
        }
#endif
        raw_steering = cmd->steering_raw;
        steering_percent = percent;
        if (servo_set_steering(percent)) { failed=true; safe_locked(ZONE_STATE_SYS_ERROR); }
        else {
            active=true;
            uint16_t press=wheel_buttons_pressed(&buttons,cmd->buttons,k_uptime_get());
            int rc=0;
            if (press) {
                enum blinker_mode selected=press==WHEEL_LSB_MASK ? BLINKER_MODE_LEFT : BLINKER_MODE_RIGHT;
                rc=blinkers_set_mode(blinkers_get_mode()==selected ? BLINKER_MODE_OFF : selected);
            }
            if (!rc) rc=blinkers_update_steering(percent);
            if (rc) { failed=true; safe_locked(ZONE_STATE_SYS_ERROR); }
            else zone=ZONE_STATE_NORMAL;
        }
    }
    k_mutex_unlock(&steering_lock);
}
void pi_link_on_link_restored(void) { }
void pi_link_on_link_lost(void)
{
    k_mutex_lock(&steering_lock,K_FOREVER);safe_locked(ZONE_STATE_LINK_LOST);k_mutex_unlock(&steering_lock);
}
void pi_link_on_invalid_frame(void)
{
    k_mutex_lock(&steering_lock,K_FOREVER);safe_locked(ZONE_STATE_SYS_ERROR);k_mutex_unlock(&steering_lock);
}
void pi_link_customize_heartbeat(hb_payload_t *hb)
{
    k_mutex_lock(&steering_lock,K_FOREVER);
    hb->zone_state=zone;
    hb->bus_degraded=0xB5; /* Steering/light test telemetry, not measured currents. */
    hb->current_motor_l=(uint16_t)raw_steering;
    /* Low byte: percent+100; bits 8..9: mode; bits 10..13: FL/FR/RL/RR GPIO levels. */
    hb->current_motor_r=(uint16_t)(steering_percent+100) |
        ((uint16_t)blinkers_get_mode()<<8) | ((uint16_t)((GPIOC->ODR>>6)&15U)<<10);
    hb->current_servo=(uint16_t)servo_get_last_pulse_us();
#ifdef LAB2_WHEEL_DUAL_DRIVE
    struct dual_drive_status drive=dual_drive_get();
    hb->bus_degraded=0xB8;
    hb->dual_target_mrpm=drive.target_mrpm;hb->dual_left_mrpm=drive.left_mrpm;hb->dual_right_mrpm=drive.right_mrpm;
    hb->dual_duty_percent=drive.duty_percent;
    hb->dual_flags=drive.armed | (drive.braking<<1) | (drive.fault<<2) | (drive.boosting<<3) | ((uint8_t)drive.reason<<4);
#endif
    k_mutex_unlock(&steering_lock);
}
static int status_cmd(const struct shell *sh,size_t argc,char **argv)
{
    ARG_UNUSED(argc);ARG_UNUSED(argv);
    k_mutex_lock(&steering_lock,K_FOREVER);
    int32_t raw=raw_steering;int16_t p=steering_percent;uint8_t z=zone;
    uint32_t pulse=servo_get_last_pulse_us(),compare=LL_TIM_OC_GetCompareCH1(TIM1);
    unsigned light_mode=blinkers_get_mode(),light_pins=(GPIOC->ODR>>6)&15U;
    k_mutex_unlock(&steering_lock);
    shell_print(sh,"Wheel steering: state=%u raw=%d percent=%d pulse_us=%u TIM1_compare=%u motor_EN=%u/%u",
                z,raw,p,pulse,compare,!!(GPIOA->ODR & BIT(6)),!!(GPIOA->ODR & BIT(7)));
    shell_print(sh,"Lights: mode=%u pins_FL_FR_RL_RR=0x%x; LSB bit9 / RSB bit8",light_mode,light_pins);
#ifdef LAB2_WHEEL_DUAL_DRIVE
    struct dual_drive_status drive=dual_drive_get();
    shell_print(sh,"DUAL: armed=%d brake=%d fault=%d target_mrpm=%d average_mrpm=%d PWM=%u%%",
                drive.armed,drive.braking,drive.fault,drive.target_mrpm,drive.actual_mrpm,drive.duty_percent);
#endif
    return 0;
}
SHELL_CMD_REGISTER(steering_status,NULL,"Read raw wheel / steering PWM status",status_cmd);
int main(void)
{
#ifdef LAB2_WHEEL_DUAL_DRIVE
    int motor_rc=dual_drive_init();
#else
    const struct device *gpioa=DEVICE_DT_GET(DT_NODELABEL(gpioa));
    int a=gpio_pin_configure(gpioa,6,GPIO_OUTPUT_INACTIVE);
    int b=gpio_pin_configure(gpioa,7,GPIO_OUTPUT_INACTIVE);
    int motor_rc=a?a:b;
#endif
    wheel_buttons_reset(&buttons);
    lights_ready=blinkers_init()==0;
    initialized=servo_prepare()==0 && motor_rc==0 && lights_ready;
    failed=!initialized;
    if(failed)zone=ZONE_STATE_SYS_ERROR;
    pi_link_init();
    printk("WHEEL STEERING: UART raw int32; PA8 PWM %u/%u/%u us. Motor drive disabled at startup.\n",
           SERVO_PULSE_LEFT_US,SERVO_PULSE_CENTER_US,SERVO_PULSE_RIGHT_US);
    printk("PWM stays OFF until a valid wheel command. Use steering_status.\n");
    printk("LSB: left lights; RSB: right lights. Same button toggles OFF.\n");
#ifdef LAB2_WHEEL_DUAL_DRIVE
    printk("DUAL mean-speed control: both encoders, both motors. Release pedals and explicitly enable.\n");
#endif
    return 0;
}
