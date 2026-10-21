/* Dual PID, steering and lights with one system-wide error-state owner. */
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stm32_ll_tim.h>
#include "pi_link.h"
#include "servo.h"
#include "wheel_mapping.h"
#include "wheel_buttons.h"
#include "blinkers.h"
#include "dual_drive.h"
#include "current_sense.h"
#include "system_state.h"
#define ALLOWED_FLAGS (FLAG_UDP_STALE | FLAG_DUAL_ENABLE)

static K_MUTEX_DEFINE(steering_lock);
static struct system_state system;
static struct wheel_buttons buttons;
static bool servo_ready,lights_ready,active;
static int32_t raw_steering=WHEEL_RAW_CENTER;
static int16_t steering_percent;

static uint16_t current_magnitude(enum current_sense_channel channel)
{
    struct current_sense_sample sample;
    if(current_sense_read(channel,&sample) || !sample.zero_valid)return UINT16_MAX;
    int64_t magnitude=sample.current_ma;
    if(magnitude<0)magnitude=-magnitude;
    return magnitude>UINT16_MAX-1 ? UINT16_MAX-1 : (uint16_t)magnitude;
}
void pi_link_get_currents(uint16_t *left,uint16_t *right,uint16_t *servo)
{
    *left=current_magnitude(CURRENT_SENSE_LEFT_MOTOR);
    *right=current_magnitude(CURRENT_SENSE_RIGHT_MOTOR);
    *servo=current_magnitude(CURRENT_SENSE_SERVO);
}

static uint8_t zone_for(enum system_reason reason)
{
    return reason==SYS_OK ? ZONE_STATE_NORMAL :
        reason==SYS_SELF_TEST ? ZONE_STATE_LOCAL_FAIL :
        reason==SYS_LINK_LOST ? ZONE_STATE_LINK_LOST : ZONE_STATE_SYS_ERROR;
}
static void error_outputs_locked(void)
{
    dual_drive_fail_safe();
    if(dual_drive_get().fault)system_state_output_failed(&system);
    if(servo_ready && active && servo_center())system_state_output_failed(&system);
    if(lights_ready && blinkers_get_mode()!=BLINKER_MODE_HAZARD && blinkers_set_error(true))
        system_state_output_failed(&system);
    wheel_buttons_reset(&buttons);steering_percent=0;
}
static bool well_formed(const cmd_payload_t *cmd,bool plausible)
{
    return plausible && !(cmd->flags & ~ALLOWED_FLAGS) && cmd->test_counter<=31 && !(cmd->test_flags & ~1U);
}
bool pi_link_command_is_fresh(const cmd_payload_t *cmd,bool plausible)
{
    return well_formed(cmd,plausible) && !(cmd->flags & FLAG_UDP_STALE);
}
static void apply_locked(const cmd_payload_t *cmd,int16_t percent)
{
    /* Brake/motor command precedes steering and ordinary turn signals. */
    if(dual_drive_command(cmd->throttle_raw,cmd->brake_raw,cmd->flags & FLAG_DUAL_ENABLE)){
        system_state_output_failed(&system);error_outputs_locked();return;
    }
    raw_steering=cmd->steering_raw;steering_percent=percent;
    if(servo_set_steering(percent)){
        system_state_output_failed(&system);error_outputs_locked();return;
    }
    active=true;
    uint16_t press=wheel_buttons_pressed(&buttons,cmd->buttons,k_uptime_get());
    int rc=0;
    if(press){
        enum blinker_mode selected=press==WHEEL_LSB_MASK ? BLINKER_MODE_LEFT : BLINKER_MODE_RIGHT;
        rc=blinkers_set_mode(blinkers_get_mode()==selected ? BLINKER_MODE_OFF : selected);
    }
    if(!rc)rc=blinkers_update_steering(percent);
    if(rc){system_state_output_failed(&system);error_outputs_locked();}
}
void pi_link_on_command_received(const cmd_payload_t *cmd,bool plausible)
{
    k_mutex_lock(&steering_lock,K_FOREVER);
    enum system_reason before=system_state_reason(&system);
    int16_t percent=0;
    if(!well_formed(cmd,plausible) || wheel_steering_percent(cmd->steering_raw,&percent)){
        system_state_bad_input(&system);
    }else{
        bool sync=(cmd->test_flags&1U) || !system.button_synced;
        if(sync)system_state_sync_button(&system,cmd->test_counter);
        if(cmd->flags & FLAG_UDP_STALE){
            system_state_link_lost(&system);
        }else{
            system_state_valid(&system);
            if(!sync)(void)system_state_button(&system,cmd->test_counter,(uint64_t)k_uptime_get());
        }
    }
    if(system_state_reason(&system)==SYS_OK){
        if(before!=SYS_OK && lights_ready && blinkers_set_error(false))
            system_state_output_failed(&system);
        if(system_state_reason(&system)==SYS_OK)apply_locked(cmd,percent);
        else error_outputs_locked();
    }else error_outputs_locked();
    k_mutex_unlock(&steering_lock);
}
void pi_link_on_link_restored(void) { /* Only a valid command can release error outputs. */ }
void pi_link_on_link_lost(void)
{
    k_mutex_lock(&steering_lock,K_FOREVER);
    system_state_link_lost(&system);error_outputs_locked();
    k_mutex_unlock(&steering_lock);
}
void pi_link_on_invalid_frame(void)
{
    k_mutex_lock(&steering_lock,K_FOREVER);
    system_state_bad_input(&system);error_outputs_locked();
    k_mutex_unlock(&steering_lock);
}
void pi_link_customize_heartbeat(hb_payload_t *hb)
{
    k_mutex_lock(&steering_lock,K_FOREVER);
    struct dual_drive_status drive=dual_drive_get();
    if(drive.fault && !system.output_fault){
        system_state_output_failed(&system);error_outputs_locked();drive=dual_drive_get();
    }
    enum system_reason reason=system_state_reason(&system);
    hb->zone_state=zone_for(reason);hb->bus_degraded=0xBA;
    hb->dual_steering_raw=(uint16_t)raw_steering;
    hb->dual_steering_packed=(uint16_t)(steering_percent+100) |
        ((uint16_t)blinkers_get_mode()<<8) | ((uint16_t)((GPIOC->ODR>>6)&15U)<<10);
    hb->dual_servo_pulse_us=(uint16_t)servo_get_last_pulse_us();
    hb->dual_target_mrpm=drive.target_mrpm;
    hb->dual_left_mrpm=drive.left_mrpm;hb->dual_right_mrpm=drive.right_mrpm;
    hb->dual_duty_percent=drive.duty_percent;
    hb->dual_flags=drive.armed | (drive.braking<<1) | (drive.fault<<2) | (drive.boosting<<3) | ((uint8_t)drive.reason<<4);
    hb->test_counter_ack=system.test_counter;
    hb->system_reason=(uint8_t)reason;
    k_mutex_unlock(&steering_lock);
}
static int status_cmd(const struct shell *sh,size_t argc,char **argv)
{
    ARG_UNUSED(argc);ARG_UNUSED(argv);
    k_mutex_lock(&steering_lock,K_FOREVER);
    enum system_reason reason=system_state_reason(&system);
    int32_t raw=raw_steering;int16_t percent=steering_percent;
    uint32_t pulse=servo_get_last_pulse_us(),compare=LL_TIM_OC_GetCompareCH1(TIM1);
    unsigned light_mode=blinkers_get_mode(),pins=(GPIOC->ODR>>6)&15U;
    struct dual_drive_status drive=dual_drive_get();
    unsigned counter=system.test_counter;
    k_mutex_unlock(&steering_lock);
    shell_print(sh,"SYSTEM %s; test_counter=%u; steer raw=%d percent=%d pulse_us=%u TIM1_compare=%u",
                system_reason_name(reason),counter,raw,percent,pulse,compare);
    shell_print(sh,"Lights mode=%u GPIO_PC6_PC9=0x%x; LSB bit9 / RSB bit8",light_mode,pins);
    shell_print(sh,"DUAL armed=%d brake=%d fault=%d target_mrpm=%d average_mrpm=%d PWM=%u%%",
                drive.armed,drive.braking,drive.fault,drive.target_mrpm,drive.actual_mrpm,drive.duty_percent);
    return 0;
}
SHELL_CMD_REGISTER(steering_status,NULL,"Read system state and actuator status",status_cmd);
int main(void)
{
    int motor_rc=dual_drive_init();
    lights_ready=blinkers_init()==0;
    servo_ready=servo_prepare()==0;
    wheel_buttons_reset(&buttons);
    system_state_init(&system,motor_rc==0 && lights_ready && servo_ready);
    error_outputs_locked(); /* STARTUP = both motors braked + 2 Hz hazards, servo initially OFF. */
    int current_rc=current_sense_init();
    if(!current_rc)current_rc=current_sense_zero_all();
    pi_link_init();
    printk("DUAL PID + ERROR STATE: boot braked/hazards; valid stream restores normal.\n");
    printk("Wheel self-test: single press fail, double press <=500 ms recover; debounce 50 ms.\n");
    printk("Use steering_status. PA8 PWM %u/%u/%u us.\n",SERVO_PULSE_LEFT_US,SERVO_PULSE_CENTER_US,SERVO_PULSE_RIGHT_US);
    printk("Current sensing PC2/PC3/PC4 startup calibration: %d (65535 means unavailable).\n",current_rc);
    return 0;
}
