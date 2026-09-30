/* Real wheel -> Pi -> raw UART -> steering. Motor outputs stay disabled. */
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/gpio.h>
#include <stm32_ll_tim.h>
#include "pi_link.h"
#include "servo.h"
#include "wheel_mapping.h"

static K_MUTEX_DEFINE(steering_lock);
static bool initialized, active, failed;
static uint8_t zone = ZONE_STATE_INIT;
static int32_t raw_steering = WHEEL_RAW_CENTER;
static int16_t steering_percent;

static void safe_locked(uint8_t reason)
{
    if (initialized && active && servo_center()) failed = true;
    steering_percent = 0;
    zone = failed ? ZONE_STATE_SYS_ERROR : reason;
}
bool pi_link_command_is_fresh(const cmd_payload_t *cmd, bool plausible)
{
    return plausible && cmd->flags == 0;
}
void pi_link_on_command_received(const cmd_payload_t *cmd, bool plausible)
{
    k_mutex_lock(&steering_lock, K_FOREVER);
    int16_t percent;
    if (!plausible || (cmd->flags & ~FLAG_UDP_STALE) || wheel_steering_percent(cmd->steering_raw,&percent)) {
        safe_locked(ZONE_STATE_SYS_ERROR);
    } else if (cmd->flags & FLAG_UDP_STALE) {
        safe_locked(active ? ZONE_STATE_LINK_LOST : ZONE_STATE_INIT);
    } else if (!initialized || failed) {
        safe_locked(ZONE_STATE_SYS_ERROR);
    } else {
        raw_steering = cmd->steering_raw;
        steering_percent = percent;
        if (servo_set_steering(percent)) { failed=true; safe_locked(ZONE_STATE_SYS_ERROR); }
        else { active=true; zone=ZONE_STATE_NORMAL; }
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
    hb->bus_degraded=0xB4; /* Steering-test telemetry, not measured currents. */
    hb->current_motor_l=(uint16_t)raw_steering;
    hb->current_motor_r=(uint16_t)(steering_percent+100);
    hb->current_servo=(uint16_t)servo_get_last_pulse_us();
    k_mutex_unlock(&steering_lock);
}
static int status_cmd(const struct shell *sh,size_t argc,char **argv)
{
    ARG_UNUSED(argc);ARG_UNUSED(argv);
    k_mutex_lock(&steering_lock,K_FOREVER);
    int32_t raw=raw_steering;int16_t p=steering_percent;uint8_t z=zone;
    uint32_t pulse=servo_get_last_pulse_us(),compare=LL_TIM_OC_GetCompareCH1(TIM1);
    k_mutex_unlock(&steering_lock);
    shell_print(sh,"Wheel steering: state=%u raw=%d percent=%d pulse_us=%u TIM1_compare=%u motor_EN=%u/%u",
                z,raw,p,pulse,compare,!!(GPIOA->ODR & BIT(6)),!!(GPIOA->ODR & BIT(7)));
    return 0;
}
SHELL_CMD_REGISTER(steering_status,NULL,"Read raw wheel / steering PWM status",status_cmd);
int main(void)
{
    const struct device *gpioa=DEVICE_DT_GET(DT_NODELABEL(gpioa));
    int a=gpio_pin_configure(gpioa,6,GPIO_OUTPUT_INACTIVE);
    int b=gpio_pin_configure(gpioa,7,GPIO_OUTPUT_INACTIVE);
    initialized=servo_prepare()==0 && a==0 && b==0;
    failed=!initialized;
    if(failed)zone=ZONE_STATE_SYS_ERROR;
    pi_link_init();
    printk("WHEEL STEERING: UART raw int32; PA8 PWM %u/%u/%u us. Motor EN outputs held LOW.\n",
           SERVO_PULSE_LEFT_US,SERVO_PULSE_CENTER_US,SERVO_PULSE_RIGHT_US);
    printk("PWM stays OFF until a valid wheel command. Use steering_status.\n");
    return 0;
}
