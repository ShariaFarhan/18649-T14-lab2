/* One PI regulates signed mean RPM, applying the same PWM to both bridges. */
#include "dual_drive.h"
#include "right_speed_model.h" /* Right-wheel measured feedforward is an initial estimate only. */
#include "motor_control.h"
#include "encoder.h"
#include <errno.h>
#include <zephyr/kernel.h>
static K_MUTEX_DEFINE(drive_lock);
static struct dual_drive_status state;
static struct right_pi controller;
static bool ready,arm_ready,ever_armed,brake_input,was_moving;
static int throttle;
static int64_t boost_until,last_update,last_command;

static int outputs(direction_t direction,uint16_t duty)
{
    int a=sys_motor_set(LEFT_MOTOR,duty,direction);
    int b=sys_motor_set(RIGHT_MOTOR,duty,direction); /* Always attempt both. */
    return a?a:b;
}
static int stop_locked(bool brake)
{
    controller.integral_micro=0;state.boosting=false;was_moving=false;
    state.target_mrpm=0;state.duty_percent=0;state.braking=brake;
    int rc=outputs(brake?STOP:FREE,0);
    state.fault=rc!=0;
    if(rc)state.reason=DUAL_OUTPUT;
    return rc;
}
static int pause_locked(enum dual_reason reason)
{
    state.reason=reason;state.armed=false;arm_ready=false;
    (void)stop_locked(ever_armed);return -EIO;
}
static int link_stop_locked(void)
{
    return pause_locked(DUAL_LINK);
}
static void feedback_locked(const struct encoder_snapshot *sample)
{
    state.left_mrpm=sample->wheel[ENCODER_LEFT].rpm_milli;
    state.right_mrpm=sample->wheel[ENCODER_RIGHT].rpm_milli;
    state.actual_mrpm=(int32_t)(((int64_t)state.left_mrpm+state.right_mrpm)/2);
}
static int update_locked(void)
{
    int64_t now=k_uptime_get();
    if(now-last_command>=60)return link_stop_locked();
    if(brake_input)return stop_locked(true); /* Brake beats throttle and startup assist. */
    state.target_mrpm=right_target_mrpm(throttle);
    if(!state.target_mrpm)return stop_locked(false);
    struct encoder_snapshot sample;
    if(encoders_get(&sample))return pause_locked(DUAL_ENCODER_LEFT);
    if(!sample.wheel[ENCODER_LEFT].valid)return pause_locked(DUAL_ENCODER_LEFT);
    if(!sample.wheel[ENCODER_RIGHT].valid)return pause_locked(DUAL_ENCODER_RIGHT);
    feedback_locked(&sample);
    if(!was_moving){
        controller.integral_micro=0;last_update=now;
        boost_until=now+(state.actual_mrpm<5000?RIGHT_START_DURATION_MS:0);
        was_moving=true;
    }
    if(now-last_update>250)return pause_locked(DUAL_SCHEDULING);
    uint16_t permille;
    state.boosting=now<boost_until && state.actual_mrpm<state.target_mrpm;
    if(state.boosting){
        permille=right_feedforward(state.target_mrpm);
        if(permille<RIGHT_START_DUTY_PERCENT*10)permille=RIGHT_START_DUTY_PERCENT*10;
    }else{
        boost_until=0;
        if(right_pi_update(&controller,state.target_mrpm,state.actual_mrpm,
                           (uint32_t)(now-last_update)*1000,&permille))return pause_locked(DUAL_INPUT);
    }
    last_update=now;state.duty_percent=(permille+5)/10;state.braking=false;
    if(outputs(FORWARD,state.duty_percent))return pause_locked(DUAL_OUTPUT);
    return 0;
}
static void drive_loop(void *a,void *b,void *c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    for(;;){
        k_sleep(K_MSEC(10));k_mutex_lock(&drive_lock,K_FOREVER);
        if(ready){
            struct encoder_snapshot sample;
            if(!encoders_get(&sample))feedback_locked(&sample);
            if(state.armed)(void)update_locked();
        }
        k_mutex_unlock(&drive_lock);
    }
}
K_THREAD_DEFINE(drive_tid,1536,drive_loop,NULL,NULL,NULL,6,0,K_TICKS_FOREVER);
int dual_drive_init(void)
{
    int rc=motor_init();
    if(!rc)rc=outputs(FREE,0);
    if(!rc)rc=encoders_init();
    if(rc)return rc;
    ready=true;k_thread_start(drive_tid);return 0;
}
int dual_drive_command(int32_t throttle_raw,int32_t brake_raw,bool enable)
{
    k_mutex_lock(&drive_lock,K_FOREVER);
    int t=right_pedal_permille(throttle_raw),b=right_pedal_permille(brake_raw),rc=0;
    if(!ready){rc=-EACCES;goto done;}
    if(t<0 || b<0){rc=pause_locked(DUAL_INPUT);goto done;}
    last_command=k_uptime_get();throttle=t;brake_input=b>20;
    if(!enable){
        bool needs_stop=state.armed || state.duty_percent || state.fault || (ever_armed && !state.braking);
        state.armed=false;arm_ready=true;
        if(needs_stop)rc=stop_locked(ever_armed);
        goto done;
    }
    if(!state.armed){
        if(state.fault || !arm_ready || t>20 || b>20)goto done;
        struct encoder_snapshot sample;
        if(encoders_get(&sample) || !sample.wheel[ENCODER_LEFT].valid){rc=pause_locked(DUAL_ENCODER_LEFT);goto done;}
        if(!sample.wheel[ENCODER_RIGHT].valid){rc=pause_locked(DUAL_ENCODER_RIGHT);goto done;}
        state.reason=DUAL_OK;state.armed=true;ever_armed=true;arm_ready=false;was_moving=false;
    }
    rc=update_locked();
done:
    k_mutex_unlock(&drive_lock);return rc;
}
void dual_drive_fail_safe(void)
{
    k_mutex_lock(&drive_lock,K_FOREVER);
    if(ready){
        bool needs_stop=state.armed || state.duty_percent || (ever_armed && !state.braking);
        state.armed=false;arm_ready=false;
        if(!state.fault && ever_armed)state.reason=DUAL_LINK;
        if(needs_stop)(void)stop_locked(ever_armed);
    }
    k_mutex_unlock(&drive_lock);
}
struct dual_drive_status dual_drive_get(void)
{
    k_mutex_lock(&drive_lock,K_FOREVER);struct dual_drive_status copy=state;
    k_mutex_unlock(&drive_lock);return copy;
}
