/* Right-only wheel bring-up. Never commands propulsion or brake on LEFT_MOTOR. */
#include "right_drive.h"
#include "right_speed_model.h"
#include "motor_control.h"
#include "encoder.h"
#include <errno.h>
#include <zephyr/kernel.h>
static K_MUTEX_DEFINE(right_lock);
static struct right_drive_status state;
static struct right_pi controller;
static bool ready,arm_ready,ever_armed,brake_input,was_moving;
static int throttle;
static int64_t boost_until,last_update,last_motion,last_count,last_command;
static bool have_count;

static int stop_locked(bool brake)
{
    controller.integral_micro=0;state.boosting=false;was_moving=false;
    state.target_mrpm=0;state.duty_percent=0;state.braking=brake;
    int rc=sys_motor_set(RIGHT_MOTOR,0,brake?STOP:FREE);
    if(rc)state.fault=true;
    return rc;
}
static int fault_locked(void)
{
    state.fault=true;state.armed=false;arm_ready=false;
    (void)stop_locked(ever_armed);
    return -EIO;
}
static int update_locked(void)
{
    int64_t now=k_uptime_get();
    if(now-last_command>=60)return fault_locked();
    /* Brake is applied before any throttle/PI/startup-assist decision. */
    if(brake_input)return stop_locked(true);
    state.target_mrpm=right_target_mrpm(throttle);
    if(!state.target_mrpm)return stop_locked(false);
    struct encoder_snapshot sample;
    if(encoders_get(&sample) || !sample.wheel[ENCODER_RIGHT].valid)return fault_locked();
    const struct encoder_sample *right=&sample.wheel[ENCODER_RIGHT];
    state.actual_mrpm=right->rpm_milli;
    if(state.actual_mrpm< -5000)return fault_locked();
    if(!was_moving){
        controller.integral_micro=0;last_update=now;last_motion=now;
        last_count=right->count;have_count=true;
        boost_until=now+(state.actual_mrpm<5000?RIGHT_START_DURATION_MS:0);
        was_moving=true;
    }
    if(!have_count || right->count!=last_count){last_motion=now;last_count=right->count;have_count=true;}
    if(now-last_motion>=1000 || now-last_update>250)return fault_locked();
    uint16_t permille;
    state.boosting=now<boost_until && state.actual_mrpm<state.target_mrpm;
    if(state.boosting){
        permille=right_feedforward(state.target_mrpm);
        if(permille<RIGHT_START_DUTY_PERCENT*10)permille=RIGHT_START_DUTY_PERCENT*10;
    }else{
        boost_until=0; /* Do not restart the pulse when measured speed falls. */
        if(right_pi_update(&controller,state.target_mrpm,state.actual_mrpm,
                           (uint32_t)(now-last_update)*1000,&permille))return fault_locked();
    }
    last_update=now;
    state.duty_percent=(permille+5)/10;state.braking=false;
    if(sys_motor_set(RIGHT_MOTOR,state.duty_percent,FORWARD))return fault_locked();
    return 0;
}
static void right_loop(void *a,void *b,void *c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    for(;;){
        k_sleep(K_MSEC(10));k_mutex_lock(&right_lock,K_FOREVER);
        if(ready){
            struct encoder_snapshot sample;
            if(!encoders_get(&sample))state.actual_mrpm=sample.wheel[ENCODER_RIGHT].rpm_milli;
            if(state.armed)(void)update_locked();
        }
        k_mutex_unlock(&right_lock);
    }
}
K_THREAD_DEFINE(right_tid,1536,right_loop,NULL,NULL,NULL,6,0,K_TICKS_FOREVER);
int right_drive_init(void)
{
    int rc=motor_init(); /* Both bridges initially FREE: all EN/direction outputs LOW. */
    if(!rc)rc=sys_motor_set(LEFT_MOTOR,0,FREE);
    if(!rc)rc=sys_motor_set(RIGHT_MOTOR,0,FREE);
    if(!rc)rc=encoders_init();
    if(rc)return rc;
    ready=true;k_thread_start(right_tid);return 0;
}
int right_drive_command(int32_t throttle_raw,int32_t brake_raw,bool enable)
{
    k_mutex_lock(&right_lock,K_FOREVER);
    int t=right_pedal_permille(throttle_raw),b=right_pedal_permille(brake_raw),rc=0;
    if(!ready){rc=-EACCES;goto done;}
    if(t<0 || b<0){rc=fault_locked();goto done;}
    last_command=k_uptime_get();throttle=t;brake_input=b>20;
    if(!enable){
        bool needs_stop=state.armed || state.duty_percent || (ever_armed && !state.braking);
        state.armed=false;arm_ready=true;
        if(needs_stop)rc=stop_locked(ever_armed);
        goto done;
    }
    if(!state.armed){
        if(!arm_ready || t>20 || b>20)goto done; /* Release pedals, then explicitly enable. */
        struct encoder_snapshot sample;
        if(encoders_get(&sample) || !sample.wheel[ENCODER_RIGHT].valid){rc=fault_locked();goto done;}
        state.fault=false;state.armed=true;ever_armed=true;arm_ready=false;was_moving=false;
    }
    rc=update_locked();
done:
    k_mutex_unlock(&right_lock);return rc;
}
void right_drive_fail_safe(void)
{
    k_mutex_lock(&right_lock,K_FOREVER);
    if(ready){
        bool needs_stop=state.armed || state.duty_percent || (ever_armed && !state.braking);
        state.armed=false;arm_ready=false;
        if(needs_stop)(void)stop_locked(ever_armed);
    }
    k_mutex_unlock(&right_lock);
}
struct right_drive_status right_drive_get(void)
{
    k_mutex_lock(&right_lock,K_FOREVER);struct right_drive_status copy=state;
    k_mutex_unlock(&right_lock);return copy;
}
