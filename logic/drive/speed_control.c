#include "speed_control.h"
#include "speed_pi.h"
#include "encoder.h"
#include "brake.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static K_MUTEX_DEFINE(drive_lock);
static bool initialized;
static struct speed_status status;
static struct speed_pi controller;
static uint64_t last_us;
static uint32_t no_motion_ms[2];

static uint64_t now_us(void){return k_ticks_to_us_floor64(k_uptime_ticks());}
static void reset_loop(void){
    speed_pi_reset(&controller);
    no_motion_ms[0]=no_motion_ms[1]=0;
    last_us=now_us();
}
static int target_from_throttle(uint16_t p){
    return p<=20 ? 0 : (int32_t)((uint32_t)(p-20)*SPEED_FULL_THROTTLE_RPM*1000U/980U);
}
static int brake_locked(void){
    int rc=motor_brake_all();
    status.duty_percent=0;reset_loop();
    if(rc)status.fault=SPEED_FAULT_OUTPUT;
    return rc;
}
static int fail_locked(enum speed_fault why){
    status.fault=why;status.brake=true;
    int rc=brake_locked();
    return rc?rc:-EIO;
}
static int update_locked(void){
    if(status.fault || status.brake)return brake_locked();
    status.target_rpm_milli=target_from_throttle(status.throttle_permille);
    if(status.target_rpm_milli==0){
        /* Healthy zero throttle means coast, not dynamic brake. */
        reset_loop();status.duty_percent=0;
        int a=sys_motor_set(LEFT_MOTOR,0,FREE);
        int b=sys_motor_set(RIGHT_MOTOR,0,FREE);
        return (a||b)?fail_locked(SPEED_FAULT_OUTPUT):0;
    }
    struct encoder_snapshot sample;
    if(encoders_get(&sample) || !sample.valid)return fail_locked(SPEED_FAULT_ENCODER);
    status.feedback_rpm_milli=sample.average_rpm_milli;
    /* Positive commands must not accelerate a backwards/mis-signed wheel. */
    if(sample.wheel[0].rpm_milli < -5000 || sample.wheel[1].rpm_milli < -5000)
        return fail_locked(SPEED_FAULT_DIRECTION);
    uint64_t t=now_us(),dt=t-last_us;
    /* A same-tick input updates P/feedforward without integrating extra time. */
    if(dt>250000)return fail_locked(SPEED_FAULT_ENCODER);
    last_us=t;
    uint16_t duty;
    if(speed_pi_update(&controller,status.target_rpm_milli,status.feedback_rpm_milli,(uint32_t)dt,&duty))
        return fail_locked(SPEED_FAULT_ENCODER);
    status.duty_percent=(duty+5U)/10U;
    /* Bench guard, not a Lab2 current-sensor protection requirement. */
    for(unsigned i=0;i<2;i++){
        int32_t rpm=sample.wheel[i].rpm_milli;
        if(status.duty_percent>=20 && rpm>-1000 && rpm<1000)
            no_motion_ms[i]+=(uint32_t)(dt/1000);
        else no_motion_ms[i]=0;
        if(no_motion_ms[i]>=1000)return fail_locked(SPEED_FAULT_NO_MOTION);
    }
    int a=sys_motor_set(LEFT_MOTOR,status.duty_percent,FORWARD);
    int b=sys_motor_set(RIGHT_MOTOR,status.duty_percent,FORWARD);
    return (a||b)?fail_locked(SPEED_FAULT_OUTPUT):0;
}
static void loop(void *a,void *b,void *c){
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    for(;;){
        k_sleep(K_MSEC(SPEED_LOOP_PERIOD_MS));
        k_mutex_lock(&drive_lock,K_FOREVER);
        if(status.armed)(void)update_locked();
        k_mutex_unlock(&drive_lock);
    }
}
K_THREAD_DEFINE(speed_tid,1536,loop,NULL,NULL,NULL,6,0,K_TICKS_FOREVER);

int speed_control_init(void){
    k_mutex_lock(&drive_lock,K_FOREVER);
    if(initialized){k_mutex_unlock(&drive_lock);return -EALREADY;}
    int rc=motor_init();
    if(!rc){status.brake=true;rc=brake_locked();}
    if(!rc)initialized=true;
    k_mutex_unlock(&drive_lock);
    if(!rc)k_thread_start(speed_tid);
    return rc;
}
int speed_control_arm(void){
    k_mutex_lock(&drive_lock,K_FOREVER);
    struct encoder_snapshot e;
    int rc=initialized?encoders_get(&e):-EACCES;
    if(!rc && !e.valid)rc=-EAGAIN;
    if(!rc && status.armed)rc=-EALREADY;
    if(!rc){
        status.fault=SPEED_FAULT_NONE;status.throttle_permille=0;
        status.target_rpm_milli=0;status.brake=false;status.armed=true;
        reset_loop();rc=update_locked();
    }
    k_mutex_unlock(&drive_lock);return rc;
}
int speed_control_disarm(void){
    k_mutex_lock(&drive_lock,K_FOREVER);
    if(!initialized){k_mutex_unlock(&drive_lock);return -EACCES;}
    status.armed=false;status.brake=true;status.throttle_permille=0;status.target_rpm_milli=0;
    int rc=brake_locked();
    k_mutex_unlock(&drive_lock);return rc;
}
int speed_control_set_input(uint16_t throttle,bool brake){
    if(throttle>1000){(void)speed_control_set_fault();return -EINVAL;}
    k_mutex_lock(&drive_lock,K_FOREVER);
    if(!initialized||!status.armed){k_mutex_unlock(&drive_lock);return -EACCES;}
    if(status.fault){k_mutex_unlock(&drive_lock);return -EIO;}
    status.throttle_permille=throttle;status.brake=brake;
    int rc=update_locked(); /* Brake applies here, not after a 10ms sleep. */
    k_mutex_unlock(&drive_lock);return rc;
}
int speed_control_set_brake(bool brake){
    k_mutex_lock(&drive_lock,K_FOREVER);
    if(!initialized){k_mutex_unlock(&drive_lock);return -EACCES;}
    if(!brake && (!status.armed || status.fault)){k_mutex_unlock(&drive_lock);return -EACCES;}
    status.brake=brake;
    int rc=brake?brake_locked():update_locked();
    k_mutex_unlock(&drive_lock);return rc;
}
int speed_control_set_fault(void){
    k_mutex_lock(&drive_lock,K_FOREVER);
    int rc=initialized?fail_locked(SPEED_FAULT_EXTERNAL):-EACCES;
    k_mutex_unlock(&drive_lock);return rc;
}
int speed_control_get(struct speed_status *out){
    if(!out)return -EINVAL;
    k_mutex_lock(&drive_lock,K_FOREVER);*out=status;
    int rc=initialized?0:-EACCES;k_mutex_unlock(&drive_lock);return rc;
}
int speed_control_manual_init(void){
    k_mutex_lock(&drive_lock,K_FOREVER);
    int rc=status.armed?-EBUSY:motor_init();
    k_mutex_unlock(&drive_lock);return rc;
}
int speed_control_manual_set(motor_mapping_t m,uint32_t duty,direction_t dir){
    k_mutex_lock(&drive_lock,K_FOREVER);
    int rc;
    if ((unsigned)m >= NUM_MOTORS) rc=-EINVAL;
    else if (status.armed && dir==STOP) { status.brake=true; rc=brake_locked(); }
    else rc=status.armed?-EBUSY:sys_motor_set(m,duty,dir);
    k_mutex_unlock(&drive_lock);return rc;
}
