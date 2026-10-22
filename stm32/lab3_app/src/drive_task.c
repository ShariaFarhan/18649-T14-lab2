#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "motor_control.h"
#include "drive_task.h"
#include "drive_controller.h"
#include "runtime.h"
K_MUTEX_DEFINE(drive_lock);
K_SEM_DEFINE(drive_event,0,1);
static struct lab3_drive_controller controller;
static uint8_t throttle_target,brake_target;
static bool submitted,normal_target;
static uint32_t generation_target,command_at;
static atomic_t failed,reported;
static int apply(struct lab3_drive_output out)
{
    direction_t direction=out.mode==LAB3_DRIVE_BRAKE ? STOP :
                          out.mode==LAB3_DRIVE_FORWARD ? FORWARD : FREE;
    int left=sys_motor_set(LEFT_MOTOR,out.duty,direction);
    int right=sys_motor_set(RIGHT_MOTOR,out.duty,direction);
    return left ? left : right;
}
static void update_locked(uint32_t dt_ms)
{
    if (atomic_get(&failed)) return;
    uint32_t now=k_uptime_get_32();
    bool normal=normal_target && !lab3_local_failed() && now-command_at<100;
    if (dt_ms) lab3_model_step(&controller,dt_ms);
    int rc=lab3_controller_command(&controller,throttle_target,brake_target,normal,now,dt_ms*1000);
    if (!rc) rc=apply(controller.output);
    if (rc) {
        atomic_set(&failed,1);
        (void)sys_motor_set(LEFT_MOTOR,0,FREE);
        (void)sys_motor_set(RIGHT_MOTOR,0,FREE);
        atomic_set(&reported,0x200);
    } else atomic_set(&reported,controller.output.duty |
                          (controller.output.mode==LAB3_DRIVE_BRAKE ? 0x100 : 0));
}
static void drive_worker(void *a,void *b,void *c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    for (;;) {
        if (!k_sem_take(&drive_event,K_MSEC(20))) {
            lab3_profile_begin(PROFILE_DRIVE);
            k_mutex_lock(&drive_lock,K_FOREVER);
            update_locked(0);
            k_mutex_unlock(&drive_lock);
            lab3_profile_end(PROFILE_DRIVE);
        }
        lab3_progress(WORK_CONTROL);
    }
}
static void pid_worker(void *a,void *b,void *c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    int64_t last=k_uptime_get(),next=last+10;
    for (;;) {
        k_sleep(K_TIMEOUT_ABS_MS(next));
        int64_t now=k_uptime_get();uint32_t dt=(uint32_t)(now-last);last=now;
        next+=((now-next)/10+1)*10;
        lab3_profile_begin(PROFILE_PID);
        k_mutex_lock(&drive_lock,K_FOREVER);
        update_locked(dt);
        k_mutex_unlock(&drive_lock);
        lab3_profile_end(PROFILE_PID);
        lab3_progress(WORK_PID);
    }
}
K_THREAD_DEFINE(drive_thread,1536,drive_worker,NULL,NULL,NULL,4,0,SYS_FOREVER_MS);
K_THREAD_DEFINE(pid_thread,1536,pid_worker,NULL,NULL,NULL,5,0,SYS_FOREVER_MS);
int drive_task_init(void)
{
    int rc=motor_init();
    if (rc) { atomic_set(&failed,1);atomic_set(&reported,0x200);return rc; }
    k_thread_start(drive_thread);k_thread_start(pid_thread);
    drive_task_submit(0,0,false,0);return 0;
}
void drive_task_submit(uint8_t throttle,uint8_t brake,bool normal,uint32_t generation)
{
    k_mutex_lock(&drive_lock,K_FOREVER);
    bool changed=!submitted || normal!=normal_target || (normal && generation!=generation_target);
    if (changed) {
        throttle_target=throttle;brake_target=brake;normal_target=normal;
        generation_target=generation;submitted=true;command_at=k_uptime_get_32();
    }
    k_mutex_unlock(&drive_lock);
    if (changed) k_sem_give(&drive_event);
}
bool drive_task_failed(void) { return atomic_get(&failed)!=0; }
struct lab3_drive_status drive_task_status(void)
{
    atomic_val_t v=atomic_get(&reported);
    return (struct lab3_drive_status){.duty=v&255,.braking=!!(v&0x100),.fault=!!(v&0x200)};
}
