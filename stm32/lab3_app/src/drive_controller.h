#ifndef LAB3_DRIVE_CONTROLLER_H
#define LAB3_DRIVE_CONTROLLER_H
#include "drive_policy.h"
#include "dual_speed_pid.h"
/* Bench plant, NOT encoder data: 200 ms response, 320 RPM at full duty. */
struct lab3_drive_controller {
    struct dual_pid pid;
    struct lab3_drive_output output;
    int32_t target_mrpm,model_mrpm;
    uint64_t boost_until;
    bool running;
};
static inline int lab3_controller_command(struct lab3_drive_controller *s,
        uint8_t throttle,uint8_t brake,bool normal,uint64_t now,uint32_t dt_us)
{
    struct lab3_drive_output requested=lab3_drive_decide(throttle,brake,normal);
    int32_t target=target_mrpm(throttle*10);
    if (requested.mode!=LAB3_DRIVE_FORWARD || !target) {
        dual_pid_reset(&s->pid);s->running=false;s->target_mrpm=0;
        if (requested.mode==LAB3_DRIVE_FORWARD) requested=(struct lab3_drive_output){LAB3_DRIVE_COAST,0};
        s->output=requested;return 0;
    }
    if (!s->running) {
        dual_pid_reset(&s->pid);s->running=true;
        s->boost_until=now+(s->model_mrpm<5000 ? DUAL_START_DURATION_MS : 0);
    }
    s->target_mrpm=target;
    uint16_t permille;
    if (now<s->boost_until && s->model_mrpm<target) {
        permille=feedforward(target);
        if (permille<DUAL_START_DUTY_PERCENT*10) permille=DUAL_START_DUTY_PERCENT*10;
    } else {
        s->boost_until=0;
        int rc=dual_pid_update(&s->pid,target,s->model_mrpm,dt_us,&permille);
        if (rc) return rc;
    }
    s->output=(struct lab3_drive_output){LAB3_DRIVE_FORWARD,(permille+5)/10};
    return 0;
}
static inline void lab3_model_step(struct lab3_drive_controller *s,uint32_t dt_ms)
{
    int32_t demand=s->output.mode==LAB3_DRIVE_FORWARD ? s->output.duty*3200 : 0;
    uint32_t tau=s->output.mode==LAB3_DRIVE_BRAKE ? 50 : 200;
    s->model_mrpm+=(int32_t)(((int64_t)demand-s->model_mrpm)*dt_ms/(tau+dt_ms));
}
#endif
