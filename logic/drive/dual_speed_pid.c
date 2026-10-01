#include "dual_speed_pid.h"
#include "right_speed_model.h"
#include <errno.h>
#include <stddef.h>
#include <string.h>
static int64_t limit(int64_t v,int64_t lo,int64_t hi){return v<lo?lo:(v>hi?hi:v);}
void dual_pid_reset(struct dual_pid *s){memset(s,0,sizeof(*s));}
int dual_pid_update(struct dual_pid *s,int32_t target,int32_t actual,uint32_t dt,uint16_t *duty)
{
    if(!s || !duty || target<0 || target>RIGHT_FULL_THROTTLE_MRPM ||
       actual < -1000000 || actual>1000000 || dt>250000)return -EINVAL;
    if(!target){dual_pid_reset(s);*duty=0;return 0;}
    /* Initial gains: Kp=0.1%/RPM, Ki=0.2%/(RPM*s), Kd=0.001%/(RPM/s).
     * D acts on measured speed, filtered with a 50 ms time constant, capped at 15% duty.
     * This avoids setpoint derivative kick and limits encoder quantization effects. */
    if(dt){
        if(s->have_previous){
            int64_t derivative=((int64_t)actual-s->previous_actual_mrpm)*1000000/dt;
            s->filtered_derivative_mrpm_s +=
                (derivative-s->filtered_derivative_mrpm_s)*dt/(50000+dt);
        }
        s->previous_actual_mrpm=actual;s->have_previous=true;
    }
    int64_t error=(int64_t)target-actual;
    int64_t candidate=limit(s->integral_micro+error*2*dt/1000,-1000000000LL,1000000000LL);
    int64_t derivative_duty=limit(-s->filtered_derivative_mrpm_s/100000,-150,150);
    int64_t base=right_feedforward(target)+error/1000+derivative_duty;
    int64_t trial=base+candidate/1000000;
    if(!((trial>1000 && error>0) || (trial<0 && error<0)))s->integral_micro=candidate;
    *duty=(uint16_t)limit(base+s->integral_micro/1000000,0,1000);
    return 0;
}
