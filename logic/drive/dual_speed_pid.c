#include "dual_speed_pid.h"
#include <errno.h>
#include <stddef.h>
#include <string.h>



/* Right wheel, 1320 CPR, unloaded sweep 2026-09-30; mean of each step's final 3 s. */
static const int32_t measured_mrpm[]={0,40174,63360,96418,139118,183195,226583,264692,295224,314508,320018};

/* Converts inverted 16-bit raw ADC reading into normalized 0–1000 per-mille throttle position. */
int pedal_permille(int32_t raw)
{
    if(raw<0 || raw>65535)return -EINVAL;
    return (int)((65535LL-raw)*1000/65535);
}

/* Map normalized throttle position to target wheel speed in milli-RPM, applying 2 pecent deadband. */
int32_t target_mrpm(int throttle)
{
    if(throttle<=20)return 0;
    if(throttle>1000)throttle=1000;
    return (int32_t)((int64_t)(throttle-20)*DUAL_FULL_THROTTLE_MRPM/980);
}

uint16_t feedforward(int32_t target)
{
    if (target<=0) return 0;

    for(unsigned i=1; i<sizeof(measured_mrpm)/sizeof(measured_mrpm[0]); i++) {
        if(target<=measured_mrpm[i])
            return (uint16_t)((i-1)*100+(int64_t)(target-measured_mrpm[i-1])*100/(measured_mrpm[i]-measured_mrpm[i-1]));
    }
    return 1000;
}



static int64_t limit(int64_t v,int64_t lo,int64_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void dual_pid_reset(struct dual_pid *s)
{
    memset(s,0,sizeof(*s));
}

int dual_pid_update(struct dual_pid *s,int32_t target,int32_t actual,uint32_t dt,uint16_t *duty)
{
    if(!s || !duty || target<0 || target>DUAL_FULL_THROTTLE_MRPM ||
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
    int64_t base=feedforward(target)+error/1000+derivative_duty;
    int64_t trial=base+candidate/1000000;
    if(!((trial>1000 && error>0) || (trial<0 && error<0)))s->integral_micro=candidate;
    *duty=(uint16_t)limit(base+s->integral_micro/1000000,0,1000);
    return 0;
}