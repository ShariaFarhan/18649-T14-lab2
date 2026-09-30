#include "right_speed_model.h"
#include <errno.h>
#include <stddef.h>
/* Right wheel, 1320 CPR, unloaded sweep 2026-09-30; mean of each step's final 3 s. */
static const int32_t measured_mrpm[]={0,40174,63360,96418,139118,183195,226583,264692,295224,314508,320018};
int right_pedal_permille(int32_t raw)
{
    if(raw<0 || raw>65535)return -EINVAL;
    return (int)((65535LL-raw)*1000/65535);
}
int32_t right_target_mrpm(int throttle)
{
    if(throttle<=20)return 0;
    if(throttle>1000)throttle=1000;
    return (int32_t)((int64_t)(throttle-20)*RIGHT_FULL_THROTTLE_MRPM/980);
}
uint16_t right_feedforward(int32_t target)
{
    if(target<=0)return 0;
    for(unsigned i=1;i<sizeof(measured_mrpm)/sizeof(measured_mrpm[0]);i++) {
        if(target<=measured_mrpm[i])return (uint16_t)((i-1)*100+
            (int64_t)(target-measured_mrpm[i-1])*100/(measured_mrpm[i]-measured_mrpm[i-1]));
    }
    return 1000;
}
int right_pi_update(struct right_pi *s,int32_t target,int32_t actual,uint32_t dt,uint16_t *duty)
{
    if(!s || !duty || target<0 || target>RIGHT_FULL_THROTTLE_MRPM ||
       actual < -1000000 || actual>1000000 || dt>250000)return -EINVAL;
    if(!target){s->integral_micro=0;*duty=0;return 0;}
    /* Initial tuning, not inferred from the static sweep: Kp=0.1%/RPM, Ki=0.2%/(RPM*s). */
    int64_t error=(int64_t)target-actual;
    int64_t candidate=s->integral_micro+error*2*dt/1000;
    if(candidate>1000000000LL)candidate=1000000000LL;
    if(candidate< -1000000000LL)candidate= -1000000000LL;
    int64_t base=right_feedforward(target)+error/1000;
    int64_t trial=base+candidate/1000000;
    if(!((trial>1000 && error>0) || (trial<0 && error<0)))s->integral_micro=candidate;
    int64_t result=base+s->integral_micro/1000000;
    *duty=(uint16_t)(result<0?0:(result>1000?1000:result));
    return 0;
}
