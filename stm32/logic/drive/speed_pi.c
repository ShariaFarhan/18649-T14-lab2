#include "speed_pi.h"
#include <errno.h>
#include <stddef.h>
#define NO_LOAD_MRPM 110000LL
#define KP 4LL /* permille duty / RPM = 0.4 percent / RPM */
#define KI 8LL /* permille duty / (RPM*s) = 0.8 percent / (RPM*s) */
static int64_t clamp(int64_t v,int64_t low,int64_t high){
    return v<low?low:(v>high?high:v);
}
void speed_pi_reset(struct speed_pi *s){ s->integral_micro=0; }
int speed_pi_update(struct speed_pi *s,int32_t target,int32_t actual,
                    uint32_t dt_us,uint16_t *duty)
{
    if(!s || !duty || dt_us>250000 ||
       target<0 || target>110000 || actual<-1000000 || actual>1000000)
        return -EINVAL;
    if(target==0){speed_pi_reset(s);*duty=0;return 0;}
    int64_t error=(int64_t)target-actual;
    int64_t candidate=clamp(s->integral_micro+error*KI*dt_us/1000,
                            -1000000000LL,1000000000LL);
    int64_t base=(int64_t)target*1000/NO_LOAD_MRPM + error*KP/1000;
    int64_t trial=base+candidate/1000000;
    /* Do not integrate farther into output saturation. */
    if(!((trial>1000 && error>0) || (trial<0 && error<0)))
        s->integral_micro=candidate;
    *duty=(uint16_t)clamp(base+s->integral_micro/1000000,0,1000);
    return 0;
}
