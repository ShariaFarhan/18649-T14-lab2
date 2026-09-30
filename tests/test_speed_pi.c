#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include "logic/drive/speed_pi.h"
static double simulate(struct speed_pi *s,double target,double load,int steps,double *left,double *right)
{
    uint16_t duty=0;
    for(int i=0;i<steps;i++){
        double mean=(*left+*right)/2;
        assert(speed_pi_update(s,(int32_t)(target*1000),(int32_t)(mean*1000),10000,&duty)==0);
        assert(duty<=1000);
        double l=110.0*duty/1000;
        double r=105.0*duty/1000-load;
        if(r<0)r=0;
        *left+=(l-*left)*.01/.25;
        *right+=(r-*right)*.01/.25;
    }
    return (*left+*right)/2;
}
int main(void)
{
    struct speed_pi s={0};uint16_t d;
    double left=0,right=0;
    double mean=simulate(&s,50,0,2000,&left,&right);
    assert(fabs(mean-50)<1.0);
    mean=simulate(&s,50,20,2000,&left,&right);
    assert(fabs(mean-50)<1.0); /* one wheel loaded; mean recovers */
    puts("PASS: synthetic two-wheel mean-speed and load-step recovery");
    speed_pi_reset(&s);
    for(int i=0;i<10000;i++)assert(speed_pi_update(&s,85000,0,10000,&d)==0&&d<=1000);
    assert(s.integral_micro==0); /* output saturates; no windup */
    assert(speed_pi_update(&s,0,85000,10000,&d)==0&&d==0&&s.integral_micro==0);
    assert(speed_pi_update(&s,85000,400000,10000,&d)==0&&d==0);
    assert(speed_pi_update(&s,85000,0,0,&d)==0); /* same-tick P update */
    assert(speed_pi_update(&s,85000,0,250001,&d)==-EINVAL);
    assert(speed_pi_update(&s,-1,0,10000,&d)==-EINVAL);
    assert(speed_pi_update(&s,110001,0,10000,&d)==-EINVAL);
    assert(speed_pi_update(&s,1000,2000000,10000,&d)==-EINVAL);
    puts("PASS: bounds, integral reset, anti-windup, invalid sample rejection");
    return 0;
}
