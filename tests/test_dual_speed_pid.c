#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "dual_speed_pid.h"
#include "right_speed_model.h"
static double plant(unsigned d){
    const double speed[]={0,40.174,63.360,96.418,139.118,183.195,226.583,264.692,295.224,314.508,320.018};
    if(d>=1000)return speed[10];
    unsigned i=d/100;return speed[i]+(speed[i+1]-speed[i])*(d%100)/100.;
}
int main(void){
    struct dual_pid s;uint16_t d,p;
    dual_pid_reset(&s);
    assert(dual_pid_update(&s,150000,100000,10000,&d)==0);
    assert(dual_pid_update(&s,250000,100000,10000,&d)==0);
    assert(s.filtered_derivative_mrpm_s==0); /* Changing target alone cannot kick D. */
    struct right_pi pi={0};
    dual_pid_reset(&s);
    assert(dual_pid_update(&s,150000,100000,10000,&d)==0);
    assert(right_pi_update(&pi,150000,100000,10000,&p)==0);
    assert(dual_pid_update(&s,150000,120000,10000,&d)==0);
    assert(right_pi_update(&pi,150000,120000,10000,&p)==0);
    assert(d<p&&s.filtered_derivative_mrpm_s>0); /* D damps rising speed. */
    dual_pid_reset(&s);pi.integral_micro=0;
    assert(dual_pid_update(&s,150000,100000,10000,&d)==0);
    assert(right_pi_update(&pi,150000,100000,10000,&p)==0);
    assert(dual_pid_update(&s,150000,80000,10000,&d)==0);
    assert(right_pi_update(&pi,150000,80000,10000,&p)==0);
    assert(d>p&&s.filtered_derivative_mrpm_s<0); /* D assists a load-induced drop. */
    assert(dual_pid_update(&s,0,80000,10000,&d)==0&&d==0&&!s.have_previous&&s.integral_micro==0);
    assert(dual_pid_update(&s,280001,0,10000,&d)<0);
    assert(dual_pid_update(&s,100000,0,250001,&d)<0);
    for(unsigned target=100;target<=280;target+=90){
        dual_pid_reset(&s);double actual=0;
        for(unsigned i=0;i<4000;i++){
            assert(dual_pid_update(&s,target*1000,(int32_t)(actual*1000),10000,&d)==0&&d<=1000);
            double expected=plant(d)-(i>=2000?15:0);if(expected<0)expected=0;
            actual+=(expected-actual)*.01/.25;
        }
        assert(fabs(actual-target)<1.5);
    }
    puts("PASS: filtered measurement D, no target kick, load recovery, reset and limits (simulation only)");
}
