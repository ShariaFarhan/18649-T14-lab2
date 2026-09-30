#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "right_speed_model.h"
static const int32_t rpm[]={40174,63360,96418,139118,183195,226583,264692,295224,314508,320018};
static double plant(double duty){
    if(duty<=0)return 0;
    if(duty>=1000)return 320.018;
    unsigned i=(unsigned)duty/100;
    double low=i?rpm[i-1]/1000.:0,high=rpm[i]/1000.;
    return low+(high-low)*(duty-i*100)/100;
}
int main(void){
    assert(right_pedal_permille(65535)==0&&right_pedal_permille(0)==1000);
    assert(right_pedal_permille(-1)<0&&right_pedal_permille(65536)<0);
    assert(right_target_mrpm(20)==0&&right_target_mrpm(1000)==280000);
    for(unsigned i=0;i<10;i++)assert(right_feedforward(rpm[i])==(i+1)*100);
    struct right_pi s={0};uint16_t d=0;
    assert(right_pi_update(&s,280001,0,10000,&d)<0);
    for(int target=50;target<=280;target+=115){
        double actual=0;s.integral_micro=0;
        for(int i=0;i<4000;i++){
            assert(right_pi_update(&s,target*1000,(int32_t)(actual*1000),10000,&d)==0);
            double expected=plant(d)-(i>=2000?15:0);if(expected<0)expected=0;
            actual+=(expected-actual)*.01/.25;
        }
        assert(fabs(actual-target)<1.5);
    }
    assert(right_pi_update(&s,0,100000,10000,&d)==0&&d==0&&s.integral_micro==0);
    puts("PASS: measured feedforward, pedal range, target limit and synthetic load recovery (not hardware tuning)");
}
