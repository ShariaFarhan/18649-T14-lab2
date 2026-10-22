#include <assert.h>
#include <stdio.h>
#include "../stm32/lab3_app/src/drive_policy.h"
int main(void)
{
    for (unsigned t=0;t<=100;t++) {
        struct lab3_drive_output o=lab3_drive_decide(t,0,true);
        assert(o.duty==t);
        assert(o.mode==(t ? LAB3_DRIVE_FORWARD : LAB3_DRIVE_COAST));
        for (unsigned b=1;b<=100;b++) {
            o=lab3_drive_decide(t,b,true);
            assert(o.mode==LAB3_DRIVE_BRAKE && o.duty==100);
        }
        o=lab3_drive_decide(t,0,false);
        assert(o.mode==LAB3_DRIVE_BRAKE && o.duty==100);
    }
    assert(lab3_drive_decide(101,0,true).mode==LAB3_DRIVE_BRAKE);
    assert(lab3_drive_decide(0,101,true).mode==LAB3_DRIVE_BRAKE);
    puts("PASS: all pedal percentages, brake overrides throttle, coast, error and invalid input");
}
