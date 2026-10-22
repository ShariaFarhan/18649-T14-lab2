#include <assert.h>
#include <stdio.h>
#include "../stm32/lab3_app/src/steer_mapping.h"
int main(void)
{
    assert(lab3_steer_percent(-1000,true)==-100);
    assert(lab3_steer_percent(-500,true)==-50);
    assert(lab3_steer_percent(0,true)==0);
    assert(lab3_steer_percent(500,true)==50);
    assert(lab3_steer_percent(1000,true)==100);
    assert(lab3_steer_percent(-15,true)==-2);
    assert(lab3_steer_percent(15,true)==2);
    assert(lab3_steer_percent(-1001,true)==0);
    assert(lab3_steer_percent(1001,true)==0);
    assert(lab3_steer_percent(-1000,false)==0);
    assert(lab3_steer_percent(1000,false)==0);
    puts("PASS: CAN steering scale, symmetric rounding, invalid input and error centering");
}
