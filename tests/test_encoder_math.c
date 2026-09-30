#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "logic/drive/encoder_math.h"

int main(void)
{
    int32_t d, rpm;
    assert(encoder_count_delta(110,100,64680,&d) && d==10);
    assert(encoder_count_delta(100,110,64680,&d) && d==-10);
    assert(encoder_count_delta(4,64678,64680,&d) && d==6);
    assert(encoder_count_delta(64678,4,64680,&d) && d==-6);
    assert(encoder_count_delta(2,UINT32_MAX-2,1ULL<<32,&d) && d==5);
    assert(encoder_count_delta(UINT32_MAX-2,2,1ULL<<32,&d) && d==-5);
    assert(!encoder_count_delta(32340,0,64680,&d));
    assert(!encoder_count_delta(64680,0,64680,&d));
    assert(!encoder_count_delta(0,0,0,&d));
    assert(!encoder_count_delta(0,0,64680,NULL));
    assert(encoder_rpm_milli(1320,1320,1000000,&rpm) && rpm==60000);
    assert(encoder_rpm_milli(-1320,1320,1000000,&rpm) && rpm==-60000);
    assert(encoder_rpm_milli(22,1320,10000,&rpm) && rpm==100000);
    assert(encoder_rpm_milli(22,1320,20000,&rpm) && rpm==50000);
    assert(encoder_rpm_milli(1,1320,10000,&rpm) && rpm==4545);
    assert(encoder_rpm_milli(0,1320,10000,&rpm) && rpm==0);
    assert(!encoder_rpm_milli(1,1320,0,&rpm));
    assert(!encoder_rpm_milli(1,0,10000,&rpm));
    assert(!encoder_rpm_milli(INT32_MAX,1320,1,&rpm));
    assert(!encoder_rpm_milli(1,UINT32_MAX,UINT64_MAX,&rpm));
    /* Move one wheel turn across counter wrap, then one turn backwards. */
    uint32_t raw=64300;int64_t total=0;
    for (unsigned i=0;i<132;i++) {
        uint32_t next=(raw+10)%64680;
        assert(encoder_count_delta(next,raw,64680,&d));total+=d;raw=next;
    }
    assert(total==1320);
    for(unsigned i=0;i<132;i++) {
        uint32_t next=(raw+64680-10)%64680;
        assert(encoder_count_delta(next,raw,64680,&d));total+=d;raw=next;
    }
    assert(total==0);
    /* Many positions/directions around the non-power-of-two timer modulus. */
    for(uint32_t start=0;start<64680;start+=97)
        for(int step=-200;step<=200;step++) {
            uint32_t next=(uint32_t)(((int64_t)start+step+64680)%64680);
            assert(encoder_count_delta(next,start,64680,&d) && d==step);
        }
    puts("PASS: x4 CPR math, forward/reverse, timer wrap, measured dt, invalid/overflow checks");
    return 0;
}
