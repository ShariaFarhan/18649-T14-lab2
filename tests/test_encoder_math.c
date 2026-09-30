#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "logic/drive/encoder_math.h"

int main(void)
{
    int32_t d, rpm;
    assert(encoder_count_delta(110,100,63360,&d) && d==10);
    assert(encoder_count_delta(100,110,63360,&d) && d==-10);
    assert(encoder_count_delta(4,63358,63360,&d) && d==6);
    assert(encoder_count_delta(63358,4,63360,&d) && d==-6);
    assert(encoder_count_delta(2,UINT32_MAX-2,1ULL<<32,&d) && d==5);
    assert(encoder_count_delta(UINT32_MAX-2,2,1ULL<<32,&d) && d==-5);
    assert(!encoder_count_delta(31680,0,63360,&d));
    assert(!encoder_count_delta(63360,0,63360,&d));
    assert(!encoder_count_delta(0,0,0,&d));
    assert(!encoder_count_delta(0,0,63360,NULL));
    assert(encoder_rpm_milli(3960,3960,1000000,&rpm) && rpm==60000);
    assert(encoder_rpm_milli(-3960,3960,1000000,&rpm) && rpm==-60000);
    assert(encoder_rpm_milli(66,3960,10000,&rpm) && rpm==100000);
    assert(encoder_rpm_milli(66,3960,20000,&rpm) && rpm==50000);
    assert(encoder_rpm_milli(1,3960,10000,&rpm) && rpm==1515);
    assert(encoder_rpm_milli(0,3960,10000,&rpm) && rpm==0);
    assert(!encoder_rpm_milli(1,3960,0,&rpm));
    assert(!encoder_rpm_milli(1,0,10000,&rpm));
    assert(!encoder_rpm_milli(INT32_MAX,3960,1,&rpm));
    assert(!encoder_rpm_milli(1,UINT32_MAX,UINT64_MAX,&rpm));
    /* Move one wheel turn across counter wrap, then one turn backwards. */
    uint32_t raw=63000;int64_t total=0;
    for (unsigned i=0;i<396;i++) {
        uint32_t next=(raw+10)%63360;
        assert(encoder_count_delta(next,raw,63360,&d));total+=d;raw=next;
    }
    assert(total==3960);
    for(unsigned i=0;i<396;i++) {
        uint32_t next=(raw+63360-10)%63360;
        assert(encoder_count_delta(next,raw,63360,&d));total+=d;raw=next;
    }
    assert(total==0);
    /* Many positions/directions around the non-power-of-two timer modulus. */
    for(uint32_t start=0;start<63360;start+=97)
        for(int step=-200;step<=200;step++) {
            uint32_t next=(uint32_t)(((int64_t)start+step+63360)%63360);
            assert(encoder_count_delta(next,start,63360,&d) && d==step);
        }
    puts("PASS: x4 CPR math, forward/reverse, timer wrap, measured dt, invalid/overflow checks");
    return 0;
}
