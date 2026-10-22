#include <assert.h>
#include <stdio.h>
#include "lab3_can_rx.h"
int main(void)
{
    struct lab3_can_rx r={0};uint8_t d[8]={0};
    lab3_receive(&r,CAN_ID_HB_N1,0,d,8,0);
    lab3_receive(&r,CAN_ID_HB_N1,1,d,8,1);
    for(unsigned n=1;n<40;++n) {
        d[0]=n&15;
        assert(lab3_receive(&r,CAN_ID_HB_N1,0,d,8,n*20)==LAB3_RX_NEW);
        d[0]=(n-1)&15;
        (void)lab3_receive(&r,CAN_ID_HB_N1,1,d,8,n*20+1);
        assert(lab3_hb_fresh(&r,0,1,n*20+1));
        assert(r.hb_applied[0]==n+1); /* No double application of delayed copies. */
    }
    struct lab3_can_rx replay={0};d[0]=0;
    assert(lab3_receive(&replay,CAN_ID_HB_N1,0,d,8,0)==LAB3_RX_NEW);
    for(unsigned t=20;t<1000;t+=20)
        assert(lab3_receive(&replay,CAN_ID_HB_N1,0,d,8,t)==LAB3_RX_REPEAT);
    assert(replay.hb_applied[0]==1 && !lab3_hb_fresh(&replay,0,0,1000));
    puts("PASS: one-period-late bus stays fresh across wrap; fixed replay never refreshes or reapplies");
}
