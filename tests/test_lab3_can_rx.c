#include <assert.h>
#include <stdio.h>
#include "lab3_can_rx.h"
int main(void)
{
    struct lab3_can_rx r = {0};
    uint8_t hb[8] = {0};
    hb[0] = can_byte0(15,0);
    assert(lab3_receive(&r,CAN_ID_HB_N1,0,hb,8,0)==LAB3_RX_NEW);
    assert(lab3_receive(&r,CAN_ID_HB_N1,1,hb,8,1)==LAB3_RX_DUPLICATE);
    assert(r.hb_applied[0]==1 && r.hb_bus_count[0][1]==1);
    assert(lab3_receive(&r,CAN_ID_HB_N1,0,hb,8,50)==LAB3_RX_REPEAT);
    assert(r.hb_ms[0][0]==0); /* Replay does not keep a peer alive. */
    assert(!lab3_hb_fresh(&r,0,0,60));
    hb[0]=can_byte0(0,0);
    assert(lab3_receive(&r,CAN_ID_HB_N1,1,hb,8,20)==LAB3_RX_NEW);
    assert(lab3_receive(&r,CAN_ID_HB_N1,0,hb,8,21)==LAB3_RX_DUPLICATE);
    assert(r.hb_applied[0]==2);
    hb[1]=ZONE_STATE_SYS_ERROR;
    assert(lab3_receive(&r,CAN_ID_HB_N1,1,hb,8,22)==LAB3_RX_CONFLICT);
    hb[1]=0;hb[0]=15;
    assert(lab3_receive(&r,CAN_ID_HB_N1,1,hb,8,23)==LAB3_RX_OLD);
    assert(lab3_receive(&r,CAN_ID_HB_N2,0,hb,8,23)==LAB3_RX_NEW);
    assert(lab3_receive(&r,CAN_ID_HB_N1,0,hb,7,24)==LAB3_RX_INVALID);
    hb[1]=99;
    assert(lab3_receive(&r,CAN_ID_HB_N1,0,hb,8,24)==LAB3_RX_INVALID);
    hb[1]=0;hb[0]=8;
    assert(lab3_receive(&r,CAN_ID_HB_N1,0,hb,8,121)==LAB3_RX_OLD);
    hb[0]=9;
    assert(lab3_receive(&r,CAN_ID_HB_N1,0,hb,8,141)==LAB3_RX_NEW); /* advancing reboot resync */
    assert(lab3_receive(&r,0x555,0,hb,8,121)==LAB3_RX_INVALID);
    uint8_t fault[8]={0,1,FAULT_R_SELF_TEST};
    assert(lab3_receive(&r,CAN_ID_FAULT,0,fault,8,130)==LAB3_RX_NEW);
    fault[1]=2;
    assert(lab3_receive(&r,CAN_ID_FAULT,0,fault,8,130)==LAB3_RX_NEW);
    assert(lab3_receive(&r,CAN_ID_FAULT,1,fault,8,131)==LAB3_RX_DUPLICATE);
    puts("PASS: first-copy delivery, both orders, 15->0, per-bus freshness, repeat/stale/conflict rejection, reboot and FAULT producers");
}
