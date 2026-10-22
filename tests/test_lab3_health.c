#include <assert.h>
#include <stdio.h>
#include "lab3_can_rx.h"
static void peer(struct lab3_can_rx *r, unsigned p, unsigned bus, uint8_t seq, uint64_t now)
{
    uint8_t hb[8]={seq};
    (void)lab3_receive(r,CAN_ID_HB_N1+p,bus,hb,8,now);
}
int main(void)
{
    struct lab3_can_rx r={0};
    assert(lab3_health_get(&r,0,1000).degraded==0); /* No spurious startup lamps. */
    peer(&r,1,0,0,0);peer(&r,1,1,0,1);
    peer(&r,2,0,0,0);peer(&r,2,1,0,1);
    assert(lab3_health_get(&r,0,20).degraded==0);
    peer(&r,1,1,1,40);peer(&r,2,1,1,40);
    assert(lab3_health_get(&r,0,59).degraded==0);
    struct lab3_health h=lab3_health_get(&r,0,60);
    assert(h.degraded==1 && h.missing_peers==0 && !h.isolated);
    peer(&r,1,0,1,61); /* Second copy still restores A's freshness. */
    assert(lab3_health_get(&r,0,61).degraded==0);
    peer(&r,2,0,2,90);peer(&r,2,1,2,90);
    h=lab3_health_get(&r,0,122); /* N2 lost, but N3 proves A and B remain usable. */
    assert(h.degraded==0 && h.missing_peers==(1u<<1) && !h.isolated);
    h=lab3_health_get(&r,0,150);
    assert(h.degraded==3 && h.missing_peers==6 && h.isolated);
    peer(&r,2,0,3,151);
    assert(lab3_health_get(&r,0,151).degraded==2);
    puts("PASS: startup, 60ms threshold, cross-bus copy recovery, peer loss vs bus loss, isolation");
}
