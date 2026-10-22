#include <assert.h>
#include <stdio.h>
#include "lab3_relay.h"
static void observe(struct lab3_relay_plan *p,struct lab3_can_rx *r,
                    uint16_t id,uint8_t *d,unsigned bus,uint64_t now)
{
    enum lab3_rx_result result=lab3_receive(r,id,bus,d,8,now);
    lab3_relay_observe(p,id,d,bus,result,now);
}
static void forward(struct lab3_relay_plan *p,struct lab3_can_rx *destination,
                    uint16_t id,uint8_t *d,unsigned bus,uint64_t now)
{
    struct lab3_relay_copy c;
    unsigned slot=(unsigned)lab3_slot(id,d);
    assert(lab3_relay_candidate(p,slot,bus,now,&c));
    assert(c.id==id && !memcmp(c.data,d,8));
    lab3_relay_queued(p,slot,bus,c.revision);
    assert(lab3_receive(destination,id,bus,c.data,8,now)==LAB3_RX_NEW);
    assert(!lab3_relay_candidate(p,slot,bus,now+1,&c));
}
int main(void)
{
    struct lab3_relay_plan p={0};struct lab3_can_rx n1={0},n2={0},n3={0};
    struct lab3_relay_copy c;
    /* N1 only A, N3 only B: 40 periods cross sequence wrap twice. */
    for (unsigned i=0;i<40;i++) {
        uint64_t t=i*20;
        uint8_t h1[8]={i&15},h2[8]={i&15},h3[8]={i&15};
        observe(&p,&n2,CAN_ID_HB_N1,h1,0,t);
        observe(&p,&n2,CAN_ID_HB_N3,h3,1,t);
        forward(&p,&n3,CAN_ID_HB_N1,h1,1,t+1);
        forward(&p,&n1,CAN_ID_HB_N3,h3,0,t+1);
        assert(lab3_receive(&n1,CAN_ID_HB_N2,0,h2,8,t+1)==LAB3_RX_NEW);
        assert(lab3_receive(&n3,CAN_ID_HB_N2,1,h2,8,t+1)==LAB3_RX_NEW);
        struct lab3_health a=lab3_health_get(&n1,0,t+2),b=lab3_health_get(&n2,1,t+2),z=lab3_health_get(&n3,2,t+2);
        assert(!a.missing_peers && a.degraded==2 && !a.isolated);
        assert(!b.missing_peers && b.degraded==0);
        assert(!z.missing_peers && z.degraded==1 && !z.isolated);
        uint8_t ctrl[8]={i&15,0,0,35,0,0,0,0};ctrl[7]=can_crc8(ctrl,7);
        observe(&p,&n2,CAN_ID_CTRL,ctrl,0,t+2);
        forward(&p,&n3,CAN_ID_CTRL,ctrl,1,t+3);
        observe(&p,&n2,CAN_ID_CTRL,ctrl,0,t+4); /* Repeat cannot re-relay. */
        observe(&p,&n2,CAN_ID_CTRL,ctrl,1,t+5); /* Returned/other-bus copy cannot loop. */
        assert(!lab3_relay_pending(&p));
    }
    /* Original copy on destination bus before forwarding cancels the work. */
    uint8_t blink[8]={0,BLINK_MODE_LEFT,1};
    observe(&p,&n2,CAN_ID_BLINK,blink,0,810);
    observe(&p,&n2,CAN_ID_BLINK,blink,1,811);
    assert(!lab3_relay_candidate(&p,7,1,812,&c));assert(p.suppressed==1);
    /* B->A works too; blocked destination expires and never holds the worker. */
    blink[0]=1;observe(&p,&n2,CAN_ID_BLINK,blink,1,830);
    assert(lab3_relay_candidate(&p,7,0,849,&c));
    assert(!lab3_relay_candidate(&p,7,0,850,&c));assert(p.expired==1);
    /* Newest pending value wins; old completion cannot consume new work. */
    blink[0]=2;observe(&p,&n2,CAN_ID_BLINK,blink,1,851);
    assert(lab3_relay_candidate(&p,7,0,852,&c));uint32_t old=c.revision;
    blink[0]=3;blink[2]=0;observe(&p,&n2,CAN_ID_BLINK,blink,1,853);
    lab3_relay_queued(&p,7,0,old);
    forward(&p,&n1,CAN_ID_BLINK,blink,0,854);
    /* Foreign FAULT producers keep separate slots; N2 never relays itself. */
    uint8_t f1[8]={0,ZONE_COCKPIT,FAULT_R_SELF_TEST},f3[8]={0,ZONE_DRIVETRAIN,FAULT_R_OUTPUT};
    observe(&p,&n2,CAN_ID_FAULT,f1,0,860);observe(&p,&n2,CAN_ID_FAULT,f3,1,860);
    forward(&p,&n3,CAN_ID_FAULT,f1,1,861);forward(&p,&n1,CAN_ID_FAULT,f3,0,861);
    uint8_t own[8]={0,ZONE_STEERING,FAULT_R_OUTPUT};
    observe(&p,&n2,CAN_ID_FAULT,own,0,862);
    uint8_t force[8]={0,12};observe(&p,&n2,CAN_ID_FORCE,force,0,863);
    assert(!lab3_relay_pending(&p)); /* FORCE belongs to N2: dual transmit, no echo relay. */
    /* Invalid data is rejected before it can become a forwarded command. */
    uint8_t bad[8]={0,0,0,255};observe(&p,&n2,CAN_ID_CTRL,bad,0,864);
    assert(!lab3_relay_pending(&p));
    /* N1 true loss still expires despite the relay being present. */
    assert(lab3_health_get(&n3,2,900).missing_peers & 1);
    puts("PASS: cross-isolation heartbeats/CTRL, both directions, wrap, dedup/loop prevention, cancel, expiry, supersession and FAULT sources");
}
