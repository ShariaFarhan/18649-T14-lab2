#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "node.h"
#include "tx_queue.h"
#include "lab3_can_rx.h"
struct store {struct l5_meta meta;struct l5_entry entries[2048];uint32_t count;};
static struct store stores[4];static struct l5_node nodes[4];
static struct lab3_can_rx rx[4];static struct l5_tx_queue queues[4][2];
static bool online[4],paused[4],link_up[4][2];static uint64_t now,bus_free[2],next_hb[4];
static uint8_t hb_seq[4];static unsigned replayed[4],events[4];
static uint64_t boot_time[4];
static uint64_t local(unsigned id){return now-boot_time[id];}
static int load(void*v,struct l5_meta*m,uint32_t*n){struct store*s=v;*m=s->meta;*n=s->count;return 0;}
static int readlog(void*v,uint32_t i,struct l5_entry*e){struct store*s=v;if(!i||i>s->count)return -1;*e=s->entries[i-1];return 0;}
static int append(void*v,const struct l5_entry*e){struct store*s=v;assert(e->index==s->count+1&&s->count<2048);s->entries[s->count++]=*e;return 0;}
static int truncatelog(void*v,uint32_t n){struct store*s=v;assert(n>=s->meta.commit);s->count=n;return 0;}
static int save(void*v,const struct l5_meta*m){((struct store*)v)->meta=*m;return 0;}
static void sendframe(void*v,unsigned mask,uint16_t id,const uint8_t*d,bool relay)
{unsigned self=(unsigned)(uintptr_t)v;for(unsigned b=0;b<2;b++)if(mask&(1u<<b))l5_tx_add(&queues[self][b],id,d,now,relay);}
static void entry(void*v,const struct l5_entry*e,bool replay)
{unsigned self=(unsigned)(uintptr_t)v;if(l5_kind(e->event)==L5_EVENT){events[self]++;if(replay)replayed[self]++;}}
static void boot(unsigned id)
{
    struct l5_storage s={&stores[id],load,readlog,append,truncatelog,save};online[id]=true;
    memset(queues[id],0,sizeof(queues[id]));memset(&rx[id],0,sizeof(rx[id]));
    boot_time[id]=now;
    assert(!l5_node_init(&nodes[id],id,&s,(void*)(uintptr_t)id,sendframe,entry,NULL,local(id)));
}
static void step(void)
{
    now+=100;
    for(unsigned p=1;p<=3;p++)if(online[p]){
        l5_node_tick(&nodes[p],local(p),paused[p],p==1);
        if(!paused[p]&&now>=next_hb[p]){
            uint8_t d[8]={hb_seq[p]++&15};d[HB_STATE]=nodes[p].raft.network.cockpit_ok?0:1;
            sendframe((void*)(uintptr_t)p,3,CAN_ID_HB_N1+p-1,d,false);next_hb[p]=now+20000;
        }
    }
    for(unsigned b=0;b<2;b++)if(now>=bus_free[b]){
        struct l5_tx_item best={.id=0xffff};unsigned who=0,slot=0;
        for(unsigned p=1;p<=3;p++)if(online[p]&&link_up[p][b]){
            struct l5_tx_item candidate;unsigned index;
            if(l5_tx_pick(&queues[p][b],now,&candidate,&index)&&candidate.id<best.id){best=candidate;who=p;slot=index;}
        }
        if(!who)continue;
        unsigned listeners=0;for(unsigned p=1;p<=3;p++)if(p!=who&&online[p]&&link_up[p][b])listeners++;
        if(!listeners)continue;
        l5_tx_reference_delay(&best,now);l5_tx_remove(&queues[who][b],slot);bus_free[b]=now+300;
        for(unsigned p=1;p<=3;p++)if(p!=who&&online[p]&&link_up[p][b]){
            if(l5_wire_id(best.id))l5_node_receive(&nodes[p],b,best.id,best.data,8,local(p),local(p));
            else{
                enum lab3_rx_result r=lab3_receive(&rx[p],best.id,b,best.data,8,now/1000);
                if(r!=LAB3_RX_INVALID&&r!=LAB3_RX_CONFLICT)l5_node_legacy(&nodes[p],b,best.id,best.data,r==LAB3_RX_NEW,local(p));
            }
        }
    }
}
static void advance(unsigned milliseconds){for(unsigned i=0;i<milliseconds*10;i++)step();}
static void equal(unsigned a,unsigned b)
{
    assert(nodes[a].raft.meta.commit==nodes[b].raft.meta.commit);
    for(unsigned i=0;i<nodes[a].raft.meta.commit;i++)assert(!memcmp(&stores[a].entries[i],&stores[b].entries[i],sizeof(struct l5_entry)));
}
static void reset(void)
{
    memset(stores,0,sizeof(stores));memset(nodes,0,sizeof(nodes));memset(online,0,sizeof(online));
    memset(paused,0,sizeof(paused));memset(bus_free,0,sizeof(bus_free));memset(next_hb,0,sizeof(next_hb));
    memset(events,0,sizeof(events));memset(replayed,0,sizeof(replayed));now=0;
    for(unsigned p=1;p<=3;p++)link_up[p][0]=link_up[p][1]=true;
}
static void clock_recovery(void)
{
    reset();boot(1);boot(2);boot(3);advance(3000);
    uint64_t before=l5_global_gettime(&nodes[3].clock,local(3));
    online[1]=false;boot(2); /* Actual reboot: local counter resets to zero. */
    nodes[2].raft.election_due=61;nodes[3].raft.election_due=local(3)/1000+99;
    advance(900);
    assert(nodes[2].raft.role==L5_LEADER&&nodes[2].time_ready);
    assert(nodes[3].clock.synced);
    uint64_t a=l5_global_gettime(&nodes[2].clock,local(2)),b=l5_global_gettime(&nodes[3].clock,local(3));
    assert(a>before&&b>before&&(a>b?a-b:b-a)<2000);
    uint32_t term=nodes[2].raft.meta.term;advance(1500);assert(nodes[2].raft.meta.term==term);
    printf("PASS cold elected leader inherits live holdover epoch, no boot-zero publication\n");
    for(int sign=-1;sign<=1;sign+=2){
    reset();boot(1);boot(2);boot(3);advance(3000);
    link_up[3][0]=link_up[3][1]=false;advance(1000);
    nodes[3].clock.global_anchor+=sign*65000;nodes[3].clock.last_return+=sign*65000;
    link_up[3][0]=link_up[3][1]=true;
    uint64_t previous=l5_global_gettime(&nodes[3].clock,local(3));
    for(unsigned i=0;i<50000;i++){
        step();uint64_t current=l5_global_gettime(&nodes[3].clock,local(3));
        assert(current>=previous&&current-previous<=110);previous=current;
    }
    a=l5_global_gettime(&nodes[1].clock,local(1));b=l5_global_gettime(&nodes[3].clock,local(3));
    assert(nodes[3].clock.synced&&(a>b?a-b:b-a)<2000);
    assert(nodes[3].raft.network.nodes==14);equal(1,3);
    }
    printf("PASS +/-65ms isolated-clock recovery: monotonic slew, fresh liveness, graph/log recovery\n");
}
static void epoch_safety(void)
{
    reset();boot(1);boot(2);boot(3);advance(3000);
    uint32_t old_nonce=nodes[3].probes[1].nonce;assert(old_nonce);
    boot(3);l5_node_tick(&nodes[3],local(3),false,false);
    assert(nodes[3].probes[1].nonce>old_nonce);
    struct l5_message replay={.type=L5_TIME_REPLY,.source=1,.destination=3,
        .term=nodes[3].raft.meta.term,.index=old_nonce,.ok=5,.event=3000000};
    nodes[3].wire.sample(&nodes[3],1,old_nonce,3000000,local(3),350);
    nodes[3].wire.message(&nodes[3],&replay,local(3));
    assert(!nodes[3].clock.established&&!nodes[3].peer_known[1]);
    online[1]=false;boot(2);boot(3);advance(1500);
    unsigned leader=nodes[2].raft.role==L5_LEADER?2:3;
    assert(nodes[leader].raft.role==L5_LEADER&&nodes[leader].raft.time_pending);
    assert(!nodes[2].clock.established&&!nodes[3].clock.established);
    boot(1);advance(1500);
    assert(nodes[leader].time_ready&&nodes[1].clock.synced);equal(1,leader);
    printf("PASS stale boot nonce rejected; two historical cold nodes wait for epoch evidence; all three can recover\n");
    reset();boot(1);boot(2);boot(3);advance(3000);
    link_up[2][0]=link_up[2][1]=false;advance(1000);
    nodes[2].clock.global_anchor+=65000;nodes[2].clock.last_return+=65000;
    link_up[2][0]=link_up[2][1]=true;advance(200);assert(nodes[2].clock.recovering);
    online[1]=false;boot(3);nodes[3].raft.election_due=61;nodes[2].raft.election_due=local(2)/1000+99;
    advance(6000);assert(nodes[3].raft.role==L5_LEADER&&nodes[3].time_ready&&nodes[2].clock.synced);
    assert(l5_global_gettime(&nodes[3].clock,local(3))>10000000);
    printf("PASS recovering holdover completes finite slew after master loss and seeds a cold leader\n");
}
static void boot_and_bridges(void)
{
    reset();boot(2);boot(3);advance(900);
    unsigned leader=nodes[2].raft.role==L5_LEADER?2:3;
    assert(nodes[leader].raft.role==L5_LEADER&&nodes[leader].time_ready);
    boot(1);advance(1500);assert(nodes[1].raft.role==L5_FOLLOWER);equal(1,leader);
    printf("PASS default leader absent at first boot: bounded grace, election, late Pi follows\n");
    for(unsigned chosen=2;chosen<=3;chosen++){
        reset();boot(1);boot(2);boot(3);advance(3000);online[1]=false;
        nodes[chosen].raft.election_due=local(chosen)/1000+61;
        unsigned other=chosen==2?3:2;nodes[other].raft.election_due=local(other)/1000+99;
        advance(800);assert(nodes[chosen].raft.role==L5_LEADER);boot(1);advance(1200);
        link_up[1][1]=false;link_up[other][0]=false;
        uint32_t before=nodes[chosen].raft.meta.commit,term=nodes[chosen].raft.meta.term;
        advance(5000);
        for(unsigned p=1;p<=3;p++){
            assert(nodes[p].raft.network.nodes==14);
            assert(nodes[p].raft.network.links[chosen]==3&&nodes[p].raft.network.links[1]==1&&nodes[p].raft.network.links[other]==2);
        }
        assert(nodes[chosen].raft.meta.term==term&&nodes[chosen].raft.meta.commit-before<20);
        equal(1,2);equal(1,3);
        for(unsigned p=1;p<=3;p++)link_up[p][0]=link_up[p][1]=true;
        advance(500);online[chosen]=false;
        for(unsigned i=0;i<15000;i++){
            step();struct l5_snapshot s;l5_node_snapshot(&nodes[other],&s);
            assert(s.cockpit_alive&&s.cockpit_ok);
        }
    }
    printf("PASS both STM leaders as sole bridge: no false peer-down or log churn\n");
    printf("PASS either STM leader power-off: surviving live cockpit never enters error\n");
}
int main(void)
{
    for(unsigned p=1;p<=3;p++)link_up[p][0]=link_up[p][1]=true;
    boot(1);boot(2);boot(3);advance(3000);
    printf("boot term=%u/%u/%u commit=%u/%u/%u role=%u/%u/%u\n",nodes[1].raft.meta.term,nodes[2].raft.meta.term,nodes[3].raft.meta.term,nodes[1].raft.meta.commit,nodes[2].raft.meta.commit,nodes[3].raft.meta.commit,nodes[1].raft.role,nodes[2].raft.role,nodes[3].raft.role);
    equal(1,2);equal(1,3);assert(nodes[1].raft.meta.term==1&&nodes[1].raft.network.cockpit_ok);
    uint32_t bridge_before=nodes[1].raft.meta.commit;
    link_up[2][1]=false;link_up[3][0]=false;advance(5000);
    for(unsigned p=1;p<=3;p++){
        assert(nodes[p].raft.network.nodes==14);
        assert(nodes[p].raft.network.links[1]==3&&nodes[p].raft.network.links[2]==1&&nodes[p].raft.network.links[3]==2);
    }
    assert(nodes[1].raft.meta.commit-bridge_before<20);equal(1,2);equal(1,3);
    for(unsigned p=1;p<=3;p++)link_up[p][0]=link_up[p][1]=true;
    advance(500);
    paused[2]=true;memset(queues[2],0,sizeof(queues[2]));
    uint8_t fault[8]={0,2,FAULT_R_SELF_TEST};sendframe((void*)(uintptr_t)2,3,CAN_ID_FAULT,fault,false);
    advance(500);struct l5_snapshot snapshot;l5_node_snapshot(&nodes[1],&snapshot);assert(!snapshot.cockpit_ok);
    uint8_t old_normal[8]={5};l5_node_legacy(&nodes[1],0,CAN_ID_HB_N2,old_normal,true,now);
    l5_node_snapshot(&nodes[1],&snapshot);assert(!snapshot.cockpit_ok); /* Stale HB cannot clear self-test. */
    paused[2]=false;advance(1000);l5_node_snapshot(&nodes[1],&snapshot);assert(snapshot.cockpit_ok);
    equal(1,2);equal(1,3);
    link_up[1][1]=false;advance(300);link_up[3][0]=false;advance(2000);
    equal(1,2);equal(1,3);assert(nodes[1].raft.meta.term==1);
    assert(nodes[1].raft.network.links[1]==1&&nodes[1].raft.network.links[3]==2);
    assert(nodes[3].clock.last_ref+60000>now);
    unsigned before=events[1];online[1]=false;advance(1000);
    unsigned leader=nodes[2].raft.role==L5_LEADER?2:3;assert(nodes[leader].raft.role==L5_LEADER);
    advance(2000);boot(1);advance(2000);equal(1,leader);
    assert(events[1]>before&&replayed[1]>0&&nodes[1].raft.role==L5_FOLLOWER);
    assert(nodes[leader].raft.role==L5_LEADER);
    for(unsigned p=1;p<=3;p++)link_up[p][0]=link_up[p][1]=true;
    advance(500);uint32_t term=nodes[leader].raft.meta.term;
    unsigned other=leader==2?3:2;
    link_up[leader][1]=false;link_up[other][0]=false;advance(1500);
    assert(nodes[leader].raft.role==L5_LEADER&&nodes[leader].raft.meta.term==term);
    assert(nodes[other].clock.last_ref+60000>now);equal(1,2);equal(1,3);
    for(unsigned p=1;p<=3;p++){link_up[p][0]=false;link_up[p][1]=true;}
    advance(1500);assert(nodes[leader].raft.meta.term==term);equal(1,2);equal(1,3);
    for(unsigned p=1;p<=3;p++)assert(nodes[p].raft.network.links[p]==2);
    printf("PASS CAN simulation: fragmented RPCs, REF forwarding, link-state commit, no link-fault election, leader failover and replay (%u)\n",replayed[1]);
    clock_recovery();boot_and_bridges();epoch_safety();
}
