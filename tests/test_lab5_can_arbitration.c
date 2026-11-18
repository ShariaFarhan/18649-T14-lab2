/* Exercise real wire IDs with simultaneous arbitration and retained TX slots.
 * This is an ACK/queue model, not a hardware timing or error-confinement test. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "node.h"
#include "tx_queue.h"
#include "lab3_can_rx.h"

struct store {struct l5_meta meta;struct l5_entry entries[2048];uint32_t count;};
struct pending {bool busy;struct l5_tx_item frame;};
static struct store stores[4];
static struct l5_node nodes[4];
static struct lab3_can_rx rx[4];
static struct l5_tx_queue queues[4][2];
static struct pending hardware[4][2][3];
static bool online[4],connected[4][2];
static uint64_t now,boot_at[4],bus_free[2],next_hb[4],next_ctrl;
static uint64_t completed_at[4][2],restarted_at[4][2];
static uint8_t hb_sequence[4],ctrl_sequence;
static unsigned controls[4],failed_frames[2];

static uint64_t local(unsigned id){return now-boot_at[id];}
static int load(void *v,struct l5_meta *m,uint32_t *count)
{struct store *s=v;*m=s->meta;*count=s->count;return 0;}
static int readlog(void *v,uint32_t index,struct l5_entry *entry)
{struct store *s=v;if(!index||index>s->count)return -1;*entry=s->entries[index-1];return 0;}
static int append(void *v,const struct l5_entry *entry)
{struct store *s=v;assert(entry->index==s->count+1&&s->count<2048);s->entries[s->count++]=*entry;return 0;}
static int truncate_log(void *v,uint32_t count)
{struct store *s=v;assert(count>=s->meta.commit);s->count=count;return 0;}
static int save(void *v,const struct l5_meta *m)
{((struct store *)v)->meta=*m;return 0;}
static void sendframe(void *v,unsigned mask,uint16_t id,const uint8_t *data,bool relay)
{
    unsigned self=(unsigned)(uintptr_t)v;
    for(unsigned b=0;b<2;b++)if(mask&(1u<<b))
        l5_tx_add(&queues[self][b],id,data,now,relay);
}
static void boot(unsigned id)
{
    struct l5_storage s={&stores[id],load,readlog,append,truncate_log,save};
    online[id]=true;boot_at[id]=now;
    memset(queues[id],0,sizeof(queues[id]));
    memset(hardware[id],0,sizeof(hardware[id]));
    memset(&rx[id],0,sizeof(rx[id]));
    for(unsigned b=0;b<2;b++)completed_at[id][b]=restarted_at[id][b]=now;
    assert(!l5_node_init(&nodes[id],id,&s,(void *)(uintptr_t)id,sendframe,NULL,NULL,local(id)));
}
static void submit(unsigned id)
{
    if(now>=next_hb[id]){
        uint8_t data[8]={hb_sequence[id]++&15};
        data[HB_STATE]=nodes[id].raft.network.cockpit_ok?ZONE_STATE_NORMAL:ZONE_STATE_SYS_ERROR;
        sendframe((void *)(uintptr_t)id,3,CAN_ID_HB_N1+id-1,data,false);
        next_hb[id]=now+20000;
    }
    if(id==1&&now>=next_ctrl){
        uint8_t data[8]={can_byte0(ctrl_sequence++,0)};
        data[CTRL_THROTTLE]=25;data[CTRL_CRC]=can_crc8(data,CTRL_CRC);
        struct l5_snapshot s;l5_node_snapshot(&nodes[1],&s);
        sendframe((void *)(uintptr_t)1,l5_snapshot_route(&s,0),CAN_ID_CTRL,data,false);
        next_ctrl=now+20000;
    }
}
static void fill_hardware(unsigned id,unsigned bus)
{
    /* Match the STM32 restart backoff. The Pi's single retained transfer is
     * not aborted here just because it has not received an ACK. */
    if(id>1&&now-completed_at[id][bus]>=60000&&
       now-restarted_at[id][bus]>=60000+id*2000+bus*3000){
        memset(hardware[id][bus],0,sizeof(hardware[id][bus]));
        restarted_at[id][bus]=now;
    }
    unsigned capacity=id==1?1:3;
    for(unsigned i=0;i<capacity;i++)if(!hardware[id][bus][i].busy){
        struct l5_tx_item frame;unsigned slot;
        if(!l5_tx_pick(&queues[id][bus],now,&frame,&slot))break;
        l5_tx_remove(&queues[id][bus],slot);
        if(id>1&&frame.id!=CAN_ID_HB_N1+id-1&&!l5_wire_id(frame.id)){
            struct l5_snapshot s;l5_node_snapshot(&nodes[id],&s);
            if(!(l5_snapshot_route(&s,0)&(1u<<bus)))break;
        }
        l5_tx_reference_delay(&frame,now);
        hardware[id][bus][i]=(struct pending){true,frame};
        break;
    }
}
static void arbitrate(unsigned bus)
{
    int selected[4]={-1,-1,-1,-1};uint16_t id=0xffff;
    for(unsigned p=1;p<=3;p++)if(online[p]&&connected[p][bus]){
        /* Equal MCP2515 buffer priorities select the highest slot number. */
        unsigned capacity=p==1?1:3;
        for(unsigned i=0;i<capacity;i++)if(hardware[p][bus][i].busy)selected[p]=(int)i;
        if(selected[p]>=0&&hardware[p][bus][selected[p]].frame.id<id)
            id=hardware[p][bus][selected[p]].frame.id;
    }
    if(id==0xffff)return;
    unsigned transmitters=0,receivers=0;bool same=true;
    struct l5_tx_item *first=NULL;
    for(unsigned p=1;p<=3;p++)if(selected[p]>=0&&hardware[p][bus][selected[p]].frame.id==id){
        struct l5_tx_item *frame=&hardware[p][bus][selected[p]].frame;
        if(first&&memcmp(first->data,frame->data,8))same=false;
        first=frame;transmitters|=1u<<p;
    }
    for(unsigned p=1;p<=3;p++)
        if(online[p]&&connected[p][bus]&&!(transmitters&(1u<<p)))receivers++;
    bus_free[bus]=now+300;
    /* Equal-ID transmitters do not ACK one another. Failed transfers remain
     * pending in hardware instead of being silently removed by the model. */
    if(!same||!receivers){failed_frames[bus]++;return;}
    struct l5_tx_item frame=*first;
    for(unsigned p=1;p<=3;p++)if(transmitters&(1u<<p)){
        hardware[p][bus][selected[p]].busy=false;completed_at[p][bus]=now;
    }
    for(unsigned p=1;p<=3;p++)if(online[p]&&connected[p][bus]&&!(transmitters&(1u<<p))){
        if(l5_wire_id(id))l5_node_receive(&nodes[p],bus,id,frame.data,8,local(p),local(p));
        else{
            enum lab3_rx_result r=lab3_receive(&rx[p],id,bus,frame.data,8,now/1000);
            if(r==LAB3_RX_NEW&&id==CAN_ID_CTRL)controls[p]++;
            if(r!=LAB3_RX_INVALID&&r!=LAB3_RX_CONFLICT)
                l5_node_legacy(&nodes[p],bus,id,frame.data,r==LAB3_RX_NEW,local(p));
        }
    }
}
static void advance(unsigned milliseconds)
{
    for(unsigned tick=0;tick<milliseconds*10;tick++){
        now+=100;
        for(unsigned p=1;p<=3;p++)if(online[p]){
            l5_node_tick(&nodes[p],local(p),false,p==1);
            submit(p);
            for(unsigned b=0;b<2;b++)fill_hardware(p,b);
        }
        for(unsigned b=0;b<2;b++)if(now>=bus_free[b])arbitrate(b);
    }
}
static void start(unsigned leader)
{
    memset(stores,0,sizeof(stores));memset(nodes,0,sizeof(nodes));
    memset(rx,0,sizeof(rx));memset(queues,0,sizeof(queues));memset(hardware,0,sizeof(hardware));
    memset(online,0,sizeof(online));memset(boot_at,0,sizeof(boot_at));
    memset(bus_free,0,sizeof(bus_free));memset(next_hb,0,sizeof(next_hb));
    memset(completed_at,0,sizeof(completed_at));memset(restarted_at,0,sizeof(restarted_at));
    memset(hb_sequence,0,sizeof(hb_sequence));memset(controls,0,sizeof(controls));
    memset(failed_frames,0,sizeof(failed_frames));now=next_ctrl=ctrl_sequence=0;
    for(unsigned p=1;p<=3;p++)connected[p][0]=connected[p][1]=true;
    boot(1);boot(2);boot(3);advance(3000);
    if(leader!=1){
        online[1]=false;
        unsigned other=leader==2?3:2;
        nodes[leader].raft.election_due=local(leader)/1000+61;
        nodes[other].raft.election_due=local(other)/1000+99;
        advance(800);assert(nodes[leader].raft.role==L5_LEADER);
        boot(1);advance(1500);
    }
    assert(nodes[leader].raft.role==L5_LEADER);
}
static void check(unsigned leader,uint32_t term)
{
    for(unsigned p=1;p<=3;p++){
        struct l5_snapshot s;l5_node_snapshot(&nodes[p],&s);
        unsigned expected=(!connected[p][0]?1u:0u)|(!connected[p][1]?2u:0u);
        assert(s.bus_degraded==expected);
        assert(s.leader==leader&&s.term==term&&s.cockpit_alive&&s.cockpit_ok);
        assert(s.network.nodes==14);
        for(unsigned q=1;q<=3;q++){
            unsigned links=(connected[q][0]?1u:0u)|(connected[q][1]?2u:0u);
            assert(s.network.links[q]==links);
        }
    }
    unsigned before2=controls[2],before3=controls[3];
    advance(1000);
    assert(controls[2]>=before2+20&&controls[3]>=before3+20);
}
int main(void)
{
    for(unsigned leader=1;leader<=3;leader++){
        /* Single-link losses must not poison the other two nodes' bus. */
        for(unsigned node=1;node<=3;node++)for(unsigned bus=0;bus<2;bus++){
            start(leader);uint32_t term=nodes[leader].raft.meta.term;
            connected[node][bus]=false;advance(2000);check(leader,term);
            connected[node][bus]=true;advance(1500);check(leader,term);
        }
        /* Every choice of sole bridge, both bus orientations and both orders.
         * Includes Front B first, then Pi A: the reported four-hazard failure. */
        for(unsigned first=1;first<=3;first++)for(unsigned second=first+1;second<=3;second++)
            for(unsigned bus=0;bus<2;bus++)for(unsigned order=0;order<2;order++){
                start(leader);uint32_t term=nodes[leader].raft.meta.term;
                unsigned a=order?second:first,b=order?first:second;
                unsigned a_bus=order?(bus^1):bus,b_bus=a_bus^1;
                connected[a][a_bus]=false;advance(1000);
                connected[b][b_bus]=false;advance(2000);check(leader,term);
                connected[a][a_bus]=true;connected[b][b_bus]=true;
                advance(1500);check(leader,term);
            }
    }
    puts("PASS CAN arbitration: 18 single-link and 36 ordered hop faults, restoration, stable leaders, and CTRL delivery");
    return 0;
}
