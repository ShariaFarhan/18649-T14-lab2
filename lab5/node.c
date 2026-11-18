#include "node.h"
#include "lab3_can_catalog.h"
#include <string.h>
static unsigned origin(uint16_t id,const uint8_t*d)
{
    if(id>=L5_RPC_BASE)return (d[0]>>4)&3;
    if(id>=L5_SHORT_BASE&&id<0x480)return (id>>4)&3;
    if(id>=L5_TIME_REPLY_BASE&&id<=L5_TIME_REPLY_BASE+3)return (d[0]>>4)&3;
    if(id>=L5_QUERY_BASE&&id<L5_PROOF_BASE+64)return (id>>4)&3;
    if(id>=L5_REF_BASE&&id<=0x4f)return (id-L5_REF_BASE)>>2;
    if(id>=L5_ACK_BASE&&id<=0xbf)return ((id-L5_ACK_BASE)>>4)&3;
    if(id>=L5_SAMPLE_BASE&&id<L5_SAMPLE_BASE+64)return ((id-L5_SAMPLE_BASE)>>4)&3;
    if(id>=CAN_ID_HB_N1&&id<=CAN_ID_HB_N3)return id-CAN_ID_HB_N1+1;
    if(id==CAN_ID_CTRL||id==CAN_ID_BLINK)return 1;
    if(id==CAN_ID_FORCE)return 2;
    if(id==CAN_ID_FAULT)return d[FAULT_ZONE];
    return 0;
}
unsigned l5_snapshot_route(const struct l5_snapshot*s,unsigned dest)
{
    unsigned mask=0;
    if(dest)mask=l5_route(&s->network,s->id,dest);
    else for(unsigned p=1;p<=3;p++)if(p!=s->id)mask|=l5_route(&s->network,s->id,p);
    return mask?mask:3; /* Discovery fallback; does not mutate the committed graph. */
}
static void wire_send(void*v,unsigned mask,uint16_t id,const uint8_t*d)
{
    struct l5_node*n=v;if(n->failed||n->raft.fatal)return;
    unsigned source=origin(id,d);
    if(source>0&&source<=3&&source!=n->raft.id&&(mask==1||mask==2)){
        unsigned bus=mask==1?0:1;
        if(n->wire.direct_known[source][bus]&&n->now_us>=n->wire.direct_at[source][bus]&&
           n->now_us-n->wire.direct_at[source][bus]<40000)return;
    }
    n->send(n->context,mask,id,d,origin(id,d)!=n->raft.id);
}
static void send_rpc(void*v,const struct l5_message*m)
{
    struct l5_node*n=v;
    if(n->raft.config.debug&&n->trace)n->trace(n->context,m,true);
    /* Consensus uses both paths for discovery even while the graph is stale.
     * This breaks the dependency between repairing routes and agreeing on routes. */
    l5_wire_send(&n->wire,m,3);
}
static void applied(void*v,const struct l5_entry*e,bool replayed)
{struct l5_node*n=v;if(n->entry)n->entry(n->context,e,replayed);}
static void alive(struct l5_node*n,unsigned source,uint64_t at)
{
    if(!n->peer_known[source]||at>n->peer_seen[source])n->peer_seen[source]=at;
    n->peer_known[source]=true;
    if((n->test_failures&(1u<<source))&&at>=n->test_failed_at[source]&&at-n->test_failed_at[source]>=100000)n->test_failures&=~(1u<<source);
}
enum { EPOCH_VALID=1,EPOCH_PRISTINE=2,EPOCH_HELD=4,CONTACT_ONLY=8,PEER_LEADER=16,PEER_FOLLOWS=32 };
#define PROBE_WINDOW_US 150000u
/* Nonces are durably reserved before use. The low half never wraps within a
 * reservation. Thus packets from a previous boot cannot authorize a clock. */
static void probe(struct l5_node*n,unsigned peer)
{
    if(peer<1||peer>3||peer==n->raft.id||n->failed||n->raft.fatal)return;
    struct l5_time_probe*p=&n->probes[peer];
    if(p->active&&n->now_us-p->sent<PROBE_WINDOW_US)return;
    if(n->now_us<n->probe_due[peer])return;
    if(!n->nonce_boot||n->nonce_round==511){
        if(!l5_reserve_id(&n->raft,&n->nonce_boot)){n->raft.fatal=true;return;}
        n->nonce_round=0;
    }
    *p=(struct l5_time_probe){.nonce=(n->nonce_boot<<9)|++n->nonce_round,
        .term=n->raft.meta.term,.sent=n->now_us,.active=true,.generation=n->authority_generation};
    n->probe_due[peer]=n->now_us+30000;
    send_rpc(n,&(struct l5_message){.type=L5_TIME_QUERY,.source=n->raft.id,.destination=peer,
        .term=p->term,.index=p->nonce,.ok=n->raft.role==L5_LEADER&&n->time_ready?CONTACT_ONLY:0});
}
static void authority_state(struct l5_node*n)
{
    if(n->observed_term==n->raft.meta.term&&n->observed_role==n->raft.role&&n->observed_leader==n->raft.leader)return;
    /* Outstanding nonce challenges remain valid liveness evidence through an
     * election. Only their authority to install time is revoked. */
    n->authority_generation++;memset(n->epochs,0,sizeof(n->epochs));
    memset(n->probe_due,0,sizeof(n->probe_due));n->authority_until=0;n->time_conflict=false;
    n->observed_term=n->raft.meta.term;n->observed_role=n->raft.role;n->observed_leader=n->raft.leader;
    n->activation_at=n->now_us;n->time_ready=false;
    if(n->raft.role==L5_LEADER&&n->clock.established&&n->clock.synced&&!n->clock.recovering&&
       n->now_us-n->clock.last_ref<1000000){
        l5_clock_master(&n->clock,n->now_us,n->raft.id,n->raft.meta.term);n->time_ready=true;
    }
    n->raft.time_pending=n->raft.role==L5_LEADER&&!n->time_ready;
}
static void finish_probe(struct l5_node*n,unsigned source)
{
    struct l5_time_probe*p=&n->probes[source];
    if(!p->active||!p->reply||n->now_us-p->sent>=PROBE_WINDOW_US)return;
    if((p->flags&EPOCH_VALID)&&!p->sample)return;
    uint32_t elapsed=(uint32_t)(p->low-(uint32_t)p->raw);
    if((p->flags&EPOCH_VALID)&&(elapsed>PROBE_WINDOW_US||p->at<p->sent))return;
    p->active=false;alive(n,source,p->sent); /* Causal proof, not receipt of arbitrary traffic. */
    if(p->term!=n->raft.meta.term||p->generation!=n->authority_generation)return;
    if(n->now_us-p->sent<(uint64_t)n->raft.config.heartbeat_ms*n->raft.config.missed_heartbeats*1000 &&
       (p->flags&(PEER_LEADER|PEER_FOLLOWS)))
        l5_confirm_contact(&n->raft,source,!!(p->flags&PEER_LEADER),p->sent/1000);
    if(p->flags&CONTACT_ONLY)return;
    struct l5_epoch_evidence*e=&n->epochs[source];
    *e=(struct l5_epoch_evidence){.global=p->raw+elapsed+p->delay,.local=p->at,
        .received=n->now_us,.age=p->age,.flags=p->flags,.valid=true};
    if((p->flags&EPOCH_VALID)&&source==n->raft.leader&&n->raft.role!=L5_LEADER){
        n->authority_leader=source;n->authority_term=p->term;
        n->authority_local=e->local;n->authority_global=e->global;n->authority_until=n->now_us+500000;
        l5_clock_verified_epoch(&n->clock,n->now_us,e->global+n->now_us-e->local,source,p->term);
    }
}
static void sample(void*v,unsigned source,uint32_t nonce,uint32_t low,uint64_t at,uint32_t delay)
{
    struct l5_node*n=v;authority_state(n);struct l5_time_probe*p=&n->probes[source];
    if(!p->active||p->nonce!=nonce||at<p->sent||n->now_us-p->sent>=PROBE_WINDOW_US)return;
    p->sample=true;p->low=low;p->at=at;p->delay=delay;finish_probe(n,source);
}
static void activate_time(struct l5_node*n)
{
    if(n->raft.role!=L5_LEADER||n->time_ready)return;
    unsigned valid=0,fresh=n->raft.pristine_start?1:0,replies=1,best=0,held=n->clock.established?1:0;
    for(unsigned p=1;p<=3;p++)if(p!=n->raft.id){
        struct l5_epoch_evidence*e=&n->epochs[p];
        if(!e->valid||n->now_us-e->received>=200000)continue;
        replies++;if(e->flags&EPOCH_PRISTINE)fresh++;
        if(e->flags&EPOCH_HELD)held++;
        if(e->flags&EPOCH_VALID){valid++;if(!best||e->age<n->epochs[best].age)best=p;}
    }
    if(valid){
        struct l5_epoch_evidence*b=&n->epochs[best];uint64_t target=b->global+n->now_us-b->local;
        for(unsigned p=1;p<=3;p++)if(p!=best&&p!=n->raft.id){
            struct l5_epoch_evidence*e=&n->epochs[p];
            if(!e->valid||!(e->flags&EPOCH_VALID)||n->now_us-e->received>=200000||e->age>b->age+100000)continue;
            int64_t delta=(int64_t)(e->global+n->now_us-e->local)-(int64_t)target;
            if(delta>2000||delta< -2000){n->time_conflict=true;return;}
        }
        n->time_conflict=false;
        int64_t delta=(int64_t)target-(int64_t)l5_global_gettime(&n->clock,n->now_us);
        if(n->clock.established&&(delta>100||delta< -100)){
            if(n->clock.last_ref<b->received||n->clock.leader!=n->raft.id){
                l5_clock_verified_epoch(&n->clock,n->now_us,target,n->raft.id,n->raft.meta.term);
                n->clock.recovering=true;
            }
            return;
        }
        if(!n->clock.established)l5_clock_verified_epoch(&n->clock,n->now_us,target,n->raft.id,n->raft.meta.term);
    }else{
        /* A fresh majority can create an epoch. Historical nodes require all
         * three confirmations: two rebooted nodes cannot rule out a hidden
         * holdover clock. Never fall back to boot time on a retry timeout. */
        if(held||n->now_us-n->activation_at<200000||!(fresh>=2||replies==3))return;
        if(n->clock.established)return;
    }
    l5_clock_master(&n->clock,n->now_us,n->raft.id,n->raft.meta.term);
    n->time_ready=true;n->raft.time_pending=false;
}
static void send_reference(void*v,unsigned leader,uint32_t term,uint8_t cycle)
{
    struct l5_node*n=v;authority_state(n);activate_time(n);
    struct l5_message ping={.type=L5_LEADER_PING,.source=leader,.term=term,
        .index=(uint32_t)n->now_us,.aux=(uint32_t)(n->now_us>>32)};
    n->pulse_history[cycle%4]=n->now_us;
    l5_wire_send(&n->wire,&ping,3);
    if(!n->time_ready)return;
    n->clock.last_ref=n->now_us;
    l5_wire_ref(&n->wire,term,cycle,l5_global_gettime(&n->clock,n->now_us));
    (void)leader;
}
static void message(void*v,const struct l5_message*m,uint64_t at)
{
    struct l5_node*n=v;
    if(m->type==L5_LEADER_PING||m->type==L5_LEADER_PONG){
        if(n->raft.paused||n->raft.fatal||m->term!=n->raft.meta.term)return;
        uint64_t stamp=((uint64_t)m->aux<<32)|m->index;
        if(m->type==L5_LEADER_PING){
            /* Only an already elected, known leader may renew its lease. */
            if(n->raft.role!=L5_FOLLOWER||n->raft.leader!=m->source)return;
            l5_confirm_contact(&n->raft,m->source,true,at/1000);alive(n,m->source,at);
            struct l5_message pong=*m;pong.type=L5_LEADER_PONG;pong.source=n->raft.id;
            l5_wire_send(&n->wire,&pong,3);
        }else if(n->raft.role==L5_LEADER&&stamp<=n->now_us&&n->now_us-stamp<
                 (uint64_t)n->raft.config.heartbeat_ms*n->raft.config.missed_heartbeats*1000){
            for(unsigned i=0;i<4;i++)if(n->pulse_history[i]==stamp&&stamp){
                l5_confirm_contact(&n->raft,m->source,false,stamp/1000);alive(n,m->source,stamp);break;
            }
        }
        return;
    }
    if(n->raft.config.debug&&n->trace)n->trace(n->context,m,false);
    if(m->type==L5_TIME_QUERY||m->type==L5_TIME_REPLY){
        if(m->destination!=n->raft.id||n->failed||n->raft.fatal)return;
        l5_receive(&n->raft,m,n->now_us/1000);authority_state(n);
        if(m->type==L5_TIME_QUERY){
            uint64_t stamp=l5_global_gettime(&n->clock,n->now_us);
            bool same_term=m->term==n->raft.meta.term;
            bool epoch=same_term&&!(m->ok&CONTACT_ONLY)&&n->clock.established&&!n->clock.recovering;
            uint64_t age=n->now_us-n->clock.last_ref;
            struct l5_message reply={.type=L5_TIME_REPLY,.source=n->raft.id,.destination=m->source,
                .term=m->term,.index=m->index,
                .ok=(epoch?EPOCH_VALID:0)|(n->clock.established?EPOCH_HELD:(n->raft.pristine_start?EPOCH_PRISTINE:0)),
                .event=(uint32_t)stamp,.aux=(uint32_t)(stamp>>32),.commit=age>UINT32_MAX?UINT32_MAX:(uint32_t)age};
            if(m->ok&CONTACT_ONLY)reply.ok=CONTACT_ONLY;
            if(same_term&&n->raft.role==L5_LEADER)reply.ok|=PEER_LEADER;
            if(same_term&&n->raft.role==L5_FOLLOWER&&n->raft.leader==m->source)reply.ok|=PEER_FOLLOWS;
            send_rpc(n,&reply);if(epoch)l5_wire_sample(&n->wire,m->source,m->index,stamp);
        }else{
            struct l5_time_probe*p=&n->probes[m->source];
            if(!p->active||p->nonce!=m->index||p->term!=m->term||n->now_us-p->sent>=PROBE_WINDOW_US)return;
            p->reply=true;p->flags=m->ok;p->raw=((uint64_t)m->aux<<32)|m->event;p->age=m->commit;
            finish_probe(n,m->source);
        }
        return;
    }
    if(m->type==L5_REF_ACK){
        if(!n->clock.established||m->term!=n->raft.meta.term)return;
        uint64_t echoed=((uint64_t)m->aux<<32)|m->index;
        uint64_t current=l5_global_gettime(&n->clock,at);
        if(n->authority_until>n->now_us&&n->authority_term==m->term&&n->authority_leader==n->raft.leader)
            current=n->authority_global+(at>=n->authority_local?at-n->authority_local:0);
        int64_t age=(int64_t)current-(int64_t)echoed;
        if(age< -2000 || age>=60000){probe(n,m->source);return;}
        alive(n,m->source,at);
    }
    l5_receive(&n->raft,m,n->now_us/1000);
}
static void reference(void*v,unsigned leader,uint8_t term,uint8_t cycle,uint64_t global,uint64_t at,uint32_t delay)
{
    struct l5_node*n=v;
    authority_state(n);
    if(leader!=n->raft.leader||term!=(n->raft.meta.term&255)||n->raft.role==L5_LEADER||n->failed)return;
    uint64_t expected=l5_global_gettime(&n->clock,at);
    bool authorized=n->authority_until>n->now_us&&n->authority_term==n->raft.meta.term&&n->authority_leader==leader;
    if(authorized)expected=n->authority_global+(at>=n->authority_local?at-n->authority_local:0);
    int64_t error=(int64_t)(global+delay)-(int64_t)expected;
    if(!n->clock.established||error< -2000||error>2000){probe(n,leader);return;}
    if(l5_receive_reference(&n->raft,leader,term,cycle,n->now_us/1000,global)){
        alive(n,leader,at);
        if(l5_clock_reference(&n->clock,at,global,delay,leader,n->raft.meta.term))
            if(n->clock_updated)n->clock_updated(n->context,l5_global_gettime(&n->clock,n->now_us));
    }
}
int l5_node_init(struct l5_node*n,unsigned id,const struct l5_storage*s,void*context,
                 void(*send)(void*,unsigned,uint16_t,const uint8_t*,bool),
                 void(*entry)(void*,const struct l5_entry*,bool),
                 void(*updated)(void*,uint64_t),uint64_t now)
{
    memset(n,0,sizeof(*n));n->context=context;n->send=send;n->entry=entry;n->clock_updated=updated;n->now_us=n->boot_us=now;
    struct l5_config cfg=L5_DEFAULT_CONFIG;
    l5_clock_init(&n->clock,now,now);
    l5_wire_init(&n->wire,id,cfg.network_delay_us,n,wire_send,message,reference);
    n->wire.sample=sample;
    for(unsigned p=1;p<=3;p++)n->wire.delay_by_hop[p]=cfg.path_delay_us[p][id];
    struct l5_callbacks cb={n,send_rpc,send_reference,applied};
    int rc=l5_init(&n->raft,id,&cfg,s,&cb,now/1000);
    if(!rc)authority_state(n);
    return rc;
}
static bool live(struct l5_node*n,unsigned p)
{return p==n->raft.id?!n->failed:n->peer_known[p]&&n->now_us-n->peer_seen[p]<(uint64_t)n->raft.config.heartbeat_ms*n->raft.config.missed_heartbeats*1000;}
void l5_node_tick(struct l5_node*n,uint64_t now,bool failed,bool input_ok)
{
    n->now_us=now;n->failed=failed;n->input_ok=input_ok;l5_pause(&n->raft,failed,now/1000);
    l5_clock_holdover(&n->clock,now);
    if(failed)memset(n->probes,0,sizeof(n->probes));
    authority_state(n);
    l5_tick(&n->raft,now/1000);
    if(failed||n->raft.fatal)return;
    authority_state(n);activate_time(n);
    for(unsigned p=1;p<=3;p++)if(p!=n->raft.id){
        bool stale=!n->peer_known[p]||now-n->peer_seen[p]>=30000;
        bool recovery=p==n->raft.leader&&(!n->clock.established||n->clock.recovering)&&now+250000>=n->authority_until;
        bool quorum_probe=n->raft.role==L5_LEADER&&now/1000-n->raft.last_ack[p]>=25;
        if(stale||recovery||quorum_probe||n->raft.time_pending)probe(n,p);
    }
    if(now>=n->next_report){
        const struct l5_network*g=l5_get_network(&n->raft);
        for(unsigned p=1;p<=3;p++){
            bool alive=live(n,p),known=g->nodes&(1u<<p);
            if(alive!=known && (alive||n->peer_known[p]||now-n->boot_us>=1000000))
                l5_client_submit(&n->raft,alive?L5_NODE_UP:L5_NODE_DOWN,p,0,now/1000);
        }
        if(g->nodes&(1u<<n->raft.id))for(unsigned b=0;b<2;b++){
            bool up=n->bus_known[b]&&now-n->bus_seen[b]<60000;
            if(up!=!!(g->links[n->raft.id]&(1u<<b)))
                l5_client_submit(&n->raft,up?L5_LINK_UP:L5_LINK_DOWN,n->raft.id,b,now/1000);
        }
        bool cockpit=live(n,1);
        if(n->raft.id==1)cockpit=cockpit&&input_ok;
        if((n->raft.id==1||!cockpit) && cockpit!=g->cockpit_ok)
            l5_client_submit(&n->raft,cockpit?L5_COCKPIT_OK:L5_COCKPIT_ERROR,1,0,now/1000);
        n->next_report=now+20000;
    }
    if(n->raft.id==2 && now>=n->next_event){
        l5_client_submit(&n->raft,L5_EVENT,2,0,now/1000);
        n->next_event=now+(uint64_t)n->raft.config.event_ms*1000;
    }
}
void l5_node_receive(struct l5_node*n,unsigned bus,uint16_t id,const uint8_t*d,unsigned dlc,uint64_t at,uint64_t now)
{
    n->now_us=now;
    if(bus<2&&(!n->bus_known[bus]||at>n->bus_seen[bus])){n->bus_seen[bus]=at;n->bus_known[bus]=true;}
    if(!n->failed)l5_wire_receive(&n->wire,bus,id,d,dlc,at,now);
}
void l5_node_legacy(struct l5_node*n,unsigned bus,uint16_t id,const uint8_t*d,bool fresh,uint64_t at)
{
    if(bus>1)return;
    if(!n->bus_known[bus]||at>n->bus_seen[bus]){n->bus_seen[bus]=at;n->bus_known[bus]=true;}
    unsigned source=origin(id,d);
    if(!fresh||source<1||source>3||source==n->raft.id)return;
    /* Lab5 liveness uses fresh REF/REF_ACK time, not delayed legacy HBs. */
    if(id==CAN_ID_FAULT&&d[FAULT_REASON]==FAULT_R_SELF_TEST){n->test_failures|=1u<<source;n->test_failed_at[source]=at;}
    /* Bounded first-copy bootstrap forwarding. Application receive-side dedup
     * prevents loops; every node can bridge when it is the remaining path. */
    if(!n->failed&&!n->raft.fatal)wire_send(n,1u<<(bus^1),id,d);
}
void l5_node_snapshot(struct l5_node*n,struct l5_snapshot*s)
{
    bool master=n->raft.role==L5_LEADER&&n->time_ready&&!n->raft.fatal&&!n->failed;
    unsigned degraded=0;
    if(n->bus_known[0]||n->bus_known[1])for(unsigned b=0;b<2;b++)
        if(!n->bus_known[b]||n->now_us-n->bus_seen[b]>=60000)degraded|=1u<<b;
    *s=(struct l5_snapshot){.clock=n->clock,.network=n->raft.network,.term=n->raft.meta.term,
        .commit=n->raft.meta.commit,.last_index=n->raft.last_index,.log_hash=n->raft.log_hash,
        .reference_timeout_us=(uint32_t)n->raft.config.heartbeat_ms*n->raft.config.missed_heartbeats*1000,
        .leader=n->raft.fatal?0:n->raft.leader,.id=n->raft.id,.bus_degraded=degraded,
        .is_leader=master,.synced=!n->raft.fatal&&!n->failed&&n->clock.synced&&(master||n->now_us-n->clock.last_ref<(uint64_t)n->raft.config.heartbeat_ms*n->raft.config.missed_heartbeats*1000),
        .cockpit_alive=live(n,1),.cockpit_ok=live(n,1)&&n->raft.network.cockpit_ok&&!n->test_failures&&n->clock.established,
        .local_failed=n->failed,.fatal=n->raft.fatal,.time_pending=n->raft.time_pending,.time_conflict=n->time_conflict};
}
unsigned l5_hazard_half_us(const struct l5_snapshot*s){return s->cockpit_alive?250000u:125000u;}
