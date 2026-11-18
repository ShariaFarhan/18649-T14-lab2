#include "raft.h"
#include <errno.h>
#include <string.h>
static unsigned bits(unsigned x){unsigned n=0;while(x){n+=x&1;x>>=1;}return n;}
static void hash_entry(struct l5_agent*r,const struct l5_entry*e)
{
    if(e->index<=r->hash_index)return;
    uint32_t words[]={e->term,e->index,e->event};
    for(unsigned i=0;i<3;i++)for(unsigned b=0;b<4;b++)r->log_hash=(r->log_hash^((words[i]>>(8*b))&255))*16777619u;
    r->hash_index=e->index;
}
static uint32_t random_next(struct l5_agent *r)
{uint32_t x=r->random;x^=x<<13;x^=x>>17;x^=x<<5;return r->random=x?x:1;}
static unsigned graph_slot(uint32_t event)
{
    unsigned kind=l5_kind(event),node=l5_node(event);
    if(kind==L5_COCKPIT_OK||kind==L5_COCKPIT_ERROR)return 0;
    if(kind==L5_NODE_UP||kind==L5_NODE_DOWN)return node;
    if(kind==L5_LINK_UP||kind==L5_LINK_DOWN)return 4+(node-1)*2+l5_bus(event);
    return 10;
}
static void project(struct l5_agent*r,const struct l5_entry*e)
{
    l5_apply(&r->projected,e->event);unsigned slot=graph_slot(e->event);
    if(slot<10)r->graph_version[slot]=e->index;
    if(l5_kind(e->event)==L5_NODE_DOWN||l5_kind(e->event)==L5_NODE_UP){
        unsigned first=4+(l5_node(e->event)-1)*2;r->graph_version[first]=r->graph_version[first+1]=e->index;
    }
}
static uint64_t timeout(struct l5_agent *r)
{return r->config.heartbeat_ms*r->config.missed_heartbeats+random_next(r)%(r->config.election_jitter_ms+1u);}
static bool save(struct l5_agent *r)
{
    if(!r->storage.save(r->storage.context,&r->meta))return true;
    r->fatal=true;r->role=L5_FOLLOWER;r->leader=0;return false;
}
static uint32_t term_at(struct l5_agent *r,uint32_t index)
{
    struct l5_entry e;
    if(!index)return 0;
    if(r->storage.read(r->storage.context,index,&e)){r->fatal=true;return UINT32_MAX;}
    return e.term;
}
static void emit(struct l5_agent *r,struct l5_message m)
{
    if(r->fatal||r->paused)return;
    m.source=r->id;r->callbacks.send(r->callbacks.context,&m);
}
static void follow(struct l5_agent *r,uint32_t term,unsigned leader,uint64_t now)
{
    if(term>r->meta.term){r->meta.term=term;r->meta.voted_for=0;if(!save(r))return;}
    r->role=L5_FOLLOWER;r->leader=leader;r->votes=0;
    r->election_due=now+timeout(r);
}
static void rebuild_projected(struct l5_agent *r)
{
    memset(&r->projected,0,sizeof(r->projected));
    memset(r->graph_version,0,sizeof(r->graph_version));
    memset(r->last_request,0,sizeof(r->last_request));memset(r->request_index,0,sizeof(r->request_index));
    for(uint32_t i=1;i<=r->last_index;i++){
        struct l5_entry e;if(r->storage.read(r->storage.context,i,&e)){r->fatal=true;return;}
        project(r,&e);
        unsigned c=l5_client(e.event);
        if(c){r->last_request[c]=l5_request(e.event);r->request_index[c]=i;}
    }
}
static void acknowledge(struct l5_agent *r,unsigned client,unsigned request,uint32_t index)
{
    if(client==r->id){
        for(unsigned i=0;i<r->pending_count;i++)if(l5_request(r->pending[i])==request){
            r->pending_count--;
            memmove(&r->pending[i],&r->pending[i+1],(r->pending_count-i)*sizeof(uint32_t));
            memmove(&r->pending_base[i],&r->pending_base[i+1],(r->pending_count-i)*sizeof(uint32_t));break;
        }
    }else emit(r,(struct l5_message){.type=L5_CLIENT_REPLY,.destination=client,.ok=1,
                                  .term=r->meta.term,.index=index,.aux=request});
}
static void apply_commit(struct l5_agent *r,uint32_t commit)
{
    if(commit>r->last_index)commit=r->last_index;
    if(commit<r->meta.commit)return;
    if(commit>r->meta.commit){r->meta.commit=commit;if(!save(r))return;}
    while(r->meta.applied<r->meta.commit){
        struct l5_entry e;uint32_t i=r->meta.applied+1;
        if(r->storage.read(r->storage.context,i,&e)){r->fatal=true;return;}
        l5_apply(&r->network,e.event);
        hash_entry(r,&e);
        /* Durable display watermark prevents repeat presentation after reboot.
         * A display should use index as its key, rather than treat stdout as a transaction. */
        r->meta.applied=i;if(!save(r))return;
        if(r->callbacks.applied)r->callbacks.applied(r->callbacks.context,&e,i<=r->replay_until);
        if(r->role==L5_LEADER && l5_client(e.event))acknowledge(r,l5_client(e.event),l5_request(e.event),i);
    }
    if(r->meta.applied>=r->replay_until)r->joining=false;
}
static void send_append(struct l5_agent *r,unsigned peer)
{
    if(peer==r->id)return;
    uint32_t next=r->next[peer];if(!next)next=1;
    if(next>r->last_index+1)next=r->last_index+1;
    struct l5_message m={.type=L5_APPEND,.destination=peer,.term=r->meta.term,
        .index=next-1,.previous_term=term_at(r,next-1),.commit=r->meta.commit};
    if(next<=r->last_index){
        struct l5_entry e;if(r->storage.read(r->storage.context,next,&e)){r->fatal=true;return;}
        m.ok=1;m.event=e.event;m.aux=e.term;
    }
    emit(r,m);
}
static void broadcast_append(struct l5_agent *r)
{for(unsigned p=1;p<=3;p++)if(p!=r->id)send_append(r,p);}
static bool append(struct l5_agent *r,uint32_t event)
{
    if(r->last_index>=r->config.log_size){r->fatal=true;return false;}
    struct l5_entry e={r->meta.term,r->last_index+1,event};
    if(r->storage.append(r->storage.context,&e)){r->fatal=true;return false;}
    r->last_index++;r->matched[r->id]=r->last_index;project(r,&e);
    unsigned c=l5_client(event);if(c){r->last_request[c]=l5_request(event);r->request_index[c]=r->last_index;}
    broadcast_append(r);return true;
}
static void become_leader(struct l5_agent *r,uint64_t now)
{
    r->role=L5_LEADER;r->leader=r->id;r->leader_seen=true;r->last_contact=now;
    r->next_ref=now;r->next_append=now;
    for(unsigned p=1;p<=3;p++){
        r->next[p]=r->last_index+1;r->matched[p]=0;r->last_ack[p]=now;
        r->ack_seen[p]=(r->votes&(1u<<p))!=0;
    }
    r->matched[r->id]=r->last_index;
    (void)append(r,0); /* Current-term entry permits committing an inherited prefix. */
}
static void candidate(struct l5_agent *r,uint64_t now)
{
    if(r->meta.term==UINT32_MAX){r->fatal=true;return;}
    r->meta.term++;r->meta.voted_for=r->id;if(!save(r))return;
    r->role=L5_CANDIDATE;r->leader=0;r->votes=1u<<r->id;r->election_due=now+timeout(r);
    emit(r,(struct l5_message){.type=L5_VOTE,.term=r->meta.term,
                              .index=r->last_index,.previous_term=term_at(r,r->last_index)});
}
static void prevote(struct l5_agent *r,uint64_t now)
{
    r->role=L5_PRE_CANDIDATE;r->leader=0;r->votes=1u<<r->id;r->election_due=now+timeout(r);
    emit(r,(struct l5_message){.type=L5_PREVOTE,.term=r->meta.term+1,
                              .index=r->last_index,.previous_term=term_at(r,r->last_index)});
}
int l5_init(struct l5_agent *r,unsigned id,const struct l5_config *cfg,
            const struct l5_storage *store,const struct l5_callbacks *cb,uint64_t now)
{
    if(id<1||id>3||cfg->n!=3||!cb->send||!cb->reference)return -EINVAL;
    memset(r,0,sizeof(*r));r->log_hash=2166136261u;r->id=id;r->config=*cfg;r->storage=*store;r->callbacks=*cb;
    r->random=cfg->seed^(0x9e3779b9u*id);r->joining=true;
    if(store->load(store->context,&r->meta,&r->last_index))return -EIO;
    r->request_sequence=r->meta.client_seq; /* Skip unused durable reservation after reboot. */
    if(r->last_index>cfg->log_size || r->meta.commit>r->last_index || r->meta.applied>r->meta.commit)return -EINVAL;
    for(uint32_t i=1;i<=r->meta.commit;i++){
        struct l5_entry e;if(store->read(store->context,i,&e))return -EIO;l5_apply(&r->network,e.event);hash_entry(r,&e);
    }
    rebuild_projected(r);r->last_contact=now;r->election_due=now+timeout(r);
    bool pristine=!r->meta.term;
    r->pristine_start=pristine;r->startup_until=now+cfg->startup_grace_ms;
    if(!pristine)r->leader_seen=true; /* Recovery must not wait forever for the initial default leader. */
    if(pristine){r->meta.term=1;r->meta.voted_for=cfg->default_leader;if(!save(r))return -EIO;}
    if(pristine && cfg->default_leader){
        r->leader=cfg->default_leader;
        if(id==cfg->default_leader){r->role=L5_LEADER;r->next_ref=now;r->next_append=now;
            for(unsigned p=1;p<=3;p++)r->next[p]=r->last_index+1;
            (void)append(r,0);
        }
    }
    return r->fatal ? -EIO : 0;
}
static void client_request(struct l5_agent *r,uint32_t event,uint32_t observed_commit)
{
    unsigned client=l5_client(event),request=l5_request(event);
    if(!l5_event_valid(event)||!client)return;
    if(request<=r->last_request[client]){
        if(r->request_index[client]<=r->meta.commit)acknowledge(r,client,request,r->request_index[client]);
        return;
    }
    if(!l5_event_changes(&r->projected,event)){
        if(!l5_event_changes(&r->network,event))acknowledge(r,client,request,r->meta.commit);
        return;
    }
    unsigned slot=graph_slot(event);
    if(slot<10&&r->graph_version[slot]>observed_commit){
        /* An old observation must not undo a newer graph transition. The
         * client rereads its committed graph and reports again if still needed. */
        if(client==r->id)acknowledge(r,client,request,r->meta.commit);
        else emit(r,(struct l5_message){.type=L5_CLIENT_REPLY,.destination=client,.ok=2,
            .term=r->meta.term,.index=r->meta.commit,.aux=request});
        return;
    }
    (void)append(r,event);
}
void l5_tick(struct l5_agent *r,uint64_t now)
{
    if(r->paused||r->fatal)return;
    if(r->role==L5_LEADER){
        unsigned available=1;
        for(unsigned p=1;p<=3;p++)if(p!=r->id && r->ack_seen[p] && now-r->last_ack[p]<(uint64_t)r->config.heartbeat_ms*r->config.missed_heartbeats)available++;
        if(available>=2)r->ever_quorum=true;
        if(r->ever_quorum && available<2){follow(r,r->meta.term,0,now);return;}
        if(now>=r->next_ref){r->callbacks.reference(r->callbacks.context,r->id,r->meta.term,r->cycle++);
            r->next_ref=now+r->config.heartbeat_ms;}
        if(now>=r->next_append){broadcast_append(r);r->next_append=now+(r->time_pending?r->config.heartbeat_ms:r->config.append_ms);}
    }else if(now>=r->election_due){
        bool waiting_default=r->config.default_leader && !r->leader_seen && r->meta.term==1 && now<r->startup_until;
        if(!waiting_default)prevote(r,now);
        else r->election_due=now+timeout(r);
    }
    if(r->pending_count && now>=r->next_client){
        uint32_t event=r->pending[0];r->next_client=now+100;
        if(r->role==L5_LEADER)client_request(r,event,r->pending_base[0]);
        else if(r->leader)emit(r,(struct l5_message){.type=L5_CLIENT,.destination=r->leader,.term=r->meta.term,.event=event,.commit=r->pending_base[0]});
    }
}
bool l5_client_submit(struct l5_agent *r,unsigned kind,unsigned node,unsigned bus,uint64_t now)
{
    if(r->pending_count==16 || r->request_sequence>=65535 || r->fatal)return false;
    for(unsigned i=0;i<r->pending_count;i++)if(l5_kind(r->pending[i])==kind && l5_node(r->pending[i])==node && l5_bus(r->pending[i])==bus)return true;
    uint32_t event=l5_event(kind,node,bus,r->id,r->request_sequence+1);
    if(!l5_event_valid(event))return false;
    uint16_t reserved;if(!l5_reserve_id(r,&reserved))return false;
    r->pending_base[r->pending_count]=r->meta.commit;r->pending[r->pending_count++]=event;r->next_client=now;return true;
}
bool l5_reserve_id(struct l5_agent*r,uint16_t*id)
{
    if(r->fatal||r->request_sequence>=65535)return false;
    if(r->request_sequence==r->meta.client_seq){
        r->meta.client_seq=r->meta.client_seq>65535-128?65535:r->meta.client_seq+128;
        if(!save(r))return false;
    }
    *id=(uint16_t)++r->request_sequence;return true;
}
void l5_receive(struct l5_agent *r,const struct l5_message *m,uint64_t now)
{
    if(r->paused||r->fatal||m->source<1||m->source>3||m->source==r->id||
       (m->destination && m->destination!=r->id))return;
    bool pre=m->type==L5_PREVOTE||m->type==L5_PREVOTE_REPLY;
    if(!pre && m->term>r->meta.term)follow(r,m->term,0,now);
    if(r->fatal)return;
    uint32_t own_last_term=term_at(r,r->last_index);
    bool up_to_date=m->previous_term>own_last_term || (m->previous_term==own_last_term && m->index>=r->last_index);
    switch(m->type){
    case L5_PREVOTE:{
        bool grant=m->term>=r->meta.term+1 && up_to_date &&
            now-r->last_contact>=r->config.heartbeat_ms*r->config.missed_heartbeats && r->role!=L5_LEADER;
        emit(r,(struct l5_message){.type=L5_PREVOTE_REPLY,.destination=m->source,.term=m->term,.ok=grant});break;
    }
    case L5_PREVOTE_REPLY:
        if(r->role==L5_PRE_CANDIDATE && m->term==r->meta.term+1 && m->ok){
            r->votes|=1u<<m->source;if(bits(r->votes)>=2)candidate(r,now);
        }break;
    case L5_VOTE:{
        bool grant=m->term==r->meta.term && up_to_date && (!r->meta.voted_for||r->meta.voted_for==m->source);
        if(grant){r->meta.voted_for=m->source;if(!save(r))return;r->election_due=now+timeout(r);}
        emit(r,(struct l5_message){.type=L5_VOTE_REPLY,.destination=m->source,.term=r->meta.term,.ok=grant});break;
    }
    case L5_VOTE_REPLY:
        if(r->role==L5_CANDIDATE && m->term==r->meta.term && m->ok){
            r->votes|=1u<<m->source;if(bits(r->votes)>=2){r->ever_quorum=true;become_leader(r,now);}
        }break;
    case L5_APPEND:{
        struct l5_message response={.type=L5_APPEND_REPLY,.destination=m->source,.term=r->meta.term,
                                   .index=r->last_index+1,.aux=m->index};
        if(m->term!=r->meta.term){emit(r,response);break;}
        follow(r,m->term,m->source,now);r->leader_seen=true;r->last_contact=now;
        if(r->joining && m->commit>r->replay_until)r->replay_until=m->commit;
        if(m->index>r->last_index || term_at(r,m->index)!=m->previous_term){emit(r,response);break;}
        uint32_t match=m->index;
        if(m->ok){
            if(!l5_event_valid(m->event)||!m->aux||m->aux>m->term){emit(r,response);break;}
            uint32_t index=m->index+1;
            if(index<=r->last_index){
                struct l5_entry old;if(r->storage.read(r->storage.context,index,&old)){r->fatal=true;return;}
                if(old.term!=m->aux || old.event!=m->event){
                    if(index<=r->meta.commit){emit(r,response);break;}
                    if(r->storage.truncate(r->storage.context,index-1)){r->fatal=true;return;}
                    r->last_index=index-1;rebuild_projected(r);
                }
            }
            if(index>r->last_index){
                struct l5_entry e={m->aux,index,m->event};
                if(index>r->config.log_size || r->storage.append(r->storage.context,&e)){r->fatal=true;return;}
                r->last_index=index;project(r,&e);
                unsigned client=l5_client(e.event);
                if(client){r->last_request[client]=l5_request(e.event);r->request_index[client]=index;}
            }
            match=index;
        }
        /* Commit only the prefix verified by this AppendEntries, never a
         * follower's unverified/conflicting suffix beyond the matched index. */
        apply_commit(r,m->commit<match?m->commit:match);
        response.ok=1;response.index=match;emit(r,response);break;
    }
    case L5_APPEND_REPLY:
        if(r->role!=L5_LEADER||m->term!=r->meta.term)break;
        r->last_ack[m->source]=now;r->ack_seen[m->source]=true;
        if(m->ok && m->index<=r->last_index){
            if(m->index>r->matched[m->source])r->matched[m->source]=m->index;
            r->next[m->source]=r->matched[m->source]+1;
            for(uint32_t i=r->last_index;i>r->meta.commit;i--){
                unsigned votes=1;for(unsigned p=1;p<=3;p++)if(p!=r->id && r->matched[p]>=i)votes++;
                if(votes>=2 && term_at(r,i)==r->meta.term){apply_commit(r,i);broadcast_append(r);break;}
            }
            if(r->next[m->source]<=r->last_index)send_append(r,m->source);
        }else if(!m->ok && m->aux==r->next[m->source]-1){
            uint32_t next=r->next[m->source]>1?r->next[m->source]-1:1;
            if(m->index && m->index<next)next=m->index;
            r->next[m->source]=next;send_append(r,m->source);
        }break;
    case L5_CLIENT:
        if(r->role==L5_LEADER)client_request(r,m->event,m->commit);
        else emit(r,(struct l5_message){.type=L5_CLIENT_REPLY,.destination=m->source,.term=r->meta.term,.aux=r->leader});
        break;
    case L5_CLIENT_REPLY:
        if(m->ok)acknowledge(r,r->id,m->aux,m->index);
        else if(m->aux>=1 && m->aux<=3)r->leader=m->aux;
        break;
    case L5_REF_ACK:
        if(r->role==L5_LEADER && m->term==r->meta.term){r->last_ack[m->source]=now;r->ack_seen[m->source]=true;}
        break;
    default:break;
    }
}
bool l5_receive_reference(struct l5_agent *r,unsigned leader,uint8_t term_low,uint8_t cycle,uint64_t now,uint64_t reference_time)
{
    (void)cycle; /* The wire layer deduplicates reference cycles. */
    if(r->paused||r->fatal||leader!=r->leader||term_low!=(r->meta.term&255)||r->role==L5_LEADER)return false;
    r->leader_seen=true;r->last_contact=now;r->election_due=now+timeout(r);
    emit(r,(struct l5_message){.type=L5_REF_ACK,.destination=leader,.term=r->meta.term,
                              .aux=(uint32_t)(reference_time>>32),.index=(uint32_t)reference_time});return true;
}
void l5_pause(struct l5_agent *r,bool paused,uint64_t now)
{
    if(r->paused==paused)return;
    r->paused=paused;
    if(paused){r->role=L5_FOLLOWER;r->leader=0;}
    r->last_contact=now;r->election_due=now+timeout(r);
}
const struct l5_network *l5_get_network(const struct l5_agent *r){return &r->network;}
