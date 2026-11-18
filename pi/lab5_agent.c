#define _GNU_SOURCE
#include "lab5_agent.h"
#include "../lab5/posix_store.h"
#include "../shared/lab3_can_rx.h"
#include "../lab5/tx_queue.h"
#include "../lab5/event_display.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
static struct {
    struct l5_node node;struct l5_posix_store store;struct lab3_can_rx legacy;
    struct l5_snapshot snapshot;pthread_mutex_t snapshot_lock;
    int sockets[2];pthread_t agent_thread,clock_thread;
    atomic_bool running,input,failed,error;
    struct l5_tx_queue outgoing[2];
    struct l5_event_display display;
    void *context;void (*pin)(void *,enum l5_pi_pin,bool);
} state;
static uint64_t us(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000+t.tv_nsec/1000;}
static void send_frame(void*v,unsigned buses,uint16_t id,const uint8_t*d,bool relay)
{
    (void)v;
    if(atomic_load(&state.failed))return;
    for(unsigned b=0;b<2;b++)if(buses&(1u<<b))l5_tx_add(&state.outgoing[b],id,d,state.node.now_us,relay);
}
static void flush(void)
{
    static bool level;
    for(unsigned b=0;b<2;b++){
        if(atomic_load(&state.failed)){state.outgoing[b].count=0;continue;}
        for(unsigned n=0;n<16;n++){
            struct l5_tx_item o;unsigned slot;uint64_t now=us();
            if(!l5_tx_pick(&state.outgoing[b],now,&o,&slot))break;
            l5_tx_reference_delay(&o,now);
            struct can_frame f={.can_id=o.id,.can_dlc=8};memcpy(f.data,o.data,8);
            if(write(state.sockets[b],&f,sizeof(f))!=(ssize_t)sizeof(f)){
                if(errno==EAGAIN||errno==ENOBUFS)break;
            }else {
                state.pin(state.context,b==0?L5_PI_CAN_TX_A:L5_PI_CAN_TX_B,true);
                if(o.relay){level=!level;state.pin(state.context,L5_PI_RELAY,level);}
            }
            l5_tx_remove(&state.outgoing[b],slot);
        }
    }
}
static void entry(void*v,const struct l5_entry*e,bool replayed)
{
    (void)v;
    int rc=l5_display_record(&state.display,e,replayed);
    if(rc)fprintf(stderr,"LAB5 event display write failed: %d (committed log remains source of truth)\n",rc);
    printf("LAB5 LOG term=%u index=%u kind=%u node=%u bus=%u client=%u request=%u %s\n",
        e->term,e->index,l5_kind(e->event),l5_node(e->event),l5_bus(e->event),l5_client(e->event),l5_request(e->event),replayed?"replayed":"live");
}
static void trace(void*v,const struct l5_message*m,bool tx)
{
    (void)v;if(m->type>6)return;
    printf("LAB5 RAFT %s type=%u src=%u dst=%u term=%u index=%u commit=%u ok=%u\n",
           tx?"TX":"RX",m->type,m->source,m->destination,m->term,m->index,m->commit,m->ok);
}
static int can_open(const char*name)
{
    unsigned idx=if_nametoindex(name);if(!idx)return -1;
    int fd=socket(PF_CAN,SOCK_RAW|SOCK_NONBLOCK,CAN_RAW);if(fd<0)return -1;
    struct sockaddr_can a={.can_family=AF_CAN,.can_ifindex=(int)idx};
    if(bind(fd,(struct sockaddr*)&a,sizeof(a))){close(fd);return -1;}
    return fd;
}
struct l5_snapshot lab5_pi_snapshot(void)
{pthread_mutex_lock(&state.snapshot_lock);struct l5_snapshot s=state.snapshot;pthread_mutex_unlock(&state.snapshot_lock);return s;}
static void *agent(void*v)
{
    (void)v;bool ref_level=false;uint64_t next_status=0;
    while(atomic_load(&state.running)){
        struct pollfd fds[2]={{state.sockets[0],POLLIN,0},{state.sockets[1],POLLIN,0}};
        poll(fds,2,1);
        for(unsigned b=0;b<2;b++)for(unsigned n=0;n<32;n++){
            struct can_frame f;if(read(state.sockets[b],&f,sizeof(f))!=(ssize_t)sizeof(f))break;
            if(f.can_id&(CAN_EFF_FLAG|CAN_RTR_FLAG|CAN_ERR_FLAG)||f.can_dlc!=8)continue;
            uint64_t now=us();
            if(l5_wire_id(f.can_id)){
                if(f.can_id<0x80){ref_level=!ref_level;state.pin(state.context,L5_PI_REF_RX,ref_level);}
                l5_node_receive(&state.node,b,f.can_id,f.data,8,now,now);
            }else{
                enum lab3_rx_result r=lab3_receive(&state.legacy,f.can_id,b,f.data,8,now/1000);
                if(r!=LAB3_RX_INVALID&&r!=LAB3_RX_CONFLICT)
                    l5_node_legacy(&state.node,b,f.can_id,f.data,r==LAB3_RX_NEW,now);
            }
        }
        uint64_t now=us();l5_node_tick(&state.node,now,atomic_load(&state.failed),atomic_load(&state.input));
        flush();
        struct l5_snapshot snapshot;l5_node_snapshot(&state.node,&snapshot);
        pthread_mutex_lock(&state.snapshot_lock);state.snapshot=snapshot;pthread_mutex_unlock(&state.snapshot_lock);
        if(now>=next_status){
            printf("LAB5 NETWORK leader=%u term=%u commit=%u hash=%08x nodes=0x%x links=%u/%u/%u cockpit=%u time_pending=%u time_conflict=%u\n",
                snapshot.leader,snapshot.term,snapshot.commit,snapshot.log_hash,snapshot.network.nodes,snapshot.network.links[1],snapshot.network.links[2],snapshot.network.links[3],snapshot.network.cockpit_ok,snapshot.time_pending,snapshot.time_conflict);
            next_status=now+1000000;
        }
    }
    return NULL;
}
static void *clock_output(void*v)
{
    (void)v;uint64_t last_second=UINT64_MAX,high_until=0,last_global=0;
    while(atomic_load(&state.running)){
        struct l5_snapshot s=lab5_pi_snapshot();uint64_t local=us();
        uint64_t global=l5_global_gettime(&s.clock,local);
        if(s.clock.established){if(global<last_global)global=last_global;last_global=global;}
        bool synced=s.synced && (s.is_leader||local-s.clock.last_ref<s.reference_timeout_us);
        state.pin(state.context,L5_PI_LEADER,s.is_leader&&!s.local_failed);
        state.pin(state.context,L5_PI_SYNC,synced);
        if(s.clock.established && last_second==UINT64_MAX)last_second=global/1000000;
        if(s.clock.established && global/1000000!=last_second){
            last_second=global/1000000;high_until=local+1000;state.pin(state.context,L5_PI_PPS,true);
            printf("LAB5 PPS global_us=%llu local_us=%llu drift_ppb=%d ratio=%d/1000000000 correction_us=%lld SYNC=%d\n",
                (unsigned long long)global,(unsigned long long)local,s.clock.drift_ppb,1000000000+s.clock.drift_ppb,(long long)l5_clock_correction(&s.clock,local),synced);
        }
        if(local>=high_until)state.pin(state.context,L5_PI_PPS,false);
        state.pin(state.context,L5_PI_HAZARD,atomic_load(&state.error)&&((global/l5_hazard_half_us(&s))%2==0));
        struct timespec delay={.tv_nsec=100000};nanosleep(&delay,NULL);
    }
    return NULL;
}
int lab5_pi_start(const char*log,const char*a,const char*b,void*context,void(*pin)(void*,enum l5_pi_pin,bool))
{
    memset(&state,0,sizeof(state));state.context=context;state.pin=pin;state.sockets[0]=state.sockets[1]=-1;
    pthread_mutex_init(&state.snapshot_lock,NULL);
    struct l5_storage store;int rc=l5_posix_open(&state.store,log,&store);if(rc)return rc;
    if((rc=l5_display_open(&state.display,log))){l5_posix_close(&state.store);return rc;}
    state.sockets[0]=can_open(a);state.sockets[1]=can_open(b);
    if(state.sockets[0]<0||state.sockets[1]<0){lab5_pi_stop();return -ENODEV;}
    rc=l5_node_init(&state.node,1,&store,NULL,send_frame,entry,NULL,us());
    if(rc){lab5_pi_stop();return rc;}
    if((rc=l5_display_repair(&state.display,&store,state.node.raft.meta.commit))){lab5_pi_stop();return rc;}
    state.node.trace=trace;
    l5_node_snapshot(&state.node,&state.snapshot);atomic_store(&state.running,true);
    rc=pthread_create(&state.agent_thread,NULL,agent,NULL);
    if(rc){atomic_store(&state.running,false);lab5_pi_stop();return -rc;}
    rc=pthread_create(&state.clock_thread,NULL,clock_output,NULL);
    if(rc){atomic_store(&state.running,false);pthread_join(state.agent_thread,NULL);lab5_pi_stop();return -rc;}
    return 0;
}
void lab5_pi_input(bool valid,bool failed,bool error)
{atomic_store(&state.input,valid);atomic_store(&state.failed,failed);atomic_store(&state.error,error);}
void lab5_pi_stop(void)
{
    if(atomic_exchange(&state.running,false)){pthread_join(state.agent_thread,NULL);pthread_join(state.clock_thread,NULL);}
    for(unsigned b=0;b<2;b++)if(state.sockets[b]>=0){close(state.sockets[b]);state.sockets[b]=-1;}
    l5_posix_close(&state.store);
}
