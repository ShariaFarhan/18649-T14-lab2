#ifndef L5_NODE_H
#define L5_NODE_H
#include "raft.h"
#include "wire.h"
#include "time_sync.h"
struct l5_snapshot {
    struct l5_clock clock;
    struct l5_network network;
    uint32_t term,commit,last_index,log_hash,reference_timeout_us;
    uint8_t leader,id,bus_degraded;
    bool is_leader,synced,cockpit_alive,cockpit_ok,local_failed,fatal,time_pending,time_conflict;
};
struct l5_time_probe {
    uint32_t nonce,term,low,delay,age,generation;
    uint64_t sent,at,raw;
    uint8_t flags;
    bool active,reply,sample;
};
struct l5_epoch_evidence {
    uint64_t global,local,received;
    uint32_t age;
    uint8_t flags;
    bool valid;
};
struct l5_node {
    uint64_t pulse_history[4];
    struct l5_agent raft;struct l5_wire wire;struct l5_clock clock;
    uint64_t now_us,peer_seen[4],bus_seen[2],next_report,next_event,boot_us;
    bool peer_known[4],bus_known[2],input_ok,failed;
    uint32_t test_failures;
    uint64_t test_failed_at[4];
    struct l5_time_probe probes[4];
    struct l5_epoch_evidence epochs[4];
    uint64_t probe_due[4],authority_local,authority_global,authority_until,activation_at;
    uint32_t authority_term,observed_term,authority_generation;
    uint32_t nonce_boot;
    uint16_t nonce_round;
    uint8_t authority_leader,observed_leader;
    enum l5_role observed_role;
    bool time_ready,time_conflict;
    void *context;
    void (*send)(void *,unsigned,uint16_t,const uint8_t *,bool relay);
    void (*entry)(void *,const struct l5_entry *,bool replayed);
    void (*clock_updated)(void *,uint64_t global_us);
    void (*trace)(void *,const struct l5_message *,bool outgoing);
};
int l5_node_init(struct l5_node *,unsigned,const struct l5_storage *,void *,
                 void (*send)(void *,unsigned,uint16_t,const uint8_t *,bool),
                 void (*entry)(void *,const struct l5_entry *,bool),
                 void (*clock_updated)(void *,uint64_t),uint64_t now_us);
void l5_node_tick(struct l5_node *,uint64_t now_us,bool local_failed,bool input_ok);
void l5_node_receive(struct l5_node *,unsigned bus,uint16_t id,const uint8_t *,unsigned dlc,uint64_t at_us,uint64_t now_us);
void l5_node_legacy(struct l5_node *,unsigned bus,uint16_t id,const uint8_t *,bool new_message,uint64_t at_us);
void l5_node_snapshot(struct l5_node *,struct l5_snapshot *);
unsigned l5_snapshot_route(const struct l5_snapshot *,unsigned destination);
unsigned l5_hazard_half_us(const struct l5_snapshot *);
#endif
