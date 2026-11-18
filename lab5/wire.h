#ifndef L5_WIRE_H
#define L5_WIRE_H
#include "raft.h"
/* Last-hop IDs differ between transmitters: a forwarded REF may compensate
 * residence time without colliding with its origin's different CAN payload. */
#define L5_REF_BASE 0x040u
#define L5_ACK_BASE 0x080u
#define L5_SAMPLE_BASE 0x0c0u
#define L5_QUERY_BASE 0x140u
#define L5_PROOF_BASE 0x180u
#define L5_TIME_REPLY_BASE 0x1c0u
#define L5_RPC_BASE 0x500u
struct l5_wire_slot {
    uint8_t bytes[36],transaction,seen[2],forwarded[2];
    uint64_t touched;
    bool active,complete,bad;
};
struct l5_reference_seen {uint8_t cycle,term,seen,forwarded;uint16_t full_term;uint64_t at,stamp;bool valid;};
struct l5_sample_seen {uint32_t nonce;bool valid;};
struct l5_wire {
    struct l5_wire_slot slots[11][4][4];
    struct l5_reference_seen refs[4],acks[4];
    struct l5_sample_seen samples[4][4];
    struct l5_sample_seen compact[2][4][4];
    uint8_t transactions[12],id;
    uint32_t delay_us;
    uint32_t delay_by_hop[4];
    void *context;
    void (*send_frame)(void *,unsigned buses,uint16_t id,const uint8_t data[8]);
    void (*message)(void *,const struct l5_message *,uint64_t local_us);
    void (*reference)(void *,unsigned leader,uint8_t term,uint8_t cycle,uint64_t global_us,uint64_t local_us,uint32_t delay_us);
    void (*sample)(void *,unsigned source,uint32_t nonce,uint32_t time_low,uint64_t local_us,uint32_t delay_us);
};
void l5_wire_init(struct l5_wire *,unsigned id,uint32_t delay_us,void *context,
                  void (*send)(void *,unsigned,uint16_t,const uint8_t *),
                  void (*message)(void *,const struct l5_message *,uint64_t),
                  void (*reference)(void *,unsigned,uint8_t,uint8_t,uint64_t,uint64_t,uint32_t));
void l5_wire_send(struct l5_wire *,const struct l5_message *,unsigned buses);
void l5_wire_ref(struct l5_wire *,uint32_t term,uint8_t cycle,uint64_t global_us);
void l5_wire_sample(struct l5_wire *,unsigned destination,uint32_t nonce,uint64_t global_us);
bool l5_wire_receive(struct l5_wire *,unsigned bus,uint16_t id,const uint8_t *,unsigned dlc,uint64_t rx_us,uint64_t now_us);
bool l5_wire_id(uint16_t id);
#endif
