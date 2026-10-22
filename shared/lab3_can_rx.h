#ifndef LAB3_CAN_RX_H
#define LAB3_CAN_RX_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "lab3_can_catalog.h"
#define LAB3_HEARTBEAT_PERIOD_MS 20u
#define LAB3_HEARTBEAT_TIMEOUT_MS (3u * LAB3_HEARTBEAT_PERIOD_MS)

/* Single-owner dispatch state. Bus freshness and application delivery are
 * independent: a delayed bus may advance without applying an older command.
 * After silence, a backward sequence needs two advancing samples to rebase.
 * A fixed replay never refreshes liveness. Four bits still cannot distinguish
 * arbitrary delayed traffic from a reboot without an epoch or larger counter. */
enum lab3_rx_result { LAB3_RX_NEW, LAB3_RX_DUPLICATE, LAB3_RX_REPEAT,
                      LAB3_RX_OLD, LAB3_RX_CONFLICT, LAB3_RX_INVALID };
struct lab3_bus_sequence {
    bool seen, candidate;
    uint8_t seq, data[8], candidate_seq;
    uint64_t advanced_ms, candidate_ms;
};
struct lab3_rx_slot {
    bool seen;
    uint8_t seq, buses, data[8];
    uint64_t accepted_ms;
    struct lab3_bus_sequence bus[2];
};
struct lab3_can_rx {
    struct lab3_rx_slot slots[9];
    uint32_t counts[6];
    bool bus_advanced;
    bool hb_seen[3][2];
    uint64_t hb_ms[3][2];
    uint32_t hb_bus_count[3][2], hb_applied[3];
};
static inline int lab3_hb_peer(uint16_t id)
{
    return id >= CAN_ID_HB_N1 && id <= CAN_ID_HB_N3 ? (int)(id - CAN_ID_HB_N1) : -1;
}
static inline int lab3_slot(uint16_t id, const uint8_t *d)
{
    switch (id) {
    case CAN_ID_CTRL: return 0;
    /* FAULT has multiple producers. Do not alias their independent counters. */
    case CAN_ID_FAULT: return d[FAULT_ZONE] >= 1 && d[FAULT_ZONE] <= 3 ? d[FAULT_ZONE] : -1;
    case CAN_ID_HB_N1: return 4;
    case CAN_ID_HB_N2: return 5;
    case CAN_ID_HB_N3: return 6;
    case CAN_ID_BLINK: return 7;
    case CAN_ID_FORCE: return 8;
    default: return -1;
    }
}
static inline bool lab3_payload_valid(uint16_t id, const uint8_t *d)
{
    if (lab3_hb_peer(id) >= 0)
        return d[HB_STATE] <= ZONE_STATE_LOCAL_FAULT && d[HB_BUS] <= 3 &&
               d[HB_REASON] <= 5 && d[HB_DUTY] <= 100;
    if (id == CAN_ID_FAULT)
        return d[FAULT_ZONE] >= 1 && d[FAULT_ZONE] <= 3 &&
               d[FAULT_REASON] >= FAULT_R_SELF_TEST && d[FAULT_REASON] <= FAULT_R_SHUTDOWN;
    if (id == CAN_ID_BLINK) return d[BLINK_MODE] <= BLINK_MODE_HAZARD && d[BLINK_PHASE] <= 1;
    if (id == CAN_ID_FORCE) return (int8_t)d[FORCE_VALUE] >= FORCE_MIN && (int8_t)d[FORCE_VALUE] <= FORCE_MAX;
    if (id == CAN_ID_CTRL) {
        int16_t steer = (int16_t)((uint16_t)d[1] | ((uint16_t)d[2] << 8));
        return steer >= CTRL_STEER_MIN && steer <= CTRL_STEER_MAX &&
               d[CTRL_THROTTLE] <= 100 && d[CTRL_BRAKE] <= 100 &&
               !(can_flags_of(d) & ~(CTRL_FLAG_UDP_STALE | CTRL_FLAG_DUAL_ENABLE)) &&
               !(d[CTRL_BUTTONS] & ~(CTRL_BTN_LSB | CTRL_BTN_RSB | CTRL_BTN_TEST)) &&
               d[CTRL_RESERVED] == 0 &&
               d[CTRL_CRC] == can_crc8(d, CTRL_CRC);
    }
    return false;
}
static inline enum lab3_rx_result lab3_bus_observe(struct lab3_bus_sequence *b,
                                                  const uint8_t *d, uint64_t now)
{
    uint8_t seq = can_seq_of(d);
    if (b->seen) {
        if (seq == b->seq)
            return memcmp(b->data,d,8) ? LAB3_RX_CONFLICT : LAB3_RX_REPEAT;
        if (!can_seq_is_new(b->seq,seq)) {
            if (now - b->advanced_ms < CAN_DEDUP_RESET_MS) return LAB3_RX_OLD;
            if (!(b->candidate && now - b->candidate_ms < CAN_DEDUP_RESET_MS &&
                  can_seq_is_new(b->candidate_seq,seq))) {
                if (!b->candidate || seq != b->candidate_seq) {
                    b->candidate = true; b->candidate_seq = seq; b->candidate_ms = now;
                }
                return LAB3_RX_OLD;
            }
        }
    }
    b->seen=true; b->candidate=false; b->seq=seq; b->advanced_ms=now;
    memcpy(b->data,d,8);
    return LAB3_RX_NEW;
}
static inline enum lab3_rx_result lab3_receive(struct lab3_can_rx *r, uint16_t id,
                         unsigned bus, const uint8_t *d, unsigned dlc, uint64_t now)
{
    enum lab3_rx_result result = LAB3_RX_INVALID;
    r->bus_advanced = false;
    if (bus > 1 || dlc != 8 || !d || !lab3_payload_valid(id,d)) goto done;
    int key=lab3_slot(id,d);
    if(key<0) goto done;
    struct lab3_rx_slot *s=&r->slots[key];
    uint8_t seq=can_seq_of(d), bit=(uint8_t)(1u<<bus);
    /* Reject conflicting copies before they can refresh bus health. */
    if(s->seen && seq==s->seq && memcmp(s->data,d,8) &&
       now-s->accepted_ms<CAN_DEDUP_RESET_MS) { result=LAB3_RX_CONFLICT; goto done; }
    result=lab3_bus_observe(&s->bus[bus],d,now);
    if(result!=LAB3_RX_NEW) goto done;
    r->bus_advanced=true;
    int peer=lab3_hb_peer(id);
    if(peer>=0) {
        r->hb_seen[peer][bus]=true; r->hb_ms[peer][bus]=now;
        ++r->hb_bus_count[peer][bus];
    }
    if(s->seen && seq==s->seq) {
        result=memcmp(s->data,d,8) ? LAB3_RX_CONFLICT : LAB3_RX_DUPLICATE;
        s->buses|=bit;
    } else if(!s->seen || can_seq_is_new(s->seq,seq) ||
              now-s->accepted_ms>=CAN_DEDUP_RESET_MS) {
        s->seen=true; s->seq=seq; s->buses=bit; s->accepted_ms=now;
        memcpy(s->data,d,8); result=LAB3_RX_NEW;
        if(peer>=0) ++r->hb_applied[peer];
    } else result=LAB3_RX_OLD; /* Alive delayed bus; never reapply an old command. */
done:
    ++r->counts[result];
    return result;
}
static inline bool lab3_hb_fresh(const struct lab3_can_rx *r, unsigned peer,
                                unsigned bus, uint64_t now)
{
    return peer < 3 && bus < 2 && r->hb_seen[peer][bus] &&
           now - r->hb_ms[peer][bus] < LAB3_HEARTBEAT_TIMEOUT_MS;
}
struct lab3_health { uint8_t degraded, missing_peers; bool isolated; };
static inline struct lab3_health lab3_health_get(const struct lab3_can_rx *r,
                                                unsigned own_peer, uint64_t now)
{
    struct lab3_health h = {0};
    bool fresh_bus[2] = {false,false}, observed = false;
    for (unsigned p = 0; p < 3; ++p) {
        if (p == own_peer) continue;
        bool a = lab3_hb_fresh(r,p,0,now), b = lab3_hb_fresh(r,p,1,now);
        fresh_bus[0] |= a; fresh_bus[1] |= b;
        observed |= r->hb_seen[p][0] || r->hb_seen[p][1];
        if (!a && !b) h.missing_peers |= 1u << p;
    }
    /* Startup without evidence of any peer is STARTUP, not two failed wires.
     * Losing one peer on A must not light A if another peer still uses A.
     * Zone loss is tracked separately across BOTH buses. */
    if (observed) {
        if (!fresh_bus[0]) h.degraded |= HB_BUS_A_DEGRADED;
        if (!fresh_bus[1]) h.degraded |= HB_BUS_B_DEGRADED;
    }
    h.isolated = observed && !fresh_bus[0] && !fresh_bus[1];
    return h;
}
static inline uint16_t lab3_hb_applied_total(const struct lab3_can_rx *r)
{
    return (uint16_t)(r->hb_applied[0] + r->hb_applied[1] + r->hb_applied[2]);
}
#endif
