#ifndef LAB3_RELAY_H
#define LAB3_RELAY_H
#include "lab3_can_rx.h"

/* N2 is the only relay. Call after the normal receive validator/deduplicator.
 * The caller serializes these operations. Forward original bytes, never seq++. */
#define LAB3_RELAY_MAX_AGE_MS 20u
struct lab3_relay_slot {
    bool valid;
    uint16_t id;
    uint8_t data[8], seen, pending;
    uint32_t revision;
    uint64_t expires;
};
struct lab3_relay_plan {
    struct lab3_relay_slot slots[9];
    uint32_t offered, suppressed, superseded, expired;
};
struct lab3_relay_copy {
    uint16_t id;
    uint8_t data[8];
    uint32_t revision;
};
static inline bool lab3_relay_allowed(uint16_t id, const uint8_t *d)
{
    return id==CAN_ID_CTRL || id==CAN_ID_BLINK || id==CAN_ID_HB_N1 ||
           id==CAN_ID_HB_N3 || (id==CAN_ID_FAULT && d[FAULT_ZONE]!=ZONE_STEERING);
}
static inline void lab3_relay_observe(struct lab3_relay_plan *p, uint16_t id,
        const uint8_t *d, unsigned bus, enum lab3_rx_result result, uint64_t now)
{
    if (bus>1 || !d || !lab3_payload_valid(id,d) || !lab3_relay_allowed(id,d)) return;
    int key=lab3_slot(id,d);
    if (key<0) return;
    struct lab3_relay_slot *s=&p->slots[key];
    if (result==LAB3_RX_NEW) {
        if (s->pending) ++p->superseded;
        s->valid=true;s->id=id;memcpy(s->data,d,8);
        s->seen=1u<<bus;s->pending=1u<<(bus^1u);
        s->expires=now+LAB3_RELAY_MAX_AGE_MS;++s->revision;++p->offered;
    } else if ((result==LAB3_RX_DUPLICATE || result==LAB3_RX_REPEAT) &&
               s->valid && s->id==id && !memcmp(s->data,d,8)) {
        s->seen|=1u<<bus;
        if (s->pending & (1u<<bus)) { s->pending&=~(1u<<bus);++p->suppressed; }
    }
}
static inline bool lab3_relay_candidate(struct lab3_relay_plan *p, unsigned key,
        unsigned bus, uint64_t now, struct lab3_relay_copy *copy)
{
    if (key>=9 || bus>1) return false;
    struct lab3_relay_slot *s=&p->slots[key];
    if (s->pending && now>=s->expires) { s->pending=0;++p->expired; }
    if (!(s->pending & (1u<<bus)) || (s->seen & (1u<<bus))) return false;
    copy->id=s->id;memcpy(copy->data,s->data,8);copy->revision=s->revision;
    return true;
}
static inline void lab3_relay_queued(struct lab3_relay_plan *p,unsigned key,
                                    unsigned bus,uint32_t revision)
{
    if (key<9 && bus<2 && p->slots[key].revision==revision)
        p->slots[key].pending&=~(1u<<bus);
}
static inline bool lab3_relay_pending(const struct lab3_relay_plan *p)
{
    for (unsigned i=0;i<9;i++) if (p->slots[i].pending) return true;
    return false;
}
#endif
