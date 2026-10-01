#ifndef LAB2_LINK_WATCHDOG_H
#define LAB2_LINK_WATCHDOG_H
#include <stdbool.h>
#include <stdint.h>
/* Passive deadline tracking. Polling a healthy link never changes actuator state. */
struct link_watchdog { uint64_t last_valid_ms; uint32_t timeout_ms; bool seen,reported; };
static inline void link_watchdog_init(struct link_watchdog *w,uint32_t timeout_ms)
{ *w=(struct link_watchdog){.timeout_ms=timeout_ms}; }
static inline void link_watchdog_feed(struct link_watchdog *w,uint64_t now)
{ w->last_valid_ms=now;w->seen=true;w->reported=false; }
static inline bool link_watchdog_poll(struct link_watchdog *w,uint64_t now)
{
    if(!w->seen || w->reported || now-w->last_valid_ms<w->timeout_ms)return false;
    w->reported=true;return true; /* One transition for a confirmed missed deadline. */
}
#endif
