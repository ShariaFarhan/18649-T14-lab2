#ifndef BLINKERS_BACK_H_
#define BLINKERS_BACK_H_

#include <stdbool.h>
#include <stdint.h>

struct rear_blinker_outputs {
    bool left;
    bool right;
};

/* Normal turn signals follow the received phase. A zone fault overrides it
 * with local hazards, toggling both lamps every 250 ms. */
void rear_blinkers_resolve(uint8_t mode, bool phase_on, bool zone_fault,
                           uint32_t now_ms, struct rear_blinker_outputs *out);

/* The hardware functions are called by back_zone after its safety checks. */
int rear_blinkers_init(void);
int rear_blinkers_apply(const struct rear_blinker_outputs *out);

#endif /* BLINKERS_BACK_H_ */