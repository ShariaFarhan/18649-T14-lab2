#ifndef FRONT_BLINKERS_H
#define FRONT_BLINKERS_H

#include <stdbool.h>
#include <stdint.h>

struct front_blinker_outputs {
    bool left;
    bool right;
};

/* Normal turn signals follow the received phase. A zone fault overrides it
 * with local hazards, toggling both lamps every 250 ms. */
void front_blinkers_resolve(uint8_t mode, bool phase_on, bool zone_fault,
                            uint32_t now_ms, struct front_blinker_outputs *out);

/* The hardware functions are called by front_zone after its safety checks. */
int front_blinkers_init(void);
int front_blinkers_apply(const struct front_blinker_outputs *out);

#endif
