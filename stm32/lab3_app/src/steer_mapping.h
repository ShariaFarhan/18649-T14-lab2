#ifndef LAB3_STEER_MAPPING_H
#define LAB3_STEER_MAPPING_H
#include <stdbool.h>
#include <stdint.h>

/* CAN uses tenths of a percent; the unchanged Lab2 driver uses percent.
 * Validate before converting, so an invalid frame never becomes a full turn. */
static inline int16_t lab3_steer_percent(int16_t command, bool normal)
{
    if (!normal || command < -1000 || command > 1000) return 0;
    return command >= 0 ? (command + 5) / 10 : (command - 5) / 10;
}
#endif
