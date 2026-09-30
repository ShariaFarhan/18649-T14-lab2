#ifndef LAB2_ENCODER_MATH_H_
#define LAB2_ENCODER_MATH_H_
#include <stdbool.h>
#include <stdint.h>
/* Modulus is ARR+1, not necessarily 65536/2^32. False means ambiguous/invalid. */
bool encoder_count_delta(uint32_t now, uint32_t previous, uint64_t modulus, int32_t *delta);
bool encoder_rpm_milli(int32_t delta, uint32_t counts_per_rev, uint64_t elapsed_us, int32_t *rpm);
#endif
