#include "encoder_math.h"
#include <limits.h>
#include <stddef.h>

bool encoder_count_delta(uint32_t now, uint32_t previous, uint64_t modulus, int32_t *delta)
{
    if (!delta || modulus < 2 || modulus > (1ULL << 32) ||
        now >= modulus || previous >= modulus) return false;
    int64_t d = (int64_t)now - previous;
    /* At exactly half a cycle direction cannot be determined. */
    if (d * 2 == (int64_t)modulus || d * 2 == -(int64_t)modulus) return false;
    if (d * 2 > (int64_t)modulus) d -= (int64_t)modulus;
    if (d * 2 < -(int64_t)modulus) d += (int64_t)modulus;
    if (d > INT32_MAX || d < INT32_MIN) return false;
    *delta = (int32_t)d;
    return true;
}

bool encoder_rpm_milli(int32_t delta, uint32_t cpr, uint64_t elapsed_us, int32_t *rpm)
{
    const int64_t factor = 60000000000LL; /* 60 sec/min * us/sec * 1000 */
    if (!rpm || !cpr || !elapsed_us || elapsed_us > INT64_MAX / (uint64_t)cpr)
        return false;
    if ((int64_t)delta > INT64_MAX / factor ||
        (int64_t)delta < INT64_MIN / factor) return false;
    int64_t value = (int64_t)delta * factor / ((int64_t)cpr * (int64_t)elapsed_us);
    if (value > INT32_MAX || value < INT32_MIN) return false;
    *rpm = (int32_t)value;
    return true;
}
