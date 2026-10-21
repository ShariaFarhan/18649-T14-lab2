#include "wheel_mapping.h"
#include <errno.h>
#include <stddef.h>
int wheel_steering_percent(int32_t raw, int16_t *percent)
{
    if (!percent || raw < WHEEL_RAW_LEFT || raw > WHEEL_RAW_RIGHT) return -EINVAL;
    *percent = raw < WHEEL_RAW_CENTER ?
        -(int16_t)(((int64_t)WHEEL_RAW_CENTER-raw)*100/(WHEEL_RAW_CENTER-WHEEL_RAW_LEFT)) :
        (int16_t)(((int64_t)raw-WHEEL_RAW_CENTER)*100/(WHEEL_RAW_RIGHT-WHEEL_RAW_CENTER));
    return 0;
}
