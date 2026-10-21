#ifndef LAB2_WHEEL_MAPPING_H
#define LAB2_WHEEL_MAPPING_H
#include <stdint.h>
#define WHEEL_RAW_LEFT 0
#define WHEEL_RAW_CENTER 33151 /* Observed neutral; repeat calibration on the installed wheel. */
#define WHEEL_RAW_RIGHT 65535
int wheel_steering_percent(int32_t raw, int16_t *percent);
#endif
