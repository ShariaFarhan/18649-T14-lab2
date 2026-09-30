#ifndef LAB2_BRAKE_H_
#define LAB2_BRAKE_H_
#include "motor_control.h"

/* Request braking on both motors, even if the first call fails.
 * Returns the first error, or zero when both succeed. Not simultaneous. */
int motor_brake_all(void);
#endif
