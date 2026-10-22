#ifndef BRAKE_H_
#define BRAKE_H_

/* Request braking on both motors, even if the first call fails.
 * Returns the first error, or zero when both succeed. Not simultaneous. */
int motor_brake_all(void);
#endif
