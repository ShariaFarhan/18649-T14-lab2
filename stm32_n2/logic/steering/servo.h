#ifndef LAB2_SERVO_H_
#define LAB2_SERVO_H_

#include <stdint.h>

/*
 * Normalized steering commands before reaching this module
 *   -1000 = full left, 0 = center, +1000 = full right.
 */
#define SERVO_STEERING_MIN       (-1000)
#define SERVO_STEERING_CENTER    0
#define SERVO_STEERING_MAX       1000

#ifndef SERVO_PERIOD_US
#define SERVO_PERIOD_US          20000U
#endif

#ifndef SERVO_PULSE_LEFT_US
#define SERVO_PULSE_LEFT_US      1000U
#endif

#ifndef SERVO_PULSE_CENTER_US
#define SERVO_PULSE_CENTER_US    1500U
#endif

#ifndef SERVO_PULSE_RIGHT_US
#define SERVO_PULSE_RIGHT_US     2000U
#endif

/* Check/initialize hardware with PWM disabled (safe for startup/retry). */
int servo_prepare(void);

/* Initialize the PWM device and move the steering servo to center. */
int servo_init(void);

/*
 * Set a normalized steering command. Values outside [-1000, 1000] are clamped
 * before being mapped to the calibrated pulse-width endpoints.
 */
int servo_set_steering(int16_t steering_normalized);

/* Move to the configured center position. */
int servo_center(void);

/* Stop generating PWM. The servo will no longer actively hold position. */
int servo_disable(void);

/* Last successfully commanded pulse width, or 0 after servo_disable(). */
uint32_t servo_get_last_pulse_us(void);

#endif /* LAB2_SERVO_H_ */
