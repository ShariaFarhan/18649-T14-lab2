#ifndef LAB2_SPEED_PI_H_
#define LAB2_SPEED_PI_H_
#include <stdint.h>
struct speed_pi { int64_t integral_micro; };
void speed_pi_reset(struct speed_pi *s);
/* Input milli-RPM, dt microseconds. Output duty in permille (0..1000).
 * Initial gains/feedforward are proposals, not hardware tuned. */
int speed_pi_update(struct speed_pi *s, int32_t target, int32_t actual,
                    uint32_t dt_us, uint16_t *duty);
#endif
