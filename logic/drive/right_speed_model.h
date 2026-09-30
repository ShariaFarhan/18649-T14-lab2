#ifndef LAB2_RIGHT_SPEED_MODEL_H
#define LAB2_RIGHT_SPEED_MODEL_H
#include <stdbool.h>
#include <stdint.h>
#define RIGHT_FULL_THROTTLE_MRPM 280000
#define RIGHT_START_DUTY_PERCENT 40U
#define RIGHT_START_DURATION_MS 150U
struct right_pi { int64_t integral_micro; };
int right_pedal_permille(int32_t raw);
int32_t right_target_mrpm(int throttle_permille);
uint16_t right_feedforward(int32_t target_mrpm);
int right_pi_update(struct right_pi *s,int32_t target,int32_t actual,uint32_t dt_us,uint16_t *duty);
#endif
