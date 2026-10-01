#ifndef LAB2_DUAL_SPEED_PID_H
#define LAB2_DUAL_SPEED_PID_H
#include <stdbool.h>
#include <stdint.h>
struct dual_pid {
    int64_t integral_micro;
    int64_t filtered_derivative_mrpm_s;
    int32_t previous_actual_mrpm;
    bool have_previous;
};
void dual_pid_reset(struct dual_pid *s);
/* Milli-RPM and microseconds in, duty permille out. Filtered D on measurement. */
int dual_pid_update(struct dual_pid *s,int32_t target,int32_t actual,uint32_t dt_us,uint16_t *duty);
#endif
