#ifndef LAB2_TEST_POINTS_H
#define LAB2_TEST_POINTS_H
#include <stdint.h>
struct test_point_counts {
    uint32_t command_rx, pwm_set, errors;
};
int test_points_init(void);
int test_points_command_received(void);
/* Called immediately after the final successful LEFT PWM timer write. */
int test_points_pwm_written(void);
struct test_point_counts test_points_counts(void);
#endif
