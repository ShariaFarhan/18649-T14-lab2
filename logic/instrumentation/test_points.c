#include "test_points.h"
#include <errno.h>
#include <stdbool.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

static const struct gpio_dt_spec command_rx =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), cmd_rx_test_gpios);
static const struct gpio_dt_spec pwm_set =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pwm_set_test_gpios);
static bool initialized;
static atomic_t rx_count, pwm_count, error_count;

int test_points_init(void)
{
    if (initialized) return 0;
    if (!gpio_is_ready_dt(&command_rx) || !gpio_is_ready_dt(&pwm_set)) return -ENODEV;
    int rc = gpio_pin_configure_dt(&command_rx, GPIO_OUTPUT_INACTIVE);
    if (!rc) rc = gpio_pin_configure_dt(&pwm_set, GPIO_OUTPUT_INACTIVE);
    if (!rc) initialized = true;
    return rc;
}
static int mark(const struct gpio_dt_spec *pin, atomic_t *counter)
{
    int rc = initialized ? gpio_pin_toggle_dt(pin) : -EACCES;
    if (rc) atomic_inc(&error_count);
    else atomic_inc(counter);
    return rc;
}
int test_points_command_received(void) { return mark(&command_rx, &rx_count); }
int test_points_pwm_written(void) { return mark(&pwm_set, &pwm_count); }
struct test_point_counts test_points_counts(void)
{
    return (struct test_point_counts){
        .command_rx = (uint32_t)atomic_get(&rx_count),
        .pwm_set = (uint32_t)atomic_get(&pwm_count),
        .errors = (uint32_t)atomic_get(&error_count)
    };
}
