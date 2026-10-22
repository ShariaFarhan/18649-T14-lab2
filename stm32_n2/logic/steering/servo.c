#include "servo.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>

#define STEERING_SERVO_NODE DT_ALIAS(steering_servo)

#if !DT_NODE_EXISTS(STEERING_SERVO_NODE)
#error "Devicetree alias 'steering-servo' is required"
#endif

#if !DT_NODE_HAS_PROP(STEERING_SERVO_NODE, pwms)
#error "The 'steering-servo' Devicetree node must contain a pwms property"
#endif

BUILD_ASSERT(SERVO_PERIOD_US > SERVO_PULSE_LEFT_US, "left servo pulse must be shorter than the period");
BUILD_ASSERT(SERVO_PERIOD_US > SERVO_PULSE_CENTER_US, "center servo pulse must be shorter than the period");
BUILD_ASSERT(SERVO_PERIOD_US > SERVO_PULSE_RIGHT_US, "right servo pulse must be shorter than the period");
BUILD_ASSERT(SERVO_PULSE_LEFT_US > 0U && SERVO_PULSE_CENTER_US > 0U && SERVO_PULSE_RIGHT_US > 0U, "servo pulse widths must be nonzero");

static const struct pwm_dt_spec servo_pwm = PWM_DT_SPEC_GET(STEERING_SERVO_NODE);

static bool servo_initialized;
static uint32_t last_pulse_us;

static int16_t clamp_steering(int16_t steering)
{
    if (steering < SERVO_STEERING_MIN) {
        return SERVO_STEERING_MIN;
    }
    if (steering > SERVO_STEERING_MAX) {
        return SERVO_STEERING_MAX;
    }
    return steering;
}

/*
 * Piecewise interpolation to keep measured center exact even when the mechanical L/R ranges are asymmetric.
 */
static uint32_t steering_to_pulse_us(int16_t steering)
{
    const int32_t position = clamp_steering(steering);
    int64_t pulse;

    if (position <= SERVO_STEERING_CENTER) {
        const int64_t travel = position - SERVO_STEERING_MIN;
        const int64_t span = (int64_t)SERVO_PULSE_CENTER_US - (int64_t)SERVO_PULSE_LEFT_US;

        pulse = (int64_t)SERVO_PULSE_LEFT_US + (span * travel) / (SERVO_STEERING_CENTER - SERVO_STEERING_MIN);

    } else {
        const int64_t travel = position - SERVO_STEERING_CENTER;
        const int64_t span = (int64_t)SERVO_PULSE_RIGHT_US - (int64_t)SERVO_PULSE_CENTER_US;

        pulse = (int64_t)SERVO_PULSE_CENTER_US + (span * travel) / (SERVO_STEERING_MAX - SERVO_STEERING_CENTER);
    }

    return (uint32_t)pulse;
}

static int set_pulse_us(uint32_t pulse_us)
{
    int rc;

    if (!servo_initialized) {
        return -EACCES;
    }

    rc = pwm_set_dt(&servo_pwm, PWM_USEC(SERVO_PERIOD_US), PWM_USEC(pulse_us));
    if (rc == 0) {
        last_pulse_us = pulse_us;
    }

    return rc;
}

int servo_prepare(void)
{
    if (!pwm_is_ready_dt(&servo_pwm)) return -ENODEV;
    servo_initialized = true;
    int rc = servo_disable();
    if (rc) servo_initialized = false;
    return rc;
}

int servo_init(void)
{
    int rc;

    if (!pwm_is_ready_dt(&servo_pwm)) {
        return -ENODEV;
    }

    servo_initialized = true;
    rc = set_pulse_us(SERVO_PULSE_CENTER_US);
    if (rc != 0) {
        servo_initialized = false;
    }

    return rc;
}

int servo_set_steering(int16_t steering_normalized)
{
    return set_pulse_us(steering_to_pulse_us(steering_normalized));
}

int servo_center(void)
{
    return set_pulse_us(SERVO_PULSE_CENTER_US);
}

int servo_disable(void)
{
    int rc;

    if (!servo_initialized) {
        return -EACCES;
    }

    rc = pwm_set_dt(&servo_pwm, PWM_USEC(SERVO_PERIOD_US), 0U);
    if (rc == 0) {
        last_pulse_us = 0U;
    }

    return rc;
}

uint32_t servo_get_last_pulse_us(void)
{
    return last_pulse_us;
}
