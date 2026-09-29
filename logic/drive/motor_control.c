#include "motor_control.h"

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

/* Devicetree Aliases matching your setup */
#define MOTOR_LEFT_PWM   DT_ALIAS(motor_left_pwm)
#define MOTOR_RIGHT_PWM  DT_ALIAS(motor_right_pwm)
#define MOTOR_LEFT_IN1   DT_ALIAS(motor_left_in1)
#define MOTOR_LEFT_IN2   DT_ALIAS(motor_left_in2)
#define MOTOR_RIGHT_IN1  DT_ALIAS(motor_right_in1)
#define MOTOR_RIGHT_IN2  DT_ALIAS(motor_right_in2)

#if !DT_NODE_HAS_STATUS(MOTOR_LEFT_PWM, okay) || \
    !DT_NODE_HAS_STATUS(MOTOR_RIGHT_PWM, okay) || \
    !DT_NODE_HAS_STATUS(MOTOR_LEFT_IN1, okay) || \
    !DT_NODE_HAS_STATUS(MOTOR_LEFT_IN2, okay) || \
    !DT_NODE_HAS_STATUS(MOTOR_RIGHT_IN1, okay) || \
    !DT_NODE_HAS_STATUS(MOTOR_RIGHT_IN2, okay)
#error "Devicetree aliases missing for left or right motor!"
#endif

/* Array of motor structures mapping directly to motor_mapping_t indices */
static const struct motor_attr motors[NUM_MOTORS] = {
    [LEFT_MOTOR] = {
        .pwm = PWM_DT_SPEC_GET(MOTOR_LEFT_PWM),
        .in1 = GPIO_DT_SPEC_GET(MOTOR_LEFT_IN1, gpios),
        .in2 = GPIO_DT_SPEC_GET(MOTOR_LEFT_IN2, gpios),
    },
    [RIGHT_MOTOR] = {
        .pwm = PWM_DT_SPEC_GET(MOTOR_RIGHT_PWM),
        .in1 = GPIO_DT_SPEC_GET(MOTOR_RIGHT_IN1, gpios),
        .in2 = GPIO_DT_SPEC_GET(MOTOR_RIGHT_IN2, gpios),
    }
};

/* Best-effort shutdown of both channels, including after partial init failure. */
static int disable_all_motors(void)
{
    int first_error = 0;

    for (size_t i = 0; i < NUM_MOTORS; i++) {
        int rc = pwm_is_ready_dt(&motors[i].pwm)
                     ? pwm_set_pulse_dt(&motors[i].pwm, 0U)
                     : -ENODEV;

        if (first_error == 0 && rc != 0) {
            first_error = rc;
        }
    }
    return first_error;
}

int motor_init(void)
{
    /* Disable EN before configuring direction pins, also on reinitialization. */
    int rc = disable_all_motors();

    if (rc != 0) {
        goto fail;
    }

    for (size_t i = 0; i < NUM_MOTORS; i++) {
        if (!gpio_is_ready_dt(&motors[i].in1) ||
            !gpio_is_ready_dt(&motors[i].in2)) {
            rc = -ENODEV;
            goto fail;
        }

        rc = gpio_pin_configure_dt(&motors[i].in1, GPIO_OUTPUT_INACTIVE);
        if (rc != 0) {
            goto fail;
        }
        rc = gpio_pin_configure_dt(&motors[i].in2, GPIO_OUTPUT_INACTIVE);
        if (rc != 0) {
            goto fail;
        }
    }
    return 0;

fail:
    /* Preserve the original error even if a shutdown attempt also fails. */
    (void)disable_all_motors();
    return rc;
}

static int motor_apply(const struct motor_attr *m, int in1, int in2,
                       uint32_t pulse_ns)
{
    /* Do not change direction while EN is active. */
    int rc = pwm_set_pulse_dt(&m->pwm, 0U);

    if (rc != 0) {
        goto fail;
    }
    rc = gpio_pin_set_dt(&m->in1, in1);
    if (rc != 0) {
        goto fail;
    }
    rc = gpio_pin_set_dt(&m->in2, in2);
    if (rc != 0) {
        goto fail;
    }
    if (pulse_ns != 0U) {
        rc = pwm_set_pulse_dt(&m->pwm, pulse_ns);
        if (rc != 0) {
            goto fail;
        }
    }
    return 0;

fail:
    /* A failed driver cannot guarantee shutdown; still attempt it and report. */
    (void)pwm_set_pulse_dt(&m->pwm, 0U);
    return rc;
}

int sys_motor_set(motor_mapping_t motor, uint32_t duty_cycle, direction_t direction)
{
    if ((unsigned int)motor >= NUM_MOTORS || duty_cycle > MAX_DUTY_CYCLE) {
        return -EINVAL;
    }

    const struct motor_attr *m = &motors[motor];
    uint32_t pulse_ns = (uint32_t)(((uint64_t)duty_cycle * m->pwm.period) /
                                   MAX_DUTY_CYCLE);

    switch (direction) {
    case FREE:
        /* L298N coast: EN must stay low, regardless of requested duty. */
        return motor_apply(m, 0, 0, 0U);
    case FORWARD:
        return motor_apply(m, 1, 0, pulse_ns);
    case BACKWARD:
        return motor_apply(m, 0, 1, pulse_ns);
    case STOP:
        /* L298N dynamic brake: equal IN pins and EN continuously high. */
        return motor_apply(m, 1, 1, m->pwm.period);
    default:
        return -EINVAL;
    }
}

int motor_debug_pins(motor_mapping_t motor, int in1, int in2, uint32_t duty)
{
    if ((unsigned int)motor >= NUM_MOTORS || duty > MAX_DUTY_CYCLE ||
        (in1 != 0 && in1 != 1) || (in2 != 0 && in2 != 1)) {
        return -EINVAL;
    }

    const struct motor_attr *m = &motors[motor];
    uint32_t pulse_ns = (uint32_t)(((uint64_t)duty * m->pwm.period) /
                                   MAX_DUTY_CYCLE);
    int rc = motor_apply(m, in1, in2, pulse_ns);

    printk("motor %d: IN=%d%d duty=%u rc=%d\n", motor, in1, in2, duty, rc);
    return rc;
}
