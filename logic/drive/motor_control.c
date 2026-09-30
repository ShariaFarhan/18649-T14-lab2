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

int motor_init(void)
{
    int rc;

    for (size_t i = 0; i < NUM_MOTORS; i++) {
        /* Verify hardware readiness */
        if (!pwm_is_ready_dt(&motors[i].pwm) ||
            !gpio_is_ready_dt(&motors[i].in1) ||
            !gpio_is_ready_dt(&motors[i].in2)) {
            printk("Error: Motor hardware channel %zu not ready\n", i);
            return -ENODEV;
        }

        /* Configure IN1 and IN2 pins as active outputs */
        rc = gpio_pin_configure_dt(&motors[i].in1, GPIO_OUTPUT_INACTIVE);
        if (rc != 0) return rc;

        rc = gpio_pin_configure_dt(&motors[i].in2, GPIO_OUTPUT_INACTIVE);
        if (rc != 0) return rc;

        /* Initial state: Idle / Coast */
        sys_motor_set((motor_mapping_t)i, 0, FREE);
    }

    return 0;
}

/* Dynamic braking of one L298N bridge. Requires motor_init() first. */
int motor_brake(motor_mapping_t motor)
{
    if ((unsigned int)motor >= NUM_MOTORS) {
        return -EINVAL;
    }

    const struct motor_attr *m = &motors[motor];
    /* Disable the bridge while setting its two direction inputs. */
    int rc = pwm_set_pulse_dt(&m->pwm, 0U);
    if (rc != 0) goto fail;
    /* 11 - Set both direction pins HIGH (Dynamic Brake)[cite: 6] */
    rc = gpio_pin_set_dt(&m->in1, 1); // Replaces gpio_set[cite: 6]
    if (rc != 0) goto fail;
    rc = gpio_pin_set_dt(&m->in2, 1); // Replaces gpio_set[cite: 6]
    if (rc != 0) goto fail;

    /* Pulse == period means EN stays HIGH, not a forward-drive command. */
    rc = pwm_set_pulse_dt(&m->pwm, m->pwm.period);
    if (rc == 0) return 0;

fail:
    /* Best effort: disable a bridge whose control update failed.
     * Return the error; physical braking is not guaranteed on failure. */
    (void)pwm_set_pulse_dt(&m->pwm, 0U);
    return rc;
}

int sys_motor_set(motor_mapping_t motor, uint32_t duty_cycle, direction_t direction)
{
    /* STOP ignores duty_cycle and returns before normal PWM handling. */
    if (direction == STOP) {
        return motor_brake(motor);
    }

    if (motor >= NUM_MOTORS) return -1;
    if (duty_cycle > MAX_DUTY_CYCLE) return -1;

    const struct motor_attr *m = &motors[motor];
    uint32_t pulse_ticks = (duty_cycle * m->pwm.period) / MAX_DUTY_CYCLE;

    if (direction == FREE) {
        /* 00 - Set both direction pins to LOW (Coast) */
        gpio_pin_set_dt(&m->in1, 0); // Replaces gpio_clr
        gpio_pin_set_dt(&m->in2, 0); // Replaces gpio_clr
    }
    else if (direction == FORWARD) {
        /* Forward state (with left/right motor inversion logic preserved from your file)[cite: 6] */
        gpio_pin_set_dt(&m->in1, 1); //[cite: 6]
        gpio_pin_set_dt(&m->in2, 0);
        // if (motor == LEFT_MOTOR) {
        //     gpio_pin_set_dt(&m->in1, 0); //[cite: 6]
        //     gpio_pin_set_dt(&m->in2, 1); //[cite: 6]
        // } else {
        //     gpio_pin_set_dt(&m->in1, 1); //[cite: 6]
        //     gpio_pin_set_dt(&m->in2, 0); //[cite: 6]
        // }
    }
    else if (direction == BACKWARD) {
        /* Backward state[cite: 6] */
        gpio_pin_set_dt(&m->in1, 0); //[cite: 6]
        gpio_pin_set_dt(&m->in2, 1); //[cite: 6]
        // if (motor == LEFT_MOTOR) {
        //     gpio_pin_set_dt(&m->in1, 1); //[cite: 6]
        //     gpio_pin_set_dt(&m->in2, 0); //[cite: 6]
        // } else {
        //     gpio_pin_set_dt(&m->in1, 0); //[cite: 6]
        //     gpio_pin_set_dt(&m->in2, 1); //[cite: 6]
        // }
    }
    else {
        return -1;
    }

    /* Set pulse width on the PWM channel */
    int rc = pwm_set_pulse_dt(&m->pwm, pulse_ticks);
    if (rc != 0) {
        return -1;
    }

    return 0;
}

int motor_debug_pins(motor_mapping_t motor, int in1, int in2, uint32_t duty)
{
    const struct motor_attr *m = &motors[motor];
    int rc;

    rc = gpio_pin_set_dt(&m->in1, in1);
    printk("m%d in1=%d rc=%d\n", motor, in1, rc);
    rc = gpio_pin_set_dt(&m->in2, in2);
    printk("m%d in2=%d rc=%d\n", motor, in2, rc);
    rc = pwm_set_pulse_dt(&m->pwm, (duty * m->pwm.period) / MAX_DUTY_CYCLE);
    printk("m%d pwm rc=%d\n", motor, rc);
    return rc;
}