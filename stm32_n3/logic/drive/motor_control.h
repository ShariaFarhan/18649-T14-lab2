#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#define MAX_DUTY_CYCLE 100

typedef enum {
    LEFT_MOTOR = 0,
    RIGHT_MOTOR = 1,
    NUM_MOTORS = 2
} motor_mapping_t;

typedef enum {
    FREE = 0,    /* Coast / High Impedance (EN low, duty ignored) */
    FORWARD,     /* Direction 1 */
    BACKWARD,    /* Direction 2 */
    STOP         /* Dynamic braking (equal IN pins, EN high) */
} direction_t;

struct motor_attr {
    struct pwm_dt_spec pwm;
    struct gpio_dt_spec in1;
    struct gpio_dt_spec in2;
};

int motor_init(void);

/**
 * @brief Set speed and direction for a specific motor channel.
 *
 * @param motor LEFT_MOTOR or RIGHT_MOTOR
 * @param duty_cycle Speed percentage (0 - 100); ignored for STOP
 * @param direction FREE, FORWARD, BACKWARD, or STOP
 * @return 0 on success, negative errno on error
 */
int sys_motor_set(motor_mapping_t motor, uint32_t duty_cycle,
                  direction_t direction);

/* Brake only the selected motor; requires motor_init(). */
int motor_brake(motor_mapping_t motor);

int motor_debug_pins(motor_mapping_t motor, int in1, int in2, uint32_t duty);

#endif /* MOTOR_DRIVER_H_ */
