#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "blinkers.h"
#include "servo.h"
#include "motor_control.h"
#include "brake.h"
#include "encoder.h"

//
//	SERVO
//
static int cmd_servo_init(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = servo_init();

	if (rc != 0) {
		shell_error(sh, "servo_init failed: %d", rc);
		return rc;
	}

	shell_print(sh, "Servo initialized at center (%u us)",
		    servo_get_last_pulse_us());
	(void)blinkers_update_steering(0);
	return 0;
}

static int cmd_servo_set(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	long steering;
	int rc;

	ARG_UNUSED(argc);

	errno = 0;
	steering = strtol(argv[1], &end, 10);
	if (errno != 0 || *argv[1] == '\0' || *end != '\0' ||
	    steering < SERVO_STEERING_MIN || steering > SERVO_STEERING_MAX) {
		shell_error(sh, "Expected an integer from -100 to 100");
		return -EINVAL;
	}

	rc = servo_set_steering((int16_t)steering);
	if (rc != 0) {
		shell_error(sh, "Set failed: %d (run 'servo init' first)", rc);
		return rc;
	}

	shell_print(sh, "Steering %ld -> %u us", steering,
		    servo_get_last_pulse_us());
	(void)blinkers_update_steering((int16_t)steering);
	return 0;
}

static int cmd_servo_center(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = servo_center();

	if (rc != 0) {
		shell_error(sh, "Center failed: %d (run 'servo init' first)", rc);
		return rc;
	}

	shell_print(sh, "Servo centered at %u us", servo_get_last_pulse_us());
	(void)blinkers_update_steering(0);
	return 0;
}

static int cmd_servo_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int rc = servo_disable();

	if (rc != 0) {
		shell_error(sh, "Disable failed: %d (run 'servo init' first)", rc);
		return rc;
	}

	shell_print(sh, "Servo PWM disabled");
	return 0;
}

static int cmd_servo_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t pulse_us = servo_get_last_pulse_us();

	if (pulse_us == 0U) {
		shell_print(sh, "Servo PWM is inactive or not initialized");
	} else {
		shell_print(sh, "Last commanded pulse: %u us", pulse_us);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	servo_commands,
	SHELL_CMD(init, NULL, "Initialize PWM and move to center", cmd_servo_init),
	SHELL_CMD_ARG(set, NULL, "Set steering from -100 to 100", cmd_servo_set, 2, 0),
	SHELL_CMD(center, NULL, "Move to calibrated center", cmd_servo_center),
	SHELL_CMD(off, NULL, "Disable the PWM output", cmd_servo_off),
	SHELL_CMD(status, NULL, "Show the last commanded pulse", cmd_servo_status),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(servo, &servo_commands, "Steering servo test commands", NULL);

//
//	BLINKERS
//

static const char *blinker_mode_name(enum blinker_mode mode)
{
	switch (mode) {
	case BLINKER_MODE_OFF:
		return "off";
	case BLINKER_MODE_LEFT:
		return "left";
	case BLINKER_MODE_RIGHT:
		return "right";
	case BLINKER_MODE_HAZARD:
		return "hazard";
	default:
		return "unknown";
	}
}

static int set_blinker_mode(const struct shell *sh, enum blinker_mode mode)
{
	int rc = blinkers_set_mode(mode);

	if (rc != 0) {
		shell_error(sh, "Blinker command failed: %d", rc);
		return rc;
	}

	shell_print(sh, "Blinker mode: %s", blinker_mode_name(mode));
	return 0;
}

static int cmd_blinker_init(const struct shell *sh, size_t argc, char **argv)
{
	int rc;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rc = blinkers_init();
	if (rc != 0) {
		shell_error(sh, "Blinker initialization failed: %d", rc);
		return rc;
	}

	shell_print(sh, "Blinkers initialized and off");
	return 0;
}

static int cmd_blinker_left(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return set_blinker_mode(sh, BLINKER_MODE_LEFT);
}

static int cmd_blinker_right(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return set_blinker_mode(sh, BLINKER_MODE_RIGHT);
}

static int cmd_blinker_hazard(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return set_blinker_mode(sh, BLINKER_MODE_HAZARD);
}

static int cmd_blinker_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	return set_blinker_mode(sh, BLINKER_MODE_OFF);
}

static int cmd_blinker_steering(const struct shell *sh, size_t argc,
				char **argv)
{
	char *end;
	long steering;
	int rc;

	ARG_UNUSED(argc);

	errno = 0;
	steering = strtol(argv[1], &end, 10);
	if (errno != 0 || *argv[1] == '\0' || *end != '\0' || steering < -100 ||
	    steering > 100) {
		shell_error(sh, "Expected an integer from -100 to 100");
		return -EINVAL;
	}

	rc = blinkers_update_steering((int16_t)steering);
	if (rc != 0) {
		shell_error(sh, "Steering update failed: %d", rc);
		return rc;
	}

	shell_print(sh, "Steering %ld; mode=%s; turn-crossed=%s", steering,
		    blinker_mode_name(blinkers_get_mode()),
		    blinkers_turn_crossed() ? "yes" : "no");
	return 0;
}

static int cmd_blinker_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "mode=%s steering=%d turn-crossed=%s",
		    blinker_mode_name(blinkers_get_mode()),
		    blinkers_get_last_steering(),
		    blinkers_turn_crossed() ? "yes" : "no");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	blinker_commands,
	SHELL_CMD(init, NULL, "Initialize all blinker GPIOs off", cmd_blinker_init),
	SHELL_CMD(left, NULL, "Select the left turn signal", cmd_blinker_left),
	SHELL_CMD(right, NULL, "Select the right turn signal", cmd_blinker_right),
	SHELL_CMD(hazard, NULL, "Flash all four outputs at 2 Hz",
		  cmd_blinker_hazard),
	SHELL_CMD(off, NULL, "Turn every blinker off", cmd_blinker_off),
	SHELL_CMD_ARG(steering, NULL, "Feed steering position from -100 to 100",
		      cmd_blinker_steering, 2, 0),
	SHELL_CMD(status, NULL, "Show the blinker state", cmd_blinker_status),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(blinker, &blinker_commands, "Blinker test commands", NULL);

//
//	MOTOR CONTROL
//
static int cmd_motor_init(const struct shell *sh, size_t argc, char **argv)
{
    int rc = motor_init();
    if (rc == 0) {
        shell_print(sh, "Motor driver initialized successfully.");
    } else {
        shell_error(sh, "Motor driver initialization failed: %d", rc);
    }
    return rc;
}

static int cmd_motor_set(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 4) {
        shell_error(sh, "Usage: motor set <left|right> <forward|backward|free|stop> <duty_cycle: 0-100>");
        return -EINVAL;
    }

    /* 1. Parse Motor Channel */
    motor_mapping_t motor;
    if (strcmp(argv[1], "left") == 0) {
        motor = LEFT_MOTOR;
    } else if (strcmp(argv[1], "right") == 0) {
        motor = RIGHT_MOTOR;
    } else {
        shell_error(sh, "Invalid motor: %s (use 'left' or 'right')", argv[1]);
        return -EINVAL;
    }

    /* 2. Parse Direction */
    direction_t dir;
    if (strcmp(argv[2], "forward") == 0) {
        dir = FORWARD;
    } else if (strcmp(argv[2], "backward") == 0) {
        dir = BACKWARD;
    } else if (strcmp(argv[2], "free") == 0) {
        dir = FREE;
    } else if (strcmp(argv[2], "stop") == 0) {
        dir = STOP;
    } else {
        shell_error(sh, "Invalid direction: %s (use 'forward', 'backward', 'free', or 'stop')", argv[2]);
        return -EINVAL;
    }

    /* 3. Parse Duty Cycle */
    uint32_t duty_cycle = (uint32_t)strtoul(argv[3], NULL, 10);
    if (duty_cycle > MAX_DUTY_CYCLE) {
        shell_error(sh, "Invalid duty cycle: %u (must be 0 to %d)", duty_cycle, MAX_DUTY_CYCLE);
        return -EINVAL;
    }

    /* Execute Driver Call */
    int rc = sys_motor_set(motor, duty_cycle, dir);
    if (rc == 0) {
        shell_print(sh, "Motor %s set to %s; duty=%u%% (ignored for stop).", argv[1], argv[2], duty_cycle);
    } else {
        shell_error(sh, "Failed to set motor parameters (code %d).", rc);
    }

    return rc;
}

static int cmd_motor_brake(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    int rc = motor_brake_all();
    if (rc != 0) {
        shell_error(sh, "Braking failed on at least one motor: %d", rc);
    } else {
        shell_print(sh, "Both motors: dynamic braking");
    }
    return rc;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_motor,
    SHELL_CMD(brake, NULL, "Dynamically brake both motors", cmd_motor_brake),
    SHELL_CMD(init, NULL, "Initialize motor driver pins & PWM", cmd_motor_init),
    SHELL_CMD(set, NULL, "Set motor: motor set <left|right> <forward|backward|free|stop> <duty_cycle>", cmd_motor_set),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(motor, &sub_motor, "Motor driver control commands", NULL);

//
//	MAIN
//

int main(void)
{
	int blinker_rc = blinkers_init();
	int motor_rc = motor_init();
	int encoder_rc = encoders_init();
	printk("Encoder init: %d; use encoder status / zero / sign\n", encoder_rc);

	printk("Motor test shell ready. Type 'motor' for commands.\n");
	if (motor_rc != 0) {
			printk("Motor initialization failed: %d\n", motor_rc);
	}
	printk("Servo test shell ready. Type 'servo' for commands.\n");
	printk("PWM remains off until 'servo init'.\n");
	if (blinker_rc == 0) {
		printk("Blinker test shell ready. Type 'blinker' for commands.\n");
	} else {
		printk("Blinker initialization failed: %d\n", blinker_rc);
	}
	return 0;
}

/*	MOTOR CONTROL DEBUGGING */
// int main(void)
// {
// 	motor_init();
// 	blinkers_init();

// 	/* 1. Drive Forward at 50% Speed */
// 	motor_set_throttle(20, MOTOR_DIR_FORWARD);

// 	k_msleep(2000);
// 	motor_set_throttle(0, MOTOR_DIR_FORWARD);
// 	// k_msleep(2000);
// 	motor_set_throttle(20, MOTOR_DIR_BACKWARD);

// 	/* 2. Hit Brake Pedal (Priority Brake engages immediately) */
// 	// motor_set_brake(true);

// 	// k_msleep(1000);

// 	/* 3. Release Brake Pedal (Returns output back to 50% Forward) */
// 	// motor_set_brake(false);

// 	return 0;
// }

// int main(void)
// {
//     motor_init();

//     while (1) {
//         /* Drive Forward at 20% speed for 2 seconds */
//         sys_motor_set(LEFT_MOTOR, 70, FORWARD);
//         sys_motor_set(RIGHT_MOTOR, 70, FORWARD);
//         k_msleep(2000);

//         /* FREE Coast (00) for 2 seconds - Stops heating! */
//         sys_motor_set(LEFT_MOTOR, 0, FREE);
//         sys_motor_set(RIGHT_MOTOR, 0, FREE);
//         k_msleep(2000);

//         /* Drive Backward at 20% speed for 2 seconds */
//         sys_motor_set(LEFT_MOTOR, 70, BACKWARD);
//         sys_motor_set(RIGHT_MOTOR, 70, BACKWARD);
//         k_msleep(2000);

//         /* STOP Brake for 1 second */
//         sys_motor_set(LEFT_MOTOR, 0, STOP);
//         sys_motor_set(RIGHT_MOTOR, 0, STOP);
//         k_msleep(1000);
//     }

//     return 0;
// }

// int main(void)
// {
//     printk("motor_init rc=%d\n", motor_init());

//     for (int m = 0; m < NUM_MOTORS; m++) {
//         printk("--- motor %d: IN1 only ---\n", m);
//         motor_debug_pins(m, 1, 0, 100);
//         k_msleep(4000);
//         printk("--- motor %d: IN2 only ---\n", m);
//         motor_debug_pins(m, 0, 1, 100);
//         k_msleep(4000);
//         motor_debug_pins(m, 0, 0, 0);
//     }
//     return 0;
// }

// int main(void)
// {
//     printk("motor_init rc=%d\n", motor_init());
//     sys_motor_set(RIGHT_MOTOR, 100, FREE);  /* enable held high, IN pins low */
//     return 0;
// }