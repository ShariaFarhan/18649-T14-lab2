#!/usr/bin/env python3
"""Host regression tests; no Zephyr SDK or attached hardware required.

Runs the actual motor driver and the unchanged motor shell handler extracted
from main.c against GPIO/PWM mocks. This does not test Zephyr bindings, timer
timing, or physical L298N outputs. Run: python3 tests/test_motor_host.py
"""

from pathlib import Path
import os
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]

STUB = r"""
#pragma once
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
struct gpio_dt_spec { int pin; };
struct pwm_dt_spec { int channel; uint32_t period; };
#define DT_ALIAS(x) x
#define DT_NODE_HAS_STATUS(x,y) 1
#define PWM_DT_SPEC_GET(x) {.channel=x,.period=50000U}
#define GPIO_DT_SPEC_GET(x,y) {.pin=x}
#define motor_left_pwm 0
#define motor_right_pwm 1
#define motor_left_in1 0
#define motor_left_in2 1
#define motor_right_in1 2
#define motor_right_in2 3
#define GPIO_OUTPUT_INACTIVE 0
/* Keep format checking active without printing test commands. */
#define printk(...) ((void)snprintf(NULL, 0, __VA_ARGS__))
struct shell { int unused; };
#define shell_error(sh, ...) ((void)(sh), (void)snprintf(NULL, 0, __VA_ARGS__))
#define shell_print(sh, ...) ((void)(sh), (void)snprintf(NULL, 0, __VA_ARGS__))
#define speed_control_manual_set sys_motor_set

static int pins[4], calls, fail_at, pwm_unready, gpio_unready;
static bool pwm_always_fails;
static uint32_t pulses[2];
static int failures;
static int operation(void) {
    calls++;
    if (calls == fail_at) { failures++; return -EIO; }
    return 0;
}
static bool pwm_is_ready_dt(const struct pwm_dt_spec *p) {
    return p->channel != pwm_unready;
}
static bool gpio_is_ready_dt(const struct gpio_dt_spec *p) {
    return p->pin != gpio_unready;
}
static int gpio_pin_configure_dt(const struct gpio_dt_spec *p, int mode) {
    (void)mode;
    /* Initialization must disable both channels before changing direction. */
    assert(pulses[0] == 0 && pulses[1] == 0);
    int rc = operation();
    if (!rc) pins[p->pin] = 0;
    return rc;
}
static int gpio_pin_set_dt(const struct gpio_dt_spec *p, int value) {
    /* Direction changes must happen with this bridge disabled. */
    assert(pulses[p->pin / 2] == 0);
    int rc = operation();
    if (!rc) pins[p->pin] = value;
    return rc;
}
static int pwm_set_pulse_dt(const struct pwm_dt_spec *p, uint32_t value) {
    assert(value <= p->period);
    int rc = operation();
    if (pwm_always_fails) return -EIO;
    if (!rc) pulses[p->channel] = value;
    return rc;
}
static void reset(void) {
    memset(pins, 0, sizeof(pins));
    memset(pulses, 0, sizeof(pulses));
    calls = fail_at = failures = 0;
    pwm_unready = gpio_unready = -1;
    pwm_always_fails = false;
}
static int test_points_init(void) { return 0; }
static int test_points_pwm_written(void) { return 0; }
"""

TESTS = r"""
int main(void) {
    /* All accepted duties, both channels, and all L298N operating modes. */
    for (int motor = 0; motor < NUM_MOTORS; motor++) {
        for (uint32_t duty = 0; duty <= 100; duty++) {
            reset();
            assert(sys_motor_set(motor, duty, FORWARD) == 0);
            assert(pins[2*motor] == 0 && pins[2*motor+1] == 1);
            assert(pulses[motor] == duty * 500);
            assert(sys_motor_set(motor, duty, BACKWARD) == 0);
            assert(pins[2*motor] == 1 && pins[2*motor+1] == 0);
            assert(pulses[motor] == duty * 500);
            assert(sys_motor_set(motor, duty, STOP) == 0);
            assert(pins[2*motor] == pins[2*motor+1]);
            assert(pulses[motor] == 50000);
            assert(sys_motor_set(motor, duty, FREE) == 0);
            assert(pulses[motor] == 0);
            assert(pulses[1-motor] == 0);
        }
    }
    puts("PASS: both motors, all duties, forward/reverse/brake/coast");

    /* Inject failure at disable, IN1, IN2, and enable operations. */
    for (int motor = 0; motor < NUM_MOTORS; motor++) {
        for (int mode = FORWARD; mode <= STOP; mode++) {
            for (int step = 1; step <= 4; step++) {
                reset(); pulses[motor] = 25000; fail_at = step;
                assert(sys_motor_set(motor, 50, mode) == -EIO);
                assert(failures == 1 && pulses[motor] == 0);
                /* No nonzero PWM write follows a failed GPIO operation. */
                assert(calls == step + 1);
            }
        }
    }
    reset(); pwm_always_fails = true;
    assert(sys_motor_set(LEFT_MOTOR, 50, FORWARD) == -EIO);
    puts("PASS: GPIO/PWM failures propagate, with best-effort EN shutdown");

    reset(); pulses[0] = pulses[1] = 25000;
    assert(motor_init() == 0);
    assert(pulses[0] == 0 && pulses[1] == 0);
    /* Two initial PWM writes and four direction-pin configurations. */
    for (int step = 1; step <= 6; step++) {
        reset(); pulses[0] = pulses[1] = 25000; fail_at = step;
        assert(motor_init() == -EIO);
        assert(failures == 1 && pulses[0] == 0 && pulses[1] == 0);
    }
    for (int motor = 0; motor < NUM_MOTORS; motor++) {
        reset(); pwm_unready = motor; pulses[1-motor] = 25000;
        assert(motor_init() == -ENODEV);
        assert(pulses[1-motor] == 0);
    }
    for (int pin = 0; pin < 4; pin++) {
        reset(); gpio_unready = pin; pulses[0] = pulses[1] = 25000;
        assert(motor_init() == -ENODEV);
        assert(pulses[0] == 0 && pulses[1] == 0);
    }
    reset(); pwm_always_fails = true;
    assert(motor_init() == -EIO);
    puts("PASS: init/reinit failure reporting and partial-init cleanup");

    reset();
    assert(sys_motor_set((motor_mapping_t)-1, 50, FORWARD) == -EINVAL);
    assert(sys_motor_set(NUM_MOTORS, 50, FORWARD) == -EINVAL);
    assert(sys_motor_set(LEFT_MOTOR, 101, FORWARD) == -EINVAL);
    assert(sys_motor_set(LEFT_MOTOR, 50, (direction_t)99) == -EINVAL);
    assert(motor_debug_pins(NUM_MOTORS, 1, 0, 50) == -EINVAL);
    assert(motor_debug_pins(LEFT_MOTOR, 2, 0, 50) == -EINVAL);
    assert(motor_debug_pins(LEFT_MOTOR, 1, 0, 101) == -EINVAL);
    assert(calls == 0);
    assert(motor_debug_pins(RIGHT_MOTOR, 1, 0, 50) == 0);
    assert(pulses[1] == 25000);
    reset(); fail_at = 2;
    assert(motor_debug_pins(LEFT_MOTOR, 1, 0, 50) == -EIO);
    assert(pulses[0] == 0);
    puts("PASS: API validation and debug-path error handling");

    char *invalid[] = {"", "abc", "50junk", "-1", "-0", "+50", " 50",
        "50 ", "1.5", "101", "4294967346", "99999999999999999999999999999"};
    for (size_t i = 0; i < sizeof(invalid)/sizeof(invalid[0]); i++) {
        reset();
        char *args[] = {"set", "left", "forward", invalid[i]};
        assert(cmd_motor_set(NULL, 4, args) == -EINVAL);
        assert(calls == 0);
    }
    char *valid[] = {"0", "50", "100"};
    for (size_t i = 0; i < sizeof(valid)/sizeof(valid[0]); i++) {
        reset();
        char *args[] = {"set", "right", "forward", valid[i]};
        assert(cmd_motor_set(NULL, 4, args) == 0);
        assert(pulses[1] == (uint32_t)strtoul(valid[i], NULL, 10) * 500);
    }
    reset();
    char *stop[] = {"set", "left", "stop", "0"};
    char *coast[] = {"set", "left", "free", "100"};
    assert(cmd_motor_set(NULL, 4, stop) == 0 && pulses[0] == 50000);
    assert(cmd_motor_set(NULL, 4, coast) == 0 && pulses[0] == 0);
    reset();
    assert(cmd_motor_set(NULL, 3, stop) == -EINVAL);
    assert(cmd_motor_set(NULL, 5, stop) == -EINVAL);
    assert(calls == 0);
    puts("PASS: shell rejects malformed/overflowing input; valid commands work");
    return 0;
}
"""


def main():
    source = (ROOT / "stm32_app/src/main.c").read_text()
    start = source.index("static int cmd_motor_set(")
    end = source.index("static int cmd_motor_brake(", start)
    # Extract the actual handler, not a test-side rewrite of the parser.
    handler = source[start:end]
    with tempfile.TemporaryDirectory(prefix="lab2-motor-tests-") as tmp:
        temp = Path(tmp)
        (temp / "stub.h").write_text(STUB)
        (temp / "test_points.h").write_text(
            "int test_points_init(void);\n"
            "int test_points_pwm_written(void);\n"
        )
        for name in ("device.h", "devicetree.h", "drivers/gpio.h",
                     "drivers/pwm.h", "sys/printk.h"):
            header = temp / "zephyr" / name
            header.parent.mkdir(parents=True, exist_ok=True)
            header.write_text('#include "stub.h"\n')
        unit = temp / "test.c"
        unit.write_text('#include "logic/drive/motor_control.c"\n' + handler + TESTS)
        binary = temp / "test"
        subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                        "-Werror", "-fsanitize=address,undefined", "-g",
                        "-I", str(temp), "-I", str(ROOT), str(unit),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
