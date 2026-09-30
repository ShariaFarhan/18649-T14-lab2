#!/usr/bin/env python3
"""Run actual motor/brake C sources against GPIO/PWM mocks. No hardware motion."""
import os,subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
STUB = '\n#pragma once\n#include <assert.h>\n#include <errno.h>\n#include <stdbool.h>\n#include <stddef.h>\n#include <stdint.h>\n#include <stdio.h>\n#include <stdlib.h>\n#include <string.h>\nstruct gpio_dt_spec { int pin; };\nstruct pwm_dt_spec { int channel; uint32_t period; };\n#define DT_ALIAS(x) x\n#define DT_NODE_HAS_STATUS(x,y) 1\n#define PWM_DT_SPEC_GET(x) {.channel=x,.period=50000U}\n#define GPIO_DT_SPEC_GET(x,y) {.pin=x}\n#define motor_left_pwm 0\n#define motor_right_pwm 1\n#define motor_left_in1 0\n#define motor_left_in2 1\n#define motor_right_in1 2\n#define motor_right_in2 3\n#define GPIO_OUTPUT_INACTIVE 0\n/* Keep format checking active without printing test commands. */\n#define printk(...) ((void)snprintf(NULL, 0, __VA_ARGS__))\nstruct shell { int unused; };\n#define shell_error(sh, ...) ((void)(sh), (void)snprintf(NULL, 0, __VA_ARGS__))\n#define shell_print(sh, ...) ((void)(sh), (void)snprintf(NULL, 0, __VA_ARGS__))\n\nstatic int pins[4], calls, fail_at, pwm_unready, gpio_unready;\nstatic bool pwm_always_fails;\nstatic uint32_t pulses[2];\nstatic int failures;\nstatic int operation(void) {\n    calls++;\n    if (calls == fail_at) { failures++; return -EIO; }\n    return 0;\n}\nstatic bool pwm_is_ready_dt(const struct pwm_dt_spec *p) {\n    return p->channel != pwm_unready;\n}\nstatic bool gpio_is_ready_dt(const struct gpio_dt_spec *p) {\n    return p->pin != gpio_unready;\n}\nstatic int gpio_pin_configure_dt(const struct gpio_dt_spec *p, int mode) {\n    (void)mode;\n    /* Initialization must disable both channels before changing direction. */\n    assert(pulses[0] == 0 && pulses[1] == 0);\n    int rc = operation();\n    if (!rc) pins[p->pin] = 0;\n    return rc;\n}\nstatic int gpio_pin_set_dt(const struct gpio_dt_spec *p, int value) {\n    /* Direction changes must happen with this bridge disabled. */\n    assert(pulses[p->pin / 2] == 0);\n    int rc = operation();\n    if (!rc) pins[p->pin] = value;\n    return rc;\n}\nstatic int pwm_set_pulse_dt(const struct pwm_dt_spec *p, uint32_t value) {\n    assert(value <= p->period);\n    int rc = operation();\n    if (pwm_always_fails) return -EIO;\n    if (!rc) pulses[p->channel] = value;\n    return rc;\n}\nstatic void reset(void) {\n    memset(pins, 0, sizeof(pins));\n    memset(pulses, 0, sizeof(pulses));\n    calls = fail_at = failures = 0;\n    pwm_unready = gpio_unready = -1;\n    pwm_always_fails = false;\n}\n'
# The real motor driver calls a marker hook immediately after a successful final write.
STUB = STUB.replace('static int failures;', 'static int failures;\nstatic unsigned marks, last_pwm_call;\nstatic int last_pwm_channel;')
STUB = STUB.replace('if (!rc) pulses[p->channel] = value;',
    'if (!rc) { pulses[p->channel] = value; last_pwm_call=calls; last_pwm_channel=p->channel; }')
STUB = STUB.replace('calls = fail_at = failures = 0;', 'calls = fail_at = failures = 0; marks=last_pwm_call=0;')
STUB += r"""
static int test_points_init(void){return 0;}
static int test_points_pwm_written(void){
    assert(last_pwm_channel==0 && last_pwm_call==(unsigned)calls);
    marks++;
    return 0;
}
"""

TESTS = r"""
int main(void) {
    for (int motor=0; motor<2; ++motor) {
        for (uint32_t duty=0; duty<=100; ++duty) {
            reset(); pulses[motor]=12500;
            pins[2*motor]=pins[2*motor+1]=1;
            assert(sys_motor_set(motor,duty,STOP)==0);
            assert(pins[2*motor]==0 && pins[2*motor+1]==0);
            assert(pulses[motor]==50000 && pulses[1-motor]==0);
            assert(marks==(motor==LEFT_MOTOR ? 1U : 0U));
            assert(calls==4); /* No later normal-PWM write overwrites EN. */
        }
    }
    reset(); assert(sys_motor_set(LEFT_MOTOR,UINT32_MAX,STOP)==0);
    assert(pulses[0]==50000); /* STOP really ignores the duty argument. */
    reset(); assert(motor_brake((motor_mapping_t)-1)==-EINVAL);
    assert(motor_brake(NUM_MOTORS)==-EINVAL && calls==0);
    reset(); assert(motor_brake_all()==0);
    assert(pulses[0]==50000 && pulses[1]==50000);
    for(int failed=1;failed<=4;failed++) {
        reset(); fail_at=failed;
        assert(motor_brake(LEFT_MOTOR)==-EIO);
        assert(marks==0);
        assert(pulses[0]==0); /* failure reported, best-effort EN disable */
    }
    reset(); fail_at=2;
    assert(motor_brake_all()==-EIO);
    assert(pulses[0]==0 && pulses[1]==50000); /* right still attempted */
    reset(); fail_at=6;
    assert(motor_brake_all()==-EIO);
    assert(pulses[0]==50000 && pulses[1]==0);
    reset(); assert(sys_motor_set(LEFT_MOTOR,50,FORWARD)==0);
    assert(pulses[0]==25000 && marks==1);
    reset(); assert(sys_motor_set(LEFT_MOTOR,0,FREE)==0);
    assert(pulses[0]==0 && marks==1);
    reset(); assert(sys_motor_set(RIGHT_MOTOR,50,FORWARD)==0 && marks==0);
    puts("PASS: STOP ignores duty; full EN maintained; both motors attempted; errors propagated");
    return 0;
}
"""
def main():
    with tempfile.TemporaryDirectory(prefix='lab2-brake-') as folder:
        t=Path(folder);(t/'stub.h').write_text(STUB)
        (t/'test_points.h').write_text('#include "stub.h"\n')
        for name in ['device.h','devicetree.h','drivers/gpio.h','drivers/pwm.h','sys/printk.h']:
            p=t/'zephyr'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "stub.h"\n')
        src=t/'test.c';src.write_text('#include "logic/drive/motor_control.c"\n#include "logic/drive/brake.c"\n'+TESTS)
        exe=t/'test'
        subprocess.run([os.environ.get('CC','cc'),'-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-g','-I',str(t),'-I',str(ROOT),str(src),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
if __name__=='__main__':main()
