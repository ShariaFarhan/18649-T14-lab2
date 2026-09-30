#!/usr/bin/env python3
"""Exercise actual speed_control.c with simulated time/encoders and motor I/O."""
from pathlib import Path
import subprocess,tempfile,os
ROOT=Path(__file__).resolve().parents[1]
STUB=r"""
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
struct pwm_dt_spec { int unused; };
struct gpio_dt_spec { int unused; };
#define GPIO_DT_SPEC_GET(a,b) {0}
#define DT_PATH(x) 0
#define GPIO_OUTPUT_INACTIVE 0
#define K_MUTEX_DEFINE(x) int x
#define K_FOREVER 0
#define K_TICKS_FOREVER 0
#define K_MSEC(x) (x)
#define ARG_UNUSED(x) (void)(x)
#define K_THREAD_DEFINE(id,sz,fn,a,b,c,pri,opt,delay) static void *const id=(void*)fn
static uint64_t fake_us;
static inline int k_mutex_lock(int *m,int t){(void)m;(void)t;return 0;}
static inline int k_mutex_unlock(int *m){(void)m;return 0;}
static inline void k_sleep(int t){(void)t;}
static inline void k_thread_start(void *t){(void)t;}
static inline uint64_t k_uptime_ticks(void){return fake_us;}
static inline uint64_t k_ticks_to_us_floor64(uint64_t t){return t;}
static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *s){(void)s;return true;}
static inline int gpio_pin_configure_dt(const struct gpio_dt_spec *s,int f){(void)s;(void)f;return 0;}
static inline int gpio_pin_toggle_dt(const struct gpio_dt_spec *s){(void)s;return 0;}
"""
TEST=r"""
#include "logic/drive/speed_control.c"
static struct encoder_snapshot feedback;
static direction_t output[2];
static unsigned duty[2], fail_write;
int motor_init(void){return 0;}
int sys_motor_set(motor_mapping_t m,uint32_t p,direction_t d){
    if(fail_write){fail_write--;return -EIO;}
    output[m]=d;duty[m]=p;return 0;
}
int motor_brake_all(void){output[0]=output[1]=STOP;return 0;}
int encoders_get(struct encoder_snapshot *out){*out=feedback;return 0;}
static void setup(void){
    initialized=false;memset(&status,0,sizeof(status));memset(&feedback,0,sizeof(feedback));
    feedback.valid=true;feedback.wheel[0].valid=feedback.wheel[1].valid=true;
    fake_us=10000;fail_write=0;
    assert(speed_control_init()==0);assert(!status.armed&&output[0]==STOP);
    assert(speed_control_arm()==0);assert(output[0]==FREE&&output[1]==FREE);
}
int main(void){
    setup();
    assert(speed_control_set_input(500,false)==0);
    assert(output[0]==FORWARD&&output[1]==FORWARD&&duty[0]>0);
    assert(speed_control_manual_set(LEFT_MOTOR,50,FORWARD)==-EBUSY);
    assert(speed_control_manual_init()==-EBUSY);
    assert(speed_control_set_brake(true)==0);
    fake_us+=10000;assert(update_locked()==0);
    assert(output[0]==STOP&&output[1]==STOP); /* loop cannot overwrite brake */
    feedback.average_rpm_milli=30000;feedback.wheel[0].rpm_milli=25000;feedback.wheel[1].rpm_milli=35000;
    assert(speed_control_set_brake(false)==0&&output[0]==FORWARD);
    assert(status.feedback_rpm_milli==30000);
    assert(speed_control_set_input(0,false)==0&&output[0]==FREE&&output[1]==FREE);
    assert(controller.integral_micro==0);
    assert(speed_control_manual_set(LEFT_MOTOR,0,STOP)==0&&output[0]==STOP&&output[1]==STOP);
    assert(status.brake);
    assert(speed_control_set_input(1001,false)==-EINVAL);
    assert(status.fault==SPEED_FAULT_EXTERNAL&&output[0]==STOP&&output[1]==STOP);
    setup();feedback.valid=false;
    assert(speed_control_set_input(500,false)<0&&status.fault==SPEED_FAULT_ENCODER);
    assert(output[0]==STOP&&output[1]==STOP);
    assert(speed_control_set_input(500,false)==-EIO);
    setup();feedback.wheel[0].rpm_milli=-6000;
    assert(speed_control_set_input(500,false)<0&&status.fault==SPEED_FAULT_DIRECTION);
    setup();fail_write=1;
    assert(speed_control_set_input(500,false)<0&&status.fault==SPEED_FAULT_OUTPUT);
    assert(output[0]==STOP&&output[1]==STOP);
    setup();assert(speed_control_set_input(1000,false)==0);
    for(int i=0;i<110&&!status.fault;i++){fake_us+=10000;(void)update_locked();}
    assert(status.fault==SPEED_FAULT_NO_MOTION&&output[0]==STOP&&output[1]==STOP);
    assert(speed_control_disarm()==0&&!status.armed);
    setup();assert(speed_control_set_fault()<0&&status.fault==SPEED_FAULT_EXTERNAL);
    assert(output[0]==STOP&&output[1]==STOP);
    puts("PASS: actual control wrapper brake priority, coast, ownership, mean feedback and fault latching");
}
"""
with tempfile.TemporaryDirectory(prefix="lab2-speed-") as folder:
    t=Path(folder);(t/"stub.h").write_text(STUB)
    for name in ["kernel.h","drivers/gpio.h","drivers/pwm.h"]:
        p=t/"zephyr"/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "stub.h"\n')
    src=t/"test.c";src.write_text(TEST);exe=t/"test"
    subprocess.run([os.environ.get("CC","cc"),"-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined","-g","-I",str(t),"-I",str(ROOT),str(src),str(ROOT/"logic/drive/speed_pi.c"),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
