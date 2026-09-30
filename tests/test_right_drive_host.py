"""Exercise actual right-only controller with simulated time/encoder/output. No hardware."""
import subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
STUB=r'''
#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
struct gpio_dt_spec {int unused;};
struct pwm_dt_spec {int unused;};
#define K_MUTEX_DEFINE(x) int x
#define K_FOREVER 0
#define K_TICKS_FOREVER 0
#define K_MSEC(x) (x)
#define ARG_UNUSED(x) (void)(x)
#define K_THREAD_DEFINE(name,sz,fn,a,b,c,pri,opt,delay) static void *name=(void*)fn
static int64_t fake_ms;
static int k_mutex_lock(int *m,int t){(void)m;(void)t;return 0;}
static int k_mutex_unlock(int *m){(void)m;return 0;}
static int64_t k_uptime_get(void){return fake_ms;}
static void k_sleep(int t){(void)t;}
static void k_thread_start(void *t){(void)t;}
'''
TEST=r'''
#include "logic/drive/right_drive.c"
static struct encoder_snapshot feedback;
static direction_t output[2];
static unsigned duty[2];
static bool output_failure;
int motor_init(void){output[0]=output[1]=FREE;duty[0]=duty[1]=0;return 0;}
int sys_motor_set(motor_mapping_t m,uint32_t d,direction_t dir){
    assert(m!=LEFT_MOTOR || (dir==FREE && d==0));
    if(output_failure)return -EIO;
    output[m]=dir;duty[m]=d;return 0;
}
int encoders_init(void){return 0;}
int encoders_get(struct encoder_snapshot *out){*out=feedback;return 0;}
static void setup(void){
    memset(&state,0,sizeof(state));memset(&feedback,0,sizeof(feedback));
    memset(&controller,0,sizeof(controller));
    ready=arm_ready=ever_armed=brake_input=was_moving=have_count=output_failure=false;
    fake_ms=1000;feedback.wheel[ENCODER_RIGHT].valid=true;
    /* LEFT remains invalid: right-only control must not use average validity. */
    assert(right_drive_init()==0);assert(output[0]==FREE&&output[1]==FREE);
}
static void arm(void){
    assert(right_drive_command(65535,65535,false)==0);
    assert(right_drive_command(65535,65535,true)==0&&state.armed);
}
int main(void){
    setup();assert(right_drive_command(65535,65535,true)==0&&!state.armed);
    arm();assert(output[1]==FREE);
    assert(right_drive_command(50000,65535,true)==0&&state.boosting&&duty[1]>=40);
    assert(right_drive_command(0,0,true)==0); /* Full brake beats full throttle and startup pulse. */
    assert(output[1]==STOP&&state.braking&&!state.boosting&&state.duty_percent==0);
    assert(right_drive_command(65535,65535,true)==0&&output[1]==FREE);
    setup();arm();assert(right_drive_command(50000,65535,true)==0&&state.boosting);
    for(int i=0;i<16;i++){
        fake_ms+=10;feedback.wheel[ENCODER_RIGHT].count++;
        if(i%2==0)assert(right_drive_command(50000,65535,true)==0);
        else assert(update_locked()==0);
    }
    assert(!state.boosting); /* Pulse bounded by MCU time, not the PC. */
    fake_ms=last_command+60;assert(update_locked()!=0);
    assert(!state.armed&&output[1]==STOP&&output[0]==FREE);
    assert(right_drive_command(65535,65535,true)==0&&!state.armed); /* No automatic rearm. */
    setup();arm();assert(right_drive_command(0,65535,true)==0);
    right_drive_fail_safe();assert(!state.armed&&output[1]==STOP&&output[0]==FREE);
    assert(right_drive_command(0,65535,true)==0&&!state.armed);
    setup();arm();feedback.wheel[ENCODER_RIGHT].valid=false;
    assert(right_drive_command(0,65535,true)!=0&&output[1]==STOP);
    setup();arm();feedback.wheel[ENCODER_RIGHT].rpm_milli=-6000;
    assert(right_drive_command(0,65535,true)!=0&&output[1]==STOP);
    setup();arm();
    for(int i=0;i<110&&!state.fault;i++){fake_ms+=10;(void)right_drive_command(40000,65535,true);}
    assert(state.fault&&!state.armed&&output[1]==STOP&&output[0]==FREE);
    setup();arm();output_failure=true;assert(right_drive_command(0,65535,true)!=0&&state.fault&&!state.armed);
    puts("PASS: right-only outputs/feedback, brake priority, bounded startup, stale link, arm guard and faults");
}
'''
with tempfile.TemporaryDirectory() as d:
    t=Path(d);(t/'stub.h').write_text(STUB)
    for name in ('kernel.h','drivers/gpio.h','drivers/pwm.h'):
        p=t/'zephyr'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "stub.h"\n')
    (t/'test.c').write_text(TEST)
    subprocess.run(['cc','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(t),'-I',str(ROOT),str(t/'test.c'),str(ROOT/'logic/drive/right_speed_model.c'),'-o',str(t/'test')],check=True)
    subprocess.run([str(t/'test')],check=True)
