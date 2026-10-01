"""Exercise dual average feedback and motor outputs without hardware."""
import subprocess,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
TEST=r'''
#include "logic/drive/dual_drive.c"
static struct encoder_snapshot feedback;
static direction_t output[2];
static unsigned duty[2],writes[2];
static int failed_motor=-1;
int motor_init(void){output[0]=output[1]=FREE;duty[0]=duty[1]=0;return 0;}
int sys_motor_set(motor_mapping_t m,uint32_t d,direction_t dir){
    writes[m]++;
    if((int)m==failed_motor)return -EIO;
    output[m]=dir;duty[m]=d;return 0;
}
int encoders_init(void){return 0;}
int encoders_get(struct encoder_snapshot *out){*out=feedback;return 0;}
static void setup(void){
    memset(&state,0,sizeof(state));memset(&feedback,0,sizeof(feedback));
    memset(&controller,0,sizeof(controller));memset(writes,0,sizeof(writes));
    ready=arm_ready=ever_armed=brake_input=was_moving=false;failed_motor=-1;
    fake_ms=1000;feedback.wheel[0].valid=feedback.wheel[1].valid=true;
    assert(dual_drive_init()==0);assert(output[0]==FREE&&output[1]==FREE);
}
static void arm(void){
    assert(dual_drive_command(65535,65535,false)==0);
    assert(dual_drive_command(65535,65535,true)==0&&state.armed);
}
static void both(direction_t dir){assert(output[0]==dir&&output[1]==dir&&duty[0]==duty[1]);}
int main(void){
    setup();assert(dual_drive_command(65535,65535,true)==0&&!state.armed);
    arm();both(FREE);
    assert(dual_drive_command(50000,65535,true)==0&&state.boosting&&duty[0]>=40);both(FORWARD);
    assert(dual_drive_command(0,0,true)==0);both(STOP);
    assert(state.braking&&!state.boosting&&state.duty_percent==0);
    assert(dual_drive_command(65535,65535,true)==0);both(FREE);
    setup();arm();
    feedback.wheel[0].rpm_milli=feedback.wheel[1].rpm_milli=120000;
    assert(dual_drive_command(32767,65535,true)==0);both(FORWARD);
    unsigned before=duty[0];assert(state.actual_mrpm==120000);
    fake_ms+=10;feedback.wheel[0].rpm_milli=60000;
    feedback.wheel[0].count++;feedback.wheel[1].count++;
    assert(dual_drive_command(32767,65535,true)==0);
    assert(state.actual_mrpm==90000&&state.target_mrpm>120000&&duty[0]>before);both(FORWARD);
    /* An unequal pair with the same mean gives the same PI input, not right-only speed. */
    setup();arm();feedback.wheel[0].rpm_milli=60000;feedback.wheel[1].rpm_milli=180000;
    assert(dual_drive_command(32767,65535,true)==0&&state.actual_mrpm==120000&&duty[0]==before);
    for(unsigned wheel=0;wheel<2;wheel++){
        setup();arm();feedback.wheel[wheel].valid=false;
        assert(dual_drive_command(0,65535,true)!=0&&state.reason==(wheel?DUAL_ENCODER_RIGHT:DUAL_ENCODER_LEFT));both(STOP);
        setup();arm();feedback.wheel[wheel].rpm_milli=-6000;feedback.wheel[1-wheel].rpm_milli=100000;
        assert(dual_drive_command(0,65535,true)==0&&!state.fault);both(FORWARD);
        setup();arm();
        for(int i=0;i<150;i++){
            fake_ms+=10;feedback.wheel[1-wheel].count++;
            (void)dual_drive_command(0,65535,true);
        }
        assert(!state.fault&&state.armed);both(FORWARD);
        assert(dual_drive_command(0,0,true)==0);both(STOP);
    }
    setup();arm();assert(dual_drive_command(50000,65535,true)==0);
    for(int i=0;i<16;i++){
        fake_ms+=10;feedback.wheel[0].count++;feedback.wheel[1].count++;
        assert(dual_drive_command(50000,65535,true)==0);
    }
    assert(!state.boosting);
    fake_ms=last_command+60;assert(update_locked()!=0&&state.reason==DUAL_LINK&&!state.armed&&!state.fault);both(STOP);
    assert(dual_drive_command(65535,65535,true)==0&&!state.armed);
    arm();assert(state.armed&&!state.fault);
    setup();arm();assert(dual_drive_command(0,65535,true)==0);dual_drive_fail_safe();both(STOP);
    assert(!state.armed);
    setup();arm();failed_motor=0;unsigned previous_right=writes[1];
    assert(dual_drive_command(0,65535,true)!=0&&state.reason==DUAL_OUTPUT);
    assert(writes[1]>previous_right&&output[1]==STOP); /* Still stops the other bridge. */
    puts("PASS: both encoders averaged; lower feedback raises both PWM; no reverse/stall latch; brake; recoverable timeout; boost");
}
'''
STUB='\n#pragma once\n#include <assert.h>\n#include <stdbool.h>\n#include <stdint.h>\n#include <stddef.h>\n#include <string.h>\n#include <stdio.h>\nstruct gpio_dt_spec {int unused;};\nstruct pwm_dt_spec {int unused;};\n#define K_MUTEX_DEFINE(x) int x\n#define K_FOREVER 0\n#define K_TICKS_FOREVER 0\n#define K_MSEC(x) (x)\n#define ARG_UNUSED(x) (void)(x)\n#define K_THREAD_DEFINE(name,sz,fn,a,b,c,pri,opt,delay) static void *name=(void*)fn\nstatic int64_t fake_ms;\nstatic int k_mutex_lock(int *m,int t){(void)m;(void)t;return 0;}\nstatic int k_mutex_unlock(int *m){(void)m;return 0;}\nstatic int64_t k_uptime_get(void){return fake_ms;}\nstatic void k_sleep(int t){(void)t;}\nstatic void k_thread_start(void *t){(void)t;}\n'

with tempfile.TemporaryDirectory() as directory:
    t=Path(directory);(t/'stub.h').write_text(STUB)
    for name in ('kernel.h','drivers/gpio.h','drivers/pwm.h'):
        p=t/'zephyr'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "stub.h"\n')
    (t/'test.c').write_text(TEST)
    subprocess.run(['cc','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I',str(t),'-I',str(ROOT),str(t/'test.c'),str(ROOT/'logic/drive/right_speed_model.c'),'-o',str(t/'test')],check=True)
    subprocess.run([str(t/'test')],check=True)
