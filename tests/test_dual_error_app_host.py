"""Actual application callbacks and state model; actuator/transport stubs, no hardware."""
from pathlib import Path
import subprocess,tempfile
ROOT=Path(__file__).resolve().parents[1]
STUB=r'''
#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#define K_MUTEX_DEFINE(x) int x
#define K_FOREVER 0
#define ARG_UNUSED(x) (void)(x)
#define SHELL_CMD_REGISTER(a,b,c,d)
struct shell {int unused;};
#define printk(...) ((void)snprintf(NULL,0,__VA_ARGS__))
#define shell_print(sh,...) ((void)(sh),(void)snprintf(NULL,0,__VA_ARGS__))
static int64_t now_ms;
static int k_mutex_lock(int *m,int t){(void)m;(void)t;return 0;}
static int k_mutex_unlock(int *m){(void)m;return 0;}
static int64_t k_uptime_get(void){return now_ms;}
static uint32_t pulse;
struct fake_gpio {uint32_t ODR;};
static struct fake_gpio fake_c;
#define GPIOC (&fake_c)
#define TIM1 0
static uint32_t LL_TIM_OC_GetCompareCH1(int t){(void)t;return pulse;}
'''
TEST=r'''
#define main firmware_main
#include "stm32_app/src/wheel_dual_main.c"
#undef main
static struct dual_drive_status drive;
static enum blinker_mode mode;
static unsigned commands,brakes;
static bool fail_servo;
void pi_link_init(void){}
int dual_drive_init(void){drive=(struct dual_drive_status){0};return 0;}
void dual_drive_fail_safe(void){drive.armed=false;drive.braking=true;drive.duty_percent=0;brakes++;}
struct dual_drive_status dual_drive_get(void){return drive;}
int dual_drive_command(int32_t t,int32_t b,bool enable){
    (void)t;(void)b;commands++;drive.armed=enable;drive.braking=false;return 0;
}
int servo_prepare(void){pulse=0;return 0;}
int servo_center(void){pulse=1500;return 0;}
int servo_set_steering(int16_t p){if(fail_servo)return -EIO;pulse=1500+p*5;return 0;}
uint32_t servo_get_last_pulse_us(void){return pulse;}
int blinkers_init(void){mode=BLINKER_MODE_OFF;return 0;}
int blinkers_set_mode(enum blinker_mode value){mode=value;return 0;}
int blinkers_set_error(bool on){return blinkers_set_mode(on?BLINKER_MODE_HAZARD:BLINKER_MODE_OFF);}
enum blinker_mode blinkers_get_mode(void){return mode;}
int blinkers_update_steering(int16_t p){(void)p;return 0;}
static void expect(enum system_reason reason){
    hb_payload_t hb={0};pi_link_customize_heartbeat(&hb);
    assert(hb.system_reason==reason&&hb.bus_degraded==0xB9);
    if(reason!=SYS_OK)assert(drive.braking&&!drive.armed&&mode==BLINKER_MODE_HAZARD);
}
int main(void){
    _Static_assert(sizeof(cmd_payload_t)==17,"command wire length");
    _Static_assert(sizeof(hb_payload_t)==25,"heartbeat wire length");
    firmware_main();expect(SYS_STARTUP);assert(pulse==0&&brakes>0);
    cmd_payload_t c={.steering_raw=33151,.throttle_raw=65535,.brake_raw=65535,.test_counter=17};
    now_ms=50;pi_link_on_command_received(&c,true);expect(SYS_OK);
    assert(mode==BLINKER_MODE_OFF&&commands==1); /* First old counter is a baseline. */
    now_ms=100;c.test_counter=18;pi_link_on_command_received(&c,true);expect(SYS_SELF_TEST);
    unsigned saved=commands;
    now_ms=120;pi_link_on_command_received(&c,true);expect(SYS_SELF_TEST);assert(commands==saved);
    now_ms=300;c.test_counter=19;pi_link_on_command_received(&c,true);expect(SYS_OK);
    assert(commands==saved+1&&mode==BLINKER_MODE_OFF);
    c.steering_raw=70000;pi_link_on_command_received(&c,false);expect(SYS_BAD_INPUT);
    c.steering_raw=33151;pi_link_on_command_received(&c,true);expect(SYS_OK);
    pi_link_on_link_lost();expect(SYS_LINK_LOST);
    pi_link_on_command_received(&c,true);expect(SYS_OK);
    now_ms=900;c.test_counter=20;pi_link_on_command_received(&c,true);expect(SYS_SELF_TEST);
    pi_link_on_link_lost();expect(SYS_SELF_TEST);
    now_ms=1000;c.test_counter=21;pi_link_on_command_received(&c,true);expect(SYS_SELF_TEST);
    now_ms=1100;c.test_counter=22;pi_link_on_command_received(&c,true);expect(SYS_OK);
    now_ms=1800;c.test_counter=23;pi_link_on_command_received(&c,true);expect(SYS_SELF_TEST);
    c.test_counter=0;c.test_flags=1;pi_link_on_command_received(&c,true);expect(SYS_SELF_TEST);
    c.test_flags=0;now_ms=2000;c.test_counter=1;pi_link_on_command_received(&c,true);expect(SYS_SELF_TEST);
    now_ms=2100;c.test_counter=2;pi_link_on_command_received(&c,true);expect(SYS_OK);
    c.test_flags=2;pi_link_on_command_received(&c,true);expect(SYS_BAD_INPUT);
    c.test_flags=0;pi_link_on_command_received(&c,true);expect(SYS_OK);
    fail_servo=true;pi_link_on_command_received(&c,true);expect(SYS_OUTPUT_FAULT);
    fail_servo=false;pi_link_on_command_received(&c,true);expect(SYS_OUTPUT_FAULT);
    puts("PASS: startup brake/hazards, single/double press, duplicates, reconnect, bad input, latched self-test/output failure");
}
'''
with tempfile.TemporaryDirectory() as directory:
    t=Path(directory);(t/'stub.h').write_text(STUB)
    for name in ('zephyr/kernel.h','zephyr/shell/shell.h','stm32_ll_tim.h'):
        p=t/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('#include "stub.h"\n')
    (t/'test.c').write_text(TEST)
    cmd=['cc','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined',
         '-DLAB2_WHEEL_DUAL_DRIVE','-DLAB2_RAW_WHEEL_LINK','-I',str(t),'-I',str(ROOT)]
    for folder in ('logic/drive','logic/comms','logic/system','logic/steering','logic/blinkers'):cmd+=['-I',str(ROOT/folder)]
    cmd += [str(t/'test.c'),str(ROOT/'logic/system/system_state.c'),str(ROOT/'logic/steering/wheel_mapping.c'),
            str(ROOT/'logic/steering/wheel_buttons.c'),'-o',str(t/'test')]
    subprocess.run(cmd,check=True);subprocess.run([str(t/'test')],check=True)
