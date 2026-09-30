"""Exercise actual blinker C code, including stale queued-work and auto-cancel."""
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#define ARG_UNUSED(x) (void)(x)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define DT_ALIAS(x) x
#define DT_NODE_HAS_STATUS(x,y) 1
#define GPIO_DT_SPEC_GET(x,y) {.pin=x}
#define blinker_fl 0
#define blinker_fr 1
#define blinker_rl 2
#define blinker_rr 3
#define GPIO_OUTPUT_INACTIVE 0
struct gpio_dt_spec {int pin;};
struct k_spinlock {bool held;};
typedef unsigned k_spinlock_key_t;
struct k_work {int unused;};
struct k_timer {int unused;};
#define K_WORK_DEFINE(name,handler) struct k_work name
#define K_TIMER_DEFINE(name,handler,stop) struct k_timer name
#define K_MSEC(x) (x)
static int pins[4];
static int64_t clock_ms;
static unsigned first_ms, period_ms;
static int64_t k_uptime_get(void){return clock_ms;}
static unsigned k_spin_lock(struct k_spinlock *s){assert(!s->held);s->held=true;return 0;}
static void k_spin_unlock(struct k_spinlock *s,unsigned key){(void)key;assert(s->held);s->held=false;}
static bool gpio_is_ready_dt(const struct gpio_dt_spec *p){(void)p;return true;}
static int gpio_pin_configure_dt(const struct gpio_dt_spec *p,int mode){(void)mode;pins[p->pin]=0;return 0;}
static int gpio_pin_set_dt(const struct gpio_dt_spec *p,int v){pins[p->pin]=v;return 0;}
static void k_timer_stop(struct k_timer *t){(void)t;}
static void k_timer_start(struct k_timer *t,unsigned first,unsigned period){(void)t;first_ms=first;period_ms=period;}
static int k_work_cancel(struct k_work *w){(void)w;return 0;}
static int k_work_submit(struct k_work *w){(void)w;return 0;}
'''
TEST = r'''
static void expect(int fl,int fr,int rl,int rr){assert(pins[0]==fl&&pins[1]==fr&&pins[2]==rl&&pins[3]==rr);}
int main(void){
 assert(blinkers_init()==0);expect(0,0,0,0);
 assert(blinkers_set_mode(BLINKER_MODE_LEFT)==0);expect(1,0,1,0);
 assert(first_ms==500 && period_ms==500);
 clock_ms=499;blink_work_handler(&blink_work);expect(1,0,1,0);
 clock_ms=500;blink_work_handler(&blink_work);expect(0,0,0,0);
 clock_ms=1000;blink_work_handler(&blink_work);expect(1,0,1,0);
 clock_ms=1250;assert(blinkers_set_mode(BLINKER_MODE_RIGHT)==0);expect(0,1,0,1);
 /* Old work queued by the left timer must not shorten the new first ON phase. */
 clock_ms=1500;blink_work_handler(&blink_work);expect(0,1,0,1);
 clock_ms=1750;blink_work_handler(&blink_work);expect(0,0,0,0);
 blinkers_update_steering(34);assert(!blinkers_turn_crossed());
 blinkers_update_steering(35);assert(blinkers_turn_crossed());
 blinkers_update_steering(11);assert(blinkers_get_mode()==BLINKER_MODE_RIGHT);
 blinkers_update_steering(10);assert(blinkers_get_mode()==BLINKER_MODE_OFF);expect(0,0,0,0);
 assert(blinkers_set_mode(BLINKER_MODE_LEFT)==0);
 blinkers_update_steering(-50);assert(blinkers_turn_crossed());
 blinkers_update_steering(0);assert(blinkers_get_mode()==BLINKER_MODE_OFF);
 assert(blinkers_set_error(true)==0);expect(1,1,1,1);
 assert(first_ms==250 && period_ms==250);
 clock_ms+=250;blink_work_handler(&blink_work);expect(0,0,0,0);
 clock_ms+=250;blink_work_handler(&blink_work);expect(1,1,1,1);
 blinkers_update_steering(-100);blinkers_update_steering(0);
 assert(blinkers_get_mode()==BLINKER_MODE_HAZARD);
 assert(blinkers_set_mode((enum blinker_mode)99)==-EINVAL);
 assert(blinkers_set_mode(BLINKER_MODE_OFF)==0);
 clock_ms+=1000;blink_work_handler(&blink_work);expect(0,0,0,0);
 return 0;
}
'''

def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        (root / 'stub.h').write_text(STUB)
        for name in ('device.h','devicetree.h','kernel.h','drivers/gpio.h','sys/util.h'):
            p=root/'zephyr'/name
            p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text('#include "stub.h"\n')
        c=root/'test.c'
        c.write_text(f'#include "{ROOT / "logic/blinkers/blinkers.c"}"\n'+TEST)
        subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Wno-unused-function',
                        '-fsanitize=address,undefined','-I',str(root),str(c),'-o',str(root/'test')],check=True)
        subprocess.run([str(root/'test')],check=True)
    print('Blinker modes, periods, paired outputs, queued-work race and auto-cancel passed')

if __name__=='__main__':main()
