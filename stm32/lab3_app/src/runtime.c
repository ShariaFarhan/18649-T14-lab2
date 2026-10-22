#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/atomic.h>
#include "runtime.h"
#include "lab3_supervision.h"
#include "system_state.h"
BUILD_ASSERT(IS_ENABLED(CONFIG_WDT_DISABLE_AT_BOOT),
             "Application-owned IWDG must not be preinstalled by Zephyr boot init");

#ifndef LAB3_PROFILE_TASK
#define LAB3_PROFILE_TASK 0
#endif
static const struct gpio_dt_spec rxpins[]={GPIO_DT_SPEC_GET(DT_ALIAS(can_rx_a),gpios),GPIO_DT_SPEC_GET(DT_ALIAS(can_rx_b),gpios)};
static const struct gpio_dt_spec txpins[]={GPIO_DT_SPEC_GET(DT_ALIAS(can_tx_a),gpios),GPIO_DT_SPEC_GET(DT_ALIAS(can_tx_b),gpios)};
static const struct gpio_dt_spec errpin=GPIO_DT_SPEC_GET(DT_ALIAS(zone_err),gpios);
static const struct gpio_dt_spec profilepin=GPIO_DT_SPEC_GET(DT_ALIAS(task_exec),gpios);
static const struct gpio_dt_spec button=GPIO_DT_SPEC_GET(DT_ALIAS(sw0),gpios);
static const struct device *wdt=DEVICE_DT_GET(DT_NODELABEL(iwdg));
static atomic_t local_failed,hardware_failed,overflow,progress[WORK_COUNT];
static void (*on_button)(bool);
static struct gpio_callback button_cb;
K_MSGQ_DEFINE(button_edges,sizeof(uint64_t),8,4);
static int wdt_channel=-1;
static uint32_t required,last_feed;
static bool watchdog_started;

static int output_init(const struct gpio_dt_spec *pin,bool high)
{
    return gpio_is_ready_dt(pin) ? gpio_pin_configure_dt(pin,high ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE) : -ENODEV;
}
void lab3_progress(enum lab3_worker worker) { atomic_set(&progress[worker],k_uptime_get_32()); }
bool lab3_local_failed(void) { return atomic_get(&local_failed)!=0; }
bool lab3_runtime_failed(void) { return atomic_get(&hardware_failed)!=0; }
void lab3_err(bool error)
{ if (gpio_pin_set_dt(&errpin,error || lab3_local_failed())) atomic_set(&hardware_failed,1); }
void lab3_rx_mark(unsigned bus)
{ if (bus<2 && gpio_pin_toggle_dt(&rxpins[bus])) atomic_set(&hardware_failed,1); }
void lab3_tx_mark(unsigned bus)
{ if (bus<2 && gpio_pin_toggle_dt(&txpins[bus])) atomic_set(&hardware_failed,1); }
void lab3_profile_begin(enum lab3_profile task)
{ if ((int)task==LAB3_PROFILE_TASK && gpio_pin_set_dt(&profilepin,1)) atomic_set(&hardware_failed,1); }
void lab3_profile_end(enum lab3_profile task)
{ if ((int)task==LAB3_PROFILE_TASK && gpio_pin_set_dt(&profilepin,0)) atomic_set(&hardware_failed,1); }
static void button_isr(const struct device *dev,struct gpio_callback *cb,uint32_t pins)
{
    ARG_UNUSED(dev);ARG_UNUSED(cb);ARG_UNUSED(pins);
    uint64_t stamp=k_uptime_get();
    if (k_msgq_put(&button_edges,&stamp,K_NO_WAIT)) atomic_set(&overflow,1);
}
static void button_worker(void *a,void *b,void *c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    struct system_state test;
    system_state_init(&test,true);system_state_sync_button(&test,0);
    uint8_t counter=0;uint64_t last_edge=0;bool have_edge=false;
    for (;;) {
        uint64_t stamp;
        int rc=k_msgq_get(&button_edges,&stamp,K_MSEC(20));
        bool previous=test.self_test;
        if (atomic_set(&overflow,0)) { test.self_test=true;test.click_pending=false; }
        if (!rc && (!have_edge || stamp-last_edge>=SELF_TEST_DEBOUNCE_MS)) {
            last_edge=stamp;have_edge=true;
            (void)system_state_button(&test,++counter,stamp);
        }
        if (test.self_test!=previous) {
            atomic_set(&local_failed,test.self_test);
            if (test.self_test) lab3_err(true);
            if (on_button) on_button(test.self_test);
        }
        lab3_progress(WORK_BUTTON);
    }
}
K_THREAD_DEFINE(button_thread,1024,button_worker,NULL,NULL,NULL,3,0,SYS_FOREVER_MS);
int lab3_runtime_init(void (*action)(bool))
{
    int rc=output_init(&errpin,true);
    for (unsigned b=0;!rc && b<2;b++) {
        rc=output_init(&rxpins[b],false);
        if (!rc) rc=output_init(&txpins[b],false);
    }
    if (!rc) rc=output_init(&profilepin,false);
    if (!rc) rc=gpio_is_ready_dt(&button) ? gpio_pin_configure_dt(&button,GPIO_INPUT) : -ENODEV;
    if (!rc) {
        on_button=action;
        gpio_init_callback(&button_cb,button_isr,BIT(button.pin));
        rc=gpio_add_callback(button.port,&button_cb);
        if (!rc) rc=gpio_pin_interrupt_configure_dt(&button,GPIO_INT_EDGE_TO_ACTIVE);
    }
    if (rc) { atomic_set(&hardware_failed,1);return rc; }
    k_thread_start(button_thread);return 0;
}
int lab3_watchdog_start(uint32_t mask)
{
    if (!device_is_ready(wdt)) return -ENODEV;
    struct wdt_timeout_cfg cfg={.window={.min=0,.max=1000},.flags=WDT_FLAG_RESET_SOC};
    wdt_channel=wdt_install_timeout(wdt,&cfg);
    if (wdt_channel<0) return wdt_channel;
    int rc=wdt_setup(wdt,0); /* Independent oscillator; no debug/sleep bypass. */
    if (rc) return rc;
    required=mask;last_feed=k_uptime_get_32();
    for (unsigned i=0;i<WORK_COUNT;i++) atomic_set(&progress[i],last_feed);
    watchdog_started=true;return 0;
}
void lab3_watchdog_poll(void)
{
    lab3_progress(WORK_MAIN);
    uint32_t now=k_uptime_get_32();
    if (!watchdog_started || now-last_feed<20) return;
    for (unsigned i=0;i<WORK_COUNT;i++)
        if ((required & BIT(i)) && !lab3_progress_fresh(now,(uint32_t)atomic_get(&progress[i]))) return;
    if (wdt_feed(wdt,wdt_channel)) atomic_set(&hardware_failed,1);
    else last_feed=now;
}
