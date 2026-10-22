#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>
#include "relay_task.h"
#include "lab3_relay.h"
#include "runtime.h"

static const struct gpio_dt_spec marker=GPIO_DT_SPEC_GET(DT_ALIAS(relay_tx),gpios);
K_MUTEX_DEFINE(relay_lock);
K_SEM_DEFINE(relay_event,0,1);
static struct lab3_relay_plan plan;
static relay_sender_fn send_frame;
static atomic_t failed,queued[2],errors[2];

static void relay_worker(void *a,void *b,void *c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    bool pending=false;
    for (;;) {
        k_sem_take(&relay_event,pending ? K_MSEC(1) : K_MSEC(20));
        if (lab3_local_failed()) {
            relay_task_pause();pending=false;lab3_progress(WORK_RELAY);continue;
        }
        /* Slots are ordered CTRL, source-specific FAULTs, HBs, then BLINK.
         * Never wait for a TX slot on a disconnected bus. */
        for (unsigned key=0;key<9;key++) for (unsigned bus=0;bus<2;bus++) {
            struct lab3_relay_copy copy;
            k_mutex_lock(&relay_lock,K_FOREVER);
            bool have=lab3_relay_candidate(&plan,key,bus,k_uptime_get(),&copy);
            k_mutex_unlock(&relay_lock);
            if (!have) continue;
            struct can_frame frame={.id=copy.id,.dlc=8};
            memcpy(frame.data,copy.data,8);
            lab3_profile_begin(PROFILE_RELAY);
            int rc=send_frame(bus,&frame); /* SPI outside shared-state lock. */
            lab3_profile_end(PROFILE_RELAY);
            if (!rc) {
                atomic_inc(&queued[bus]);
                if (gpio_pin_toggle_dt(&marker)) atomic_set(&failed,1);
                k_mutex_lock(&relay_lock,K_FOREVER);
                lab3_relay_queued(&plan,key,bus,copy.revision);
                k_mutex_unlock(&relay_lock);
            } else if (rc!=-EAGAIN) {
                atomic_inc(&errors[bus]);
            }
        }
        k_mutex_lock(&relay_lock,K_FOREVER);
        pending=lab3_relay_pending(&plan);
        k_mutex_unlock(&relay_lock);
        lab3_progress(WORK_RELAY);
    }
}
/* Above STEER(5)/dispatch(6), below MCP2515 cooperative RX workers. */
K_THREAD_DEFINE(relay_thread,1536,relay_worker,NULL,NULL,NULL,4,0,SYS_FOREVER_MS);
int relay_task_init(relay_sender_fn sender)
{
    if (!sender || !gpio_is_ready_dt(&marker)) { atomic_set(&failed,1);return -ENODEV; }
    int rc=gpio_pin_configure_dt(&marker,GPIO_OUTPUT_INACTIVE);
    if (rc) { atomic_set(&failed,1);return rc; }
    send_frame=sender;
    k_thread_start(relay_thread);
    return 0;
}
void relay_task_observe(const struct can_frame *f,unsigned bus,
                        enum lab3_rx_result result,uint64_t now)
{
    if (lab3_local_failed() || f->dlc!=8 || (f->flags & (CAN_FRAME_IDE|CAN_FRAME_RTR))) return;
    k_mutex_lock(&relay_lock,K_FOREVER);
    lab3_relay_observe(&plan,f->id,f->data,bus,result,now);
    bool pending=lab3_relay_pending(&plan);
    k_mutex_unlock(&relay_lock);
    if (pending) k_sem_give(&relay_event);
}
void relay_task_pause(void)
{
    k_mutex_lock(&relay_lock,K_FOREVER);
    for (unsigned i=0;i<9;i++) plan.slots[i].pending=0;
    k_mutex_unlock(&relay_lock);
}
bool relay_task_failed(void) { return atomic_get(&failed)!=0; }
struct relay_stats relay_task_stats(void)
{
    struct relay_stats s;
    for (unsigned b=0;b<2;b++) { s.queued[b]=atomic_get(&queued[b]);s.failed[b]=atomic_get(&errors[b]); }
    k_mutex_lock(&relay_lock,K_FOREVER);
    s.offered=plan.offered;s.suppressed=plan.suppressed;
    s.superseded=plan.superseded;s.expired=plan.expired;
    k_mutex_unlock(&relay_lock);
    return s;
}
