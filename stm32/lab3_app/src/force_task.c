#include <zephyr/kernel.h>
#include "force_task.h"
#include "runtime.h"
static relay_sender_fn send_frame;
static void force_worker(void *a,void *b,void *c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    int64_t next=k_uptime_get();uint8_t seq=0,pending=0;int value=-100;
    struct can_frame frame={.id=CAN_ID_FORCE,.dlc=8};
    for (;;) {
        int64_t now=k_uptime_get();
        if (lab3_local_failed()) pending=0;
        if (now>=next) {
            if (!lab3_local_failed()) {
                frame.data[0]=can_byte0(seq++,0);
                frame.data[FORCE_VALUE]=(uint8_t)(int8_t)value;
                value=value==100 ? -100 : value+1;pending=3;
            }
            next+=((now-next)/50+1)*50;
        }
        if (pending) {
            lab3_profile_begin(PROFILE_FORCE);
            for (unsigned b=0;b<2;b++)
                if ((pending & (1u<<b)) && !send_frame(b,&frame)) pending&=~(1u<<b);
            lab3_profile_end(PROFILE_FORCE);
        }
        lab3_progress(WORK_FORCE);
        int64_t wait=next-k_uptime_get();
        if (wait>20) wait=20;
        if (pending && wait>1) wait=1;
        k_sleep(K_MSEC(wait>0 ? wait : 1));
    }
}
K_THREAD_DEFINE(force_thread,1024,force_worker,NULL,NULL,NULL,6,0,SYS_FOREVER_MS);
void force_task_init(relay_sender_fn sender)
{ send_frame=sender;k_thread_start(force_thread); }
