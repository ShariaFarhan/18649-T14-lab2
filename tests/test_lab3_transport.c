#include <assert.h>
#include <stdatomic.h>
#include <stdio.h>
#include <sched.h>
#include "lab3_fault_policy.h"
#include "../stm32/lab3_app/src/transport_gate.h"
static struct k_mutex gate[2]={{PTHREAD_MUTEX_INITIALIZER},{PTHREAD_MUTEX_INITIALIZER}};
static const struct device dev[2]={{0},{1}};
static bool ready[2]={true,true},started[2]={true,true};
static atomic_bool failed,inside,release_send,drained;
static int queued[2],markers;
static bool allowed(const struct can_frame *f)
{ return lab3_tx_allowed(atomic_load(&failed),2,f->id,f->data); }
static void marked(void *arg) { (void)arg;++markers; }
int can_send(const struct device *d,const struct can_frame *f,int wait,can_tx_callback_t cb,void *arg)
{
    (void)f;(void)wait;(void)cb;(void)arg;assert(started[d->bus]);
    if (d->bus==0) { atomic_store(&inside,true);while (!atomic_load(&release_send)) sched_yield(); }
    assert(started[d->bus]);queued[d->bus]++;return 0;
}
int can_stop(const struct device *d) { started[d->bus]=false;queued[d->bus]=0;return 0; }
int can_start(const struct device *d) { started[d->bus]=true;return 0; }
static struct can_frame hb={.id=CAN_ID_HB_N2,.dlc=8};
static void *sender(void *arg)
{
    (void)arg;assert(!lab3_transport_send(&gate[0],&dev[0],&ready[0],&hb,allowed,NULL,NULL,marked));return NULL;
}
static void *aborter(void *arg)
{
    (void)arg;k_mutex_lock(&gate[0],K_FOREVER);
    assert(!lab3_transport_restart_locked(&dev[0],&ready[0]));
    k_mutex_unlock(&gate[0]);atomic_store(&drained,true);return NULL;
}
int main(void)
{
    pthread_t send,abort;
    pthread_create(&send,NULL,sender,NULL);
    while (!atomic_load(&inside)) sched_yield();
    /* Recovery cannot interleave the driver's started check and queue load. */
    assert(k_mutex_lock(&gate[0],K_NO_WAIT)!=0);
    assert(lab3_transport_send(&gate[0],&dev[0],&ready[0],&hb,allowed,NULL,NULL,NULL)==-EAGAIN);
    /* The other bus still queues while A is busy. */
    assert(!lab3_transport_send(&gate[1],&dev[1],&ready[1],&hb,allowed,NULL,NULL,NULL));
    atomic_store(&failed,true);pthread_create(&abort,NULL,aborter,NULL);
    atomic_store(&release_send,true);pthread_join(send,NULL);pthread_join(abort,NULL);
    assert(atomic_load(&drained) && queued[0]==0 && markers==1);
    /* Reconnect during self-test cannot release an old HB or enqueue a new one. */
    assert(lab3_transport_send(&gate[0],&dev[0],&ready[0],&hb,allowed,NULL,NULL,NULL)==-ENETDOWN);
    struct can_frame fault={.id=CAN_ID_FAULT,.dlc=8,.data={0,2,FAULT_R_SELF_TEST}};
    assert(!lab3_transport_send(&gate[0],&dev[0],&ready[0],&fault,allowed,NULL,NULL,NULL));
    fault.data[FAULT_ZONE]=3;
    assert(lab3_transport_send(&gate[0],&dev[0],&ready[0],&fault,allowed,NULL,NULL,NULL)==-ENETDOWN);
    for (unsigned phase=0;phase<2;phase++) { /* Neither cached phase may suppress fallback. */
        (void)phase;assert(!lab3_follow_master(true,false,false,BLINK_MODE_HAZARD));
    }
    assert(lab3_follow_master(true,true,false,BLINK_MODE_HAZARD));
    assert(!lab3_follow_master(true,true,false,BLINK_MODE_OFF));
    puts("PASS: production TX gate, send/recovery interleaving, other-bus progress, self-test drain/reconnect, master loss");
}
