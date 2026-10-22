#ifndef LAB3_TRANSPORT_GATE_H
#define LAB3_TRANSPORT_GATE_H
#include <errno.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
/* This core is also compiled against a deterministic host CAN/mutex fake. */
static inline int lab3_transport_send(struct k_mutex *gate,const struct device *dev,
        const bool *ready,const struct can_frame *frame,
        bool (*allowed)(const struct can_frame *),can_tx_callback_t done,void *arg,
        void (*marked)(void *))
{
    if (k_mutex_lock(gate,K_NO_WAIT)) return -EAGAIN;
    int rc=(!*ready || !allowed(frame)) ? -ENETDOWN : can_send(dev,frame,K_NO_WAIT,done,arg);
    if (!rc && marked) marked(arg);
    k_mutex_unlock(gate);return rc;
}
/* Caller owns the same gate. Stop aborts the driver's retained TX slot. */
static inline int lab3_transport_restart_locked(const struct device *dev,bool *ready)
{
    int rc=can_stop(dev);
    if (!rc || rc==-EALREADY) rc=can_start(dev);
    *ready=rc==0;return rc;
}
#endif
