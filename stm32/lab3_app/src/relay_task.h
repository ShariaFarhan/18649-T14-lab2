#ifndef LAB3_RELAY_TASK_H
#define LAB3_RELAY_TASK_H
#include <zephyr/drivers/can.h>
#include "lab3_can_rx.h"
typedef int (*relay_sender_fn)(unsigned bus,const struct can_frame *frame);
struct relay_stats {
    uint32_t queued[2],failed[2],offered,suppressed,superseded,expired;
};
int relay_task_init(relay_sender_fn sender);
void relay_task_pause(void);
void relay_task_observe(const struct can_frame *frame,unsigned bus,
                        enum lab3_rx_result result,uint64_t now);
bool relay_task_failed(void);
struct relay_stats relay_task_stats(void);
#endif
