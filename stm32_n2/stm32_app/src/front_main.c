#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "can_comms.h"
#include "front_zone.h"

/* One loop: receive validated CAN events, then let the front zone act. */
static struct front_zone zone;

int main(void)
{
    struct can_comms_rx rx;

    front_zone_init(&zone);
    if (can_comms_init() != 0) printk("Front-zone CAN initialization failed\n");
    if (front_zone_hw_init(&zone, k_uptime_get_32()) != 0)
        printk("Front-zone output initialization failed\n");

    for (;;) {
        /* Bound work per pass so a busy bus cannot starve timeout checks. */
        for (unsigned i = 0; i < 16 && can_comms_receive(&rx, 0) == 0; ++i)
            front_zone_handle(&zone, &rx, k_uptime_get_32());
        front_zone_run_tick(&zone, k_uptime_get_32());
        k_sleep(K_MSEC(5));
    }
}
