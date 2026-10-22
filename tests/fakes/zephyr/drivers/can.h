#include <stdint.h>
struct device { int bus; };
struct can_frame { uint32_t id;uint8_t dlc;uint8_t data[8]; };
typedef void (*can_tx_callback_t)(const struct device *,int,void *);
int can_send(const struct device *,const struct can_frame *,int,can_tx_callback_t,void *);
int can_stop(const struct device *);
int can_start(const struct device *);
