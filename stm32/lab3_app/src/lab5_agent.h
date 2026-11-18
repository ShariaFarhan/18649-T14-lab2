#ifndef LAB5_BOARD_AGENT_H
#define LAB5_BOARD_AGENT_H
#include <zephyr/drivers/can.h>
#include "node.h"
typedef int (*lab5_board_sender)(unsigned,const struct can_frame *);
uint64_t lab5_local_us(void);
int lab5_board_init(lab5_board_sender);
bool lab5_board_capture(unsigned bus,const struct can_frame *);
void lab5_board_legacy(unsigned bus,const struct can_frame *,bool fresh);
struct l5_snapshot lab5_board_snapshot(void);
void lab5_board_lights(uint8_t mode,bool error,uint32_t epoch_ms);
unsigned lab5_board_route(void);
bool lab5_board_fault(void);
#endif
