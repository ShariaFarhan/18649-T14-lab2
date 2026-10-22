#ifndef LAB3_STEER_TASK_H
#define LAB3_STEER_TASK_H
#include <stdbool.h>
#include <stdint.h>
int steer_task_init(void);
void steer_task_submit(int16_t command, bool normal, uint32_t generation);
bool steer_task_failed(void);
uint32_t steer_task_pulse_us(void);
#endif
