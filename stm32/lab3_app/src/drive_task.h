#ifndef LAB3_DRIVE_TASK_H
#define LAB3_DRIVE_TASK_H
#include <stdbool.h>
#include <stdint.h>
struct lab3_drive_status { uint8_t duty; bool braking, fault; };
int drive_task_init(void);
void drive_task_submit(uint8_t throttle, uint8_t brake, bool normal, uint32_t generation);
bool drive_task_failed(void);
struct lab3_drive_status drive_task_status(void);
#endif
