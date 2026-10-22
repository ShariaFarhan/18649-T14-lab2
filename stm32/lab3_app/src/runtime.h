#ifndef LAB3_RUNTIME_H
#define LAB3_RUNTIME_H
#include <stdbool.h>
#include <stdint.h>
enum lab3_worker { WORK_MAIN,WORK_CONTROL,WORK_RELAY,WORK_PID,WORK_FORCE,WORK_BUTTON,WORK_COUNT };
enum lab3_profile { PROFILE_DISPATCH=1,PROFILE_STEER,PROFILE_DRIVE,PROFILE_PID,PROFILE_RELAY,PROFILE_FORCE,PROFILE_HB,PROFILE_HEALTH };
int lab3_runtime_init(void (*button_action)(bool));
bool lab3_local_failed(void);
bool lab3_runtime_failed(void);
void lab3_err(bool error);
void lab3_rx_mark(unsigned bus);
void lab3_tx_mark(unsigned bus);
void lab3_progress(enum lab3_worker worker);
int lab3_watchdog_start(uint32_t required_mask);
void lab3_watchdog_poll(void);
void lab3_profile_begin(enum lab3_profile task);
void lab3_profile_end(enum lab3_profile task);
#endif
