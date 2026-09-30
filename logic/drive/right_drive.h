#ifndef LAB2_RIGHT_DRIVE_H
#define LAB2_RIGHT_DRIVE_H
#include <stdbool.h>
#include <stdint.h>
struct right_drive_status {
    bool armed,braking,boosting,fault;
    int32_t target_mrpm,actual_mrpm;
    uint16_t duty_percent;
};
int right_drive_init(void);
int right_drive_command(int32_t throttle_raw,int32_t brake_raw,bool enable);
void right_drive_fail_safe(void);
struct right_drive_status right_drive_get(void);
#endif
