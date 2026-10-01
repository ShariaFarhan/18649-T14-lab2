#ifndef LAB2_DUAL_DRIVE_H
#define LAB2_DUAL_DRIVE_H
#include <stdbool.h>
#include <stdint.h>
enum dual_reason {
    DUAL_OK, DUAL_LINK, DUAL_ENCODER_LEFT, DUAL_ENCODER_RIGHT,
    DUAL_REVERSE_LEFT, DUAL_REVERSE_RIGHT, DUAL_STALL_LEFT, DUAL_STALL_RIGHT,
    DUAL_OUTPUT, DUAL_INPUT, DUAL_SCHEDULING
};
struct dual_drive_status {
    bool armed,braking,boosting,fault;
    int32_t target_mrpm,actual_mrpm,left_mrpm,right_mrpm;
    uint16_t duty_percent;
    enum dual_reason reason;
};
int dual_drive_init(void);
int dual_drive_command(int32_t throttle_raw,int32_t brake_raw,bool enable);
void dual_drive_fail_safe(void);
struct dual_drive_status dual_drive_get(void);
#endif
