#ifndef LAB2_SPEED_CONTROL_H_
#define LAB2_SPEED_CONTROL_H_
#include <stdbool.h>
#include <stdint.h>
#include "motor_control.h"
#define SPEED_FULL_THROTTLE_RPM 85U /* PLACEHOLDER: replace with measured sustainable speed + headroom */
#define SPEED_LOOP_PERIOD_MS 10U
enum speed_fault { SPEED_FAULT_NONE, SPEED_FAULT_ENCODER, SPEED_FAULT_DIRECTION,
    SPEED_FAULT_NO_MOTION, SPEED_FAULT_OUTPUT, SPEED_FAULT_EXTERNAL };
struct speed_status {
    bool armed, brake;
    uint16_t throttle_permille;
    int32_t target_rpm_milli, feedback_rpm_milli;
    uint32_t duty_percent;
    enum speed_fault fault;
};
int speed_control_init(void); /* initializes driver; startup brakes, no auto drive */
int speed_control_arm(void); /* requires valid encoders; target starts at zero */
int speed_control_disarm(void); /* brake and stop closed-loop ownership */
int speed_control_set_input(uint16_t throttle_permille,bool brake);
int speed_control_set_brake(bool brake);
int speed_control_set_fault(void);
int speed_control_get(struct speed_status *out);
/* Shell/manual callers use these to avoid racing the active control thread. */
int speed_control_manual_init(void);
int speed_control_manual_set(motor_mapping_t motor,uint32_t duty,direction_t direction);
#endif
