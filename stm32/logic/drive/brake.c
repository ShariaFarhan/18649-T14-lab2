#include "brake.h"

int motor_brake_all(void)
{
    int left_rc = motor_brake(LEFT_MOTOR);
    int right_rc = motor_brake(RIGHT_MOTOR);
    return left_rc != 0 ? left_rc : right_rc;
}
