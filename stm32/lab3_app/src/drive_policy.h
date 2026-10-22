#ifndef LAB3_DRIVE_POLICY_H
#define LAB3_DRIVE_POLICY_H
#include <stdbool.h>
#include <stdint.h>
enum lab3_drive_mode { LAB3_DRIVE_COAST, LAB3_DRIVE_FORWARD, LAB3_DRIVE_BRAKE };
struct lab3_drive_output { enum lab3_drive_mode mode; uint8_t duty; };
/* Lab3 scope output, not encoder speed control. Any positive brake wins. */
static inline struct lab3_drive_output lab3_drive_decide(uint8_t throttle,
                                                       uint8_t brake, bool normal)
{
    if (!normal || throttle>100 || brake>100 || brake>0)
        return (struct lab3_drive_output){LAB3_DRIVE_BRAKE,100};
    if (!throttle) return (struct lab3_drive_output){LAB3_DRIVE_COAST,0};
    return (struct lab3_drive_output){LAB3_DRIVE_FORWARD,throttle};
}
#endif
