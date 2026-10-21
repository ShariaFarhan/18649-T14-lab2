#ifndef LAB2_WHEEL_BUTTONS_H
#define LAB2_WHEEL_BUTTONS_H
#include <stdbool.h>
#include <stdint.h>
/* Observed with WinMM: LSB=512, RSB=256. These are zero-based bits. */
#define WHEEL_LSB_MASK (1U << 9)
#define WHEEL_RSB_MASK (1U << 8)
struct wheel_buttons {
    bool synced;
    uint16_t previous;
    int64_t last_press_ms[2];
};
void wheel_buttons_reset(struct wheel_buttons *state);
/* Return one newly pressed side, or zero. First sample only establishes a baseline. */
uint16_t wheel_buttons_pressed(struct wheel_buttons *state, uint16_t buttons, int64_t now_ms);
#endif
