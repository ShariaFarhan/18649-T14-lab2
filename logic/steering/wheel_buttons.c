#include "wheel_buttons.h"
#include <string.h>
void wheel_buttons_reset(struct wheel_buttons *state)
{
    memset(state, 0, sizeof(*state));
    state->last_press_ms[0] = state->last_press_ms[1] = -50;
}
uint16_t wheel_buttons_pressed(struct wheel_buttons *state, uint16_t buttons, int64_t now_ms)
{
    uint16_t pressed = buttons & ~state->previous;
    state->previous = buttons;
    if (!state->synced) { state->synced = true; return 0; }
    const uint16_t both = WHEEL_LSB_MASK | WHEEL_RSB_MASK;
    if ((buttons & both) == both) return 0; /* No new selection while both are held. */
    const uint16_t masks[] = { WHEEL_LSB_MASK, WHEEL_RSB_MASK };
    for (unsigned i = 0; i < 2; ++i) {
        if ((pressed & masks[i]) && now_ms - state->last_press_ms[i] >= 50) {
            state->last_press_ms[i] = now_ms;
            return masks[i];
        }
    }
    return 0;
}
