#include "system_state.h"
#include <string.h>

void system_state_init(struct system_state *s, bool ready)
{
    memset(s, 0, sizeof(*s));
    s->hardware_ready = ready;
    s->output_fault = !ready;
}
void system_state_valid(struct system_state *s)
{
    s->seen_valid = true;
    s->link_ok = true;
    s->bad_input = false;
}
void system_state_link_lost(struct system_state *s)
{
    s->link_ok = false;
    s->click_pending = false; /* A double-press cannot span a lost connection. */
}
void system_state_bad_input(struct system_state *s)
{
    s->bad_input = true;
    s->click_pending = false;
}
void system_state_output_failed(struct system_state *s) { s->output_fault = true; }
void system_state_reinitialized(struct system_state *s, bool success)
{
    s->hardware_ready = success;
    s->output_fault = !success;
}
void system_state_sync_button(struct system_state *s, uint8_t counter)
{
    s->test_counter = counter & 31U;
    s->button_synced = true;
    s->click_pending = false;
    s->have_press = false;
    /* A PC reconnect must never clear a simulated failure. */
}
bool system_state_button(struct system_state *s, uint8_t counter, uint64_t now)
{
    counter &= 31U;
    if (!s->button_synced) { system_state_sync_button(s, counter); return false; }
    uint8_t delta = (counter - s->test_counter) & 31U;
    if (!delta) return false;
    s->test_counter = counter;
    if (delta != 1) {
        /* Lost/coalesced presses cannot be interpreted as a recovery gesture. */
        s->self_test = true;
        s->click_pending = false;
        s->have_press = true;
        s->last_press_ms = now;
        return false;
    }
    uint64_t elapsed = now - s->last_press_ms;
    if (s->have_press && elapsed < SELF_TEST_DEBOUNCE_MS) return false;
    bool recover = s->click_pending && elapsed <= SELF_TEST_DOUBLE_MS;
    s->last_press_ms = now;
    s->have_press = true;
    s->click_pending = !recover;
    s->self_test = !recover;
    return recover;
}
enum system_reason system_state_reason(const struct system_state *s)
{
    if (!s->hardware_ready || s->output_fault) return SYS_OUTPUT_FAULT;
    if (s->self_test) return SYS_SELF_TEST;
    if (s->bad_input) return SYS_BAD_INPUT;
    if (!s->seen_valid) return SYS_STARTUP;
    if (!s->link_ok) return SYS_LINK_LOST;
    return SYS_OK;
}
const char *system_reason_name(enum system_reason reason)
{
    static const char *const names[] = {
        "NORMAL", "STARTUP", "LINK_LOST", "BAD_INPUT", "SELF_TEST", "OUTPUT_FAULT"
    };
    return (unsigned)reason < sizeof(names)/sizeof(names[0]) ? names[reason] : "UNKNOWN";
}
