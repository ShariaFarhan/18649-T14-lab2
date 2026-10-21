#ifndef LAB2_SYSTEM_STATE_H
#define LAB2_SYSTEM_STATE_H
#include <stdbool.h>
#include <stdint.h>

#define SELF_TEST_DEBOUNCE_MS 50U
#define SELF_TEST_DOUBLE_MS 500U

enum system_reason {
    SYS_OK, SYS_STARTUP, SYS_LINK_LOST, SYS_BAD_INPUT, SYS_SELF_TEST, SYS_OUTPUT_FAULT
};
struct system_state {
    bool hardware_ready, seen_valid, link_ok, bad_input, self_test, output_fault;
    bool click_pending, have_press, button_synced;
    uint8_t test_counter;
    uint64_t last_press_ms;
};
void system_state_init(struct system_state *s, bool hardware_ready);
void system_state_valid(struct system_state *s);
void system_state_link_lost(struct system_state *s);
void system_state_bad_input(struct system_state *s);
void system_state_output_failed(struct system_state *s);
void system_state_reinitialized(struct system_state *s, bool success);
void system_state_sync_button(struct system_state *s, uint8_t counter);
/* Returns true only for the second debounced press of a <=500 ms pair. */
bool system_state_button(struct system_state *s, uint8_t counter, uint64_t now_ms);
enum system_reason system_state_reason(const struct system_state *s);
const char *system_reason_name(enum system_reason reason);
#endif
