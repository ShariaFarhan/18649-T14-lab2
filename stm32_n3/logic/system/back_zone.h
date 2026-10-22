#ifndef BACK_ZONE_H
#define BACK_ZONE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_comms.h"
#include "blinkers_back.h"

#define BACK_HEARTBEAT_TIMEOUT_MS 60u
#define BACK_CTRL_TIMEOUT_MS 60u
#define BACK_FAULT_HOLD_MS 100u
#define BACK_TEST_DEBOUNCE_MS 50u
#define BACK_TEST_DOUBLE_MS 500u
#define BACK_FAULT_MIN_GAP_MS 20u
#define BACK_HB_PERIOD_MS 20u

enum back_zone_state {
    BACK_ZONE_STARTUP,
    BACK_ZONE_NORMAL,
    BACK_ZONE_SYSTEM_ERROR,
    BACK_ZONE_LOCAL_FAULT,
};

struct back_zone_status {
    enum back_zone_state state;
    bool degraded[2];
    bool error;
    uint8_t throttle;             /* motor throttle output, 0..255 */
    uint8_t brake;                /* motor brake output, 0..255 */
    struct rear_blinker_outputs lamps;
};

/* Decisions from the pure state machine; back_zone.c also applies them to hardware. */
struct back_zone_action {
    bool motor_changed;
    uint8_t throttle;             /* motor throttle output, 0..255 */
    uint8_t brake;                /* motor brake output, 0..255 */
    bool lamps_changed;
    struct rear_blinker_outputs lamps;
    bool fault_requested;
    uint8_t fault_reason;
    bool relay_requested;
    uint8_t relay_bus;            /* 0 = CAN A, 1 = CAN B */
    struct back_frame relay_frame;
};

struct back_zone_periodic_frames {
    bool heartbeat_due;
    struct back_frame heartbeat;
};

struct back_zone_peer {
    bool seen_bus[2];
    uint32_t last_bus_ms[2];
    uint8_t reported_bus_health;
    uint8_t state;
};

struct back_zone {
    struct back_dedup dedup;
    struct back_zone_peer peer[2]; /* N1 (cockpit), N2 (steering) */
    struct back_zone_status status;
    struct back_blink blink;
    bool have_blink;
    bool have_ctrl;
    bool ctrl_stale;
    uint32_t last_ctrl_ms;
    uint8_t desired_throttle;     /* motor throttle input, 0..255 */
    uint8_t desired_brake;        /* motor brake input, 0..255 */
    bool bad_input;
    bool local_fault;
    bool output_fault;
    bool have_remote_fault;
    uint32_t last_remote_fault_ms;
    bool have_test_press;
    uint32_t last_test_press_ms;
    bool have_fault_tx;
    uint32_t last_fault_tx_ms;
    bool pending_fault;
    uint8_t pending_fault_reason;
    bool periodic_started;
    uint32_t next_hb_ms;
    uint8_t hb_sequence;
    uint8_t fault_sequence;
};

void back_zone_init(struct back_zone *zone);

/* Receive an event already decoded by can_comms, never from the CAN ISR.
 * Execute action even when a malformed CTRL makes the return value negative. */
int back_zone_receive(struct back_zone *zone, const struct can_comms_rx *rx,
                      uint32_t now_ms, struct back_zone_action *action);

/* Call at least every 10 ms for heartbeat loss and local hazard timing. */
void back_zone_tick(struct back_zone *zone, uint32_t now_ms,
                    struct back_zone_action *action);

/* One event per debounced physical press; first fails, second within 500 ms restores. */
void back_zone_test_press(struct back_zone *zone, uint32_t now_ms,
                          struct back_zone_action *action);

void back_zone_set_output_fault(struct back_zone *zone, bool failed,
                                uint32_t now_ms, struct back_zone_action *action);

/* LOCAL_FAULT deliberately suppresses N3 heartbeat transmission. */
bool back_zone_heartbeat_fields(const struct back_zone *zone, uint32_t now_ms,
                                uint8_t *state, uint8_t *bus_health);

/* Call after tick from the TX scheduler; send each returned frame on A and B. */
void back_zone_periodic(struct back_zone *zone, uint32_t now_ms,
                        struct back_zone_periodic_frames *out);
int back_zone_make_fault(struct back_zone *zone, uint8_t reason,
                         struct back_frame *out);

/* Runtime wrappers: apply the pure decisions above to the N3 motor and lamps. */
int back_zone_hw_init(struct back_zone *zone, uint32_t now_ms);
void back_zone_handle(struct back_zone *zone, const struct can_comms_rx *rx,
                      uint32_t now_ms);
void back_zone_run_tick(struct back_zone *zone, uint32_t now_ms);

#endif /* BACK_ZONE_H */