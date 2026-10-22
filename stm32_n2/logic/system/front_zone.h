#ifndef FRONT_ZONE_H
#define FRONT_ZONE_H

#include <stdbool.h>
#include <stdint.h>

#include "can_comms.h"
#include "front_blinkers.h"

#define FRONT_HEARTBEAT_TIMEOUT_MS 60u
#define FRONT_CTRL_TIMEOUT_MS 60u
#define FRONT_FAULT_HOLD_MS 100u
#define FRONT_TEST_DEBOUNCE_MS 50u
#define FRONT_TEST_DOUBLE_MS 500u
#define FRONT_FAULT_MIN_GAP_MS 20u
#define FRONT_HB_PERIOD_MS 20u
#define FRONT_FORCE_PERIOD_MS 50u

enum front_zone_state {
    FRONT_ZONE_STARTUP,
    FRONT_ZONE_NORMAL,
    FRONT_ZONE_SYSTEM_ERROR,
    FRONT_ZONE_LOCAL_FAULT,
};

struct front_zone_status {
    enum front_zone_state state;
    bool degraded[2];
    bool error;
    int16_t steering;             /* normalized servo input, -1000..1000 */
    struct front_blinker_outputs lamps;
};

/* Decisions from the pure state machine; front_zone.c also applies them to hardware. */
struct front_zone_action {
    bool steering_changed;
    int16_t steering;             /* normalized servo input, -1000..1000 */
    bool lamps_changed;
    struct front_blinker_outputs lamps;
    bool fault_requested;
    uint8_t fault_reason;
    bool relay_requested;
    uint8_t relay_bus;            /* 0 = CAN A, 1 = CAN B */
    struct front_frame relay_frame;
};

struct front_zone_periodic_frames {
    bool heartbeat_due;
    struct front_frame heartbeat;
    bool force_due;
    struct front_frame force;
};

struct front_zone_peer {
    bool seen_bus[2];
    uint32_t last_bus_ms[2];
    uint8_t reported_bus_health;
    uint8_t state;
};

struct front_zone {
    struct front_dedup dedup;
    struct front_zone_peer peer[2]; /* N1, N3 */
    struct front_zone_status status;
    struct front_blink blink;
    bool have_blink;
    bool have_ctrl;
    bool ctrl_stale;
    uint32_t last_ctrl_ms;
    int16_t desired_steering;     /* normalized servo input, -1000..1000 */
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
    uint32_t next_force_ms;
    uint8_t hb_sequence;
    uint8_t force_sequence;
    uint8_t fault_sequence;
    int8_t force_counter;
};

void front_zone_init(struct front_zone *zone);

/* Receive an event already decoded by can_comms, never from the CAN ISR.
 * Execute action even when a malformed CTRL makes the return value negative. */
int front_zone_receive(struct front_zone *zone, const struct can_comms_rx *rx,
                       uint32_t now_ms, struct front_zone_action *action);

/* Call at least every 10 ms for heartbeat loss and local hazard timing. */
void front_zone_tick(struct front_zone *zone, uint32_t now_ms,
                     struct front_zone_action *action);

/* One event per debounced physical press; first fails, second within 500 ms restores. */
void front_zone_test_press(struct front_zone *zone, uint32_t now_ms,
                           struct front_zone_action *action);

void front_zone_set_output_fault(struct front_zone *zone, bool failed,
                                 uint32_t now_ms, struct front_zone_action *action);

/* LOCAL_FAULT deliberately suppresses N2 heartbeat transmission. */
bool front_zone_heartbeat_fields(const struct front_zone *zone, uint32_t now_ms,
                                 uint8_t *state, uint8_t *bus_health);

/* Call after tick from the TX scheduler; send each returned frame on A and B. */
void front_zone_periodic(struct front_zone *zone, uint32_t now_ms,
                         struct front_zone_periodic_frames *out);
int front_zone_make_fault(struct front_zone *zone, uint8_t reason,
                          struct front_frame *out);

/* Runtime wrappers: apply the pure decisions above to the N2 servo and lamps. */
int front_zone_hw_init(struct front_zone *zone, uint32_t now_ms);
void front_zone_handle(struct front_zone *zone, const struct can_comms_rx *rx,
                       uint32_t now_ms);
void front_zone_run_tick(struct front_zone *zone, uint32_t now_ms);

#endif
