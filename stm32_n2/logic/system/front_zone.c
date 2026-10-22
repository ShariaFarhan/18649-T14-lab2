#include "front_zone.h"

#include <string.h>

/* Zone policy: turn decoded CAN events and elapsed time into steering, lamp,
 * fault, and relay decisions. Hardware application is at the bottom. */

static bool fresh(bool seen, uint32_t last_ms, uint32_t now_ms, uint32_t timeout_ms)
{
    return seen && (uint32_t)(now_ms - last_ms) < timeout_ms;
}

static bool peer_alive(const struct front_zone_peer *peer, uint32_t now_ms)
{
    return fresh(peer->seen_bus[0], peer->last_bus_ms[0], now_ms,
                 FRONT_HEARTBEAT_TIMEOUT_MS) ||
           fresh(peer->seen_bus[1], peer->last_bus_ms[1], now_ms,
                 FRONT_HEARTBEAT_TIMEOUT_MS);
}

static bool any_peer_seen(const struct front_zone *zone)
{
    for (unsigned p = 0; p < 2; ++p) {
        if (zone->peer[p].seen_bus[0] || zone->peer[p].seen_bus[1]) return true;
    }
    return false;
}

static void request_fault(struct front_zone *zone, uint8_t reason,
                          uint32_t now_ms, struct front_zone_action *action)
{
    if (!zone->have_fault_tx ||
        (uint32_t)(now_ms - zone->last_fault_tx_ms) >= FRONT_FAULT_MIN_GAP_MS) {
        action->fault_requested = true;
        action->fault_reason = reason;
        zone->have_fault_tx = true;
        zone->last_fault_tx_ms = now_ms;
        zone->pending_fault = false;
    } else {
        zone->pending_fault = true;
        zone->pending_fault_reason = reason;
    }
}

static void flush_fault(struct front_zone *zone, uint32_t now_ms,
                        struct front_zone_action *action)
{
    if (zone->pending_fault && !action->fault_requested &&
        (uint32_t)(now_ms - zone->last_fault_tx_ms) >= FRONT_FAULT_MIN_GAP_MS) {
        request_fault(zone, zone->pending_fault_reason, now_ms, action);
    }
}

static void refresh(struct front_zone *zone, uint32_t now_ms,
                    struct front_zone_action *action)
{
    /* Re-evaluate the whole zone after a packet or timer tick. A missing
     * command/heartbeat prevents normal steering and selects hazards. */
    struct front_zone_status old = zone->status;
    bool seen = any_peer_seen(zone);
    for (unsigned bus = 0; bus < 2; ++bus) {
        bool bus_alive = false;
        for (unsigned p = 0; p < 2; ++p) {
            bus_alive |= fresh(zone->peer[p].seen_bus[bus],
                               zone->peer[p].last_bus_ms[bus], now_ms,
                               FRONT_HEARTBEAT_TIMEOUT_MS);
        }
        /* At cold start, no bus is called degraded until some peer appears. */
        zone->status.degraded[bus] = seen && !bus_alive;
    }

    bool peers_ok = peer_alive(&zone->peer[0], now_ms) &&
                    peer_alive(&zone->peer[1], now_ms);
    bool peer_error = (peer_alive(&zone->peer[0], now_ms) &&
                       zone->peer[0].state != ZONE_STATE_NORMAL) ||
                      (peer_alive(&zone->peer[1], now_ms) &&
                       zone->peer[1].state != ZONE_STATE_NORMAL);
    bool ctrl_ok = fresh(zone->have_ctrl, zone->last_ctrl_ms, now_ms,
                         FRONT_CTRL_TIMEOUT_MS) && !zone->ctrl_stale;
    bool remote_fault = fresh(zone->have_remote_fault, zone->last_remote_fault_ms,
                              now_ms, FRONT_FAULT_HOLD_MS);

    if (zone->local_fault) zone->status.state = FRONT_ZONE_LOCAL_FAULT;
    else if (!zone->have_ctrl ||
             (!zone->peer[0].seen_bus[0] && !zone->peer[0].seen_bus[1]) ||
             (!zone->peer[1].seen_bus[0] && !zone->peer[1].seen_bus[1]))
        zone->status.state = FRONT_ZONE_STARTUP;
    else if (!peers_ok || !ctrl_ok || peer_error || remote_fault ||
             zone->bad_input || zone->output_fault)
        zone->status.state = FRONT_ZONE_SYSTEM_ERROR;
    else zone->status.state = FRONT_ZONE_NORMAL;

    zone->status.error = zone->status.state != FRONT_ZONE_NORMAL;
    if (zone->status.error) {
        zone->status.steering = 0;
    } else {
        zone->status.steering = zone->desired_steering;
    }
    front_blinkers_resolve(zone->have_blink ? zone->blink.mode : BLINK_MODE_OFF,
                           zone->have_blink && zone->blink.phase_on,
                           zone->status.error, now_ms, &zone->status.lamps);
    if (old.steering != zone->status.steering || old.state != zone->status.state) {
        action->steering_changed = true;
        action->steering = zone->status.steering;
    }
    if (old.lamps.left != zone->status.lamps.left ||
        old.lamps.right != zone->status.lamps.right || old.state != zone->status.state) {
        action->lamps_changed = true;
        action->lamps = zone->status.lamps;
    }
    if (old.state == FRONT_ZONE_NORMAL && zone->status.state == FRONT_ZONE_SYSTEM_ERROR &&
        !zone->bad_input && !remote_fault) {
        request_fault(zone, FAULT_R_ZONE_LOST, now_ms, action);
    }
}

void front_zone_init(struct front_zone *zone)
{
    if (zone == NULL) return;
    memset(zone, 0, sizeof(*zone));
    zone->status.state = FRONT_ZONE_STARTUP;
    zone->status.error = true;
    front_dedup_reset(&zone->dedup);
}

static int peer_index(uint8_t zone)
{
    if (zone == ZONE_COCKPIT) return 0;
    if (zone == ZONE_DRIVETRAIN) return 1;
    return -1;
}

static void maybe_relay(const struct front_zone *zone, const struct front_frame *frame,
                        uint8_t source_zone, uint8_t bus, uint32_t now_ms,
                        struct front_zone_action *action)
{
    int source = peer_index(source_zone);
    int recipient = source_zone == ZONE_COCKPIT ? 1 :
                    source_zone == ZONE_DRIVETRAIN ? 0 : -1;
    if (source < 0 || recipient < 0 || zone->local_fault) return;
    const struct front_zone_peer *peer = &zone->peer[recipient];
    const struct front_zone_peer *sender = &zone->peer[source];
    uint8_t lost_bus_bit = bus == 0 ? HB_BUS_A_DEGRADED : HB_BUS_B_DEGRADED;
    uint8_t other_bus_bit = bus == 0 ? HB_BUS_B_DEGRADED : HB_BUS_A_DEGRADED;
    uint8_t other = 1u - bus;
    if (!peer_alive(sender, now_ms) ||
        !fresh(peer->seen_bus[other], peer->last_bus_ms[other], now_ms,
               FRONT_HEARTBEAT_TIMEOUT_MS)) return;
    if ((peer->reported_bus_health & lost_bus_bit) &&
        !(peer->reported_bus_health & other_bus_bit) &&
        (sender->reported_bus_health & other_bus_bit)) {
        action->relay_requested = true;
        action->relay_bus = other;
        action->relay_frame = *frame;
    }
}

int front_zone_receive(struct front_zone *zone, const struct can_comms_rx *rx,
                       uint32_t now_ms, struct front_zone_action *action)
{
    if (zone == NULL || rx == NULL || action == NULL || rx->bus > 1)
        return FRONT_DECODE_FORMAT;
    memset(action, 0, sizeof(*action));
    const struct front_frame *frame = &rx->frame;
    uint8_t bus = rx->bus;
    uint8_t source_zone = 0;

    if (rx->kind == CAN_COMMS_CONTROL && rx->decode_result != FRONT_DECODE_OK) {
        zone->bad_input = true;
        request_fault(zone, FAULT_R_BAD_INPUT, now_ms, action);
        refresh(zone, now_ms, action);
        return rx->decode_result;
    }
    if (rx->decode_result != FRONT_DECODE_OK) return rx->decode_result;

    switch (rx->kind) {
    case CAN_COMMS_CONTROL:
    case CAN_COMMS_BLINK:
        source_zone = ZONE_COCKPIT;
        break;
    case CAN_COMMS_HEARTBEAT:
        source_zone = rx->value.heartbeat.zone;
        break;
    case CAN_COMMS_FAULT:
        source_zone = rx->value.fault.zone;
        break;
    default:
        return FRONT_DECODE_FORMAT;
    }

    enum front_rx_result rx_result = front_dedup_observe(&zone->dedup, frame->id,
                                                          rx->sequence, bus, now_ms);
    /* Duplicate heartbeats still prove that their particular bus is alive,
     * but a duplicate control packet must not run steering twice. */
    if ((rx_result == FRONT_RX_NEW || rx_result == FRONT_RX_DUPLICATE) &&
        rx->kind == CAN_COMMS_HEARTBEAT) {
        int p = peer_index(source_zone);
        zone->peer[p].seen_bus[bus] = true;
        zone->peer[p].last_bus_ms[bus] = now_ms;
        zone->peer[p].reported_bus_health = rx->value.heartbeat.bus_health;
        zone->peer[p].state = rx->value.heartbeat.state;
    }
    if (rx_result == FRONT_RX_NEW) {
        switch (rx->kind) {
        case CAN_COMMS_CONTROL:
            zone->have_ctrl = true;
            zone->last_ctrl_ms = now_ms;
            zone->ctrl_stale = rx->value.command.stale;
            zone->bad_input = false;
            if (!zone->ctrl_stale) zone->desired_steering = rx->value.command.steering;
            break;
        case CAN_COMMS_BLINK:
            zone->blink = rx->value.blink;
            zone->have_blink = true;
            break;
        case CAN_COMMS_FAULT:
            if (rx->value.fault.zone != ZONE_STEERING) {
                zone->have_remote_fault = true;
                zone->last_remote_fault_ms = now_ms;
            }
            break;
        default:
            break;
        }
        maybe_relay(zone, frame, source_zone, bus, now_ms, action);
    }
    refresh(zone, now_ms, action);
    flush_fault(zone, now_ms, action);
    return FRONT_DECODE_OK;
}

void front_zone_tick(struct front_zone *zone, uint32_t now_ms,
                     struct front_zone_action *action)
{
    if (zone == NULL || action == NULL) return;
    memset(action, 0, sizeof(*action));
    refresh(zone, now_ms, action);
    flush_fault(zone, now_ms, action);
}

void front_zone_test_press(struct front_zone *zone, uint32_t now_ms,
                           struct front_zone_action *action)
{
    if (zone == NULL || action == NULL) return;
    memset(action, 0, sizeof(*action));
    uint32_t elapsed = now_ms - zone->last_test_press_ms;
    if (zone->have_test_press && elapsed < FRONT_TEST_DEBOUNCE_MS) return;
    bool restore = zone->local_fault && zone->have_test_press &&
                   elapsed <= FRONT_TEST_DOUBLE_MS;
    zone->have_test_press = true;
    zone->last_test_press_ms = now_ms;
    zone->local_fault = !restore;
    if (!restore) request_fault(zone, FAULT_R_SELF_TEST, now_ms, action);
    refresh(zone, now_ms, action);
    flush_fault(zone, now_ms, action);
}

void front_zone_set_output_fault(struct front_zone *zone, bool failed,
                                 uint32_t now_ms, struct front_zone_action *action)
{
    if (zone == NULL || action == NULL) return;
    memset(action, 0, sizeof(*action));
    bool new_failure = failed && !zone->output_fault;
    zone->output_fault = failed;
    if (new_failure) request_fault(zone, FAULT_R_OUTPUT, now_ms, action);
    refresh(zone, now_ms, action);
    flush_fault(zone, now_ms, action);
}

bool front_zone_heartbeat_fields(const struct front_zone *zone,
                                 uint8_t *state, uint8_t *bus_health)
{
    if (zone == NULL || state == NULL || bus_health == NULL || zone->local_fault) return false;
    *state = (zone->bad_input || zone->output_fault) ? ZONE_STATE_LOCAL_FAULT :
             zone->status.error ? ZONE_STATE_SYS_ERROR : ZONE_STATE_NORMAL;
    *bus_health = (zone->status.degraded[0] ? HB_BUS_A_DEGRADED : 0) |
                  (zone->status.degraded[1] ? HB_BUS_B_DEGRADED : 0);
    return true;
}

static bool deadline_due(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void advance_deadline(uint32_t *deadline_ms, uint32_t now_ms, uint32_t period_ms)
{
    *deadline_ms += (1u + (uint32_t)(now_ms - *deadline_ms) / period_ms) * period_ms;
}

void front_zone_periodic(struct front_zone *zone, uint32_t now_ms,
                         struct front_zone_periodic_frames *out)
{
    if (zone == NULL || out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (!zone->periodic_started) {
        zone->periodic_started = true;
        zone->next_hb_ms = now_ms;
        zone->next_force_ms = now_ms;
    }
    if (deadline_due(now_ms, zone->next_hb_ms)) {
        advance_deadline(&zone->next_hb_ms, now_ms, FRONT_HB_PERIOD_MS);
        uint8_t state, buses;
        if (front_zone_heartbeat_fields(zone, &state, &buses) &&
            front_build_heartbeat(zone->hb_sequence, state, buses, 0,
                                  &out->heartbeat) == FRONT_DECODE_OK) {
            out->heartbeat_due = true;
            zone->hb_sequence = (zone->hb_sequence + 1u) & CAN_SEQ_MASK;
        }
    }
    if (deadline_due(now_ms, zone->next_force_ms)) {
        advance_deadline(&zone->next_force_ms, now_ms, FRONT_FORCE_PERIOD_MS);
        if (!zone->local_fault &&
            front_build_force(zone->force_sequence, zone->force_counter,
                              &out->force) == FRONT_DECODE_OK) {
            out->force_due = true;
            zone->force_sequence = (zone->force_sequence + 1u) & CAN_SEQ_MASK;
            zone->force_counter = zone->force_counter == FORCE_MAX ?
                                  FORCE_MIN : zone->force_counter + 1;
        }
    }
}

int front_zone_make_fault(struct front_zone *zone, uint8_t reason,
                          struct front_frame *out)
{
    if (zone == NULL) return FRONT_DECODE_FORMAT;
    int result = front_build_fault(zone->fault_sequence, reason, out);
    if (result == FRONT_DECODE_OK)
        zone->fault_sequence = (zone->fault_sequence + 1u) & CAN_SEQ_MASK;
    return result;
}

#include <errno.h>

#include "servo.h"

static bool outputs_ready;

static void execute_action(struct front_zone *zone,
                           const struct front_zone_action *action, uint32_t now_ms)
{
    int rc = 0;
    if (outputs_ready && action->steering_changed)
        rc = servo_set_steering(action->steering);
    if (outputs_ready && action->lamps_changed) {
        int lamp_rc = front_blinkers_apply(&action->lamps);
        if (rc == 0) rc = lamp_rc;
    }
    if (rc != 0) {
        struct front_zone_action failure;
        front_zone_set_output_fault(zone, true, now_ms, &failure);
        /* Best effort: center steering and show hazards after an output failure. */
        if (failure.steering_changed) (void)servo_set_steering(failure.steering);
        if (failure.lamps_changed) (void)front_blinkers_apply(&failure.lamps);
        if (failure.fault_requested) {
            struct front_frame fault;
            if (front_zone_make_fault(zone, failure.fault_reason, &fault) == FRONT_DECODE_OK)
                (void)can_comms_send(fault.id, fault.data, fault.dlc);
        }
    }
    if (action->fault_requested) {
        struct front_frame fault;
        if (front_zone_make_fault(zone, action->fault_reason, &fault) == FRONT_DECODE_OK)
            (void)can_comms_send(fault.id, fault.data, fault.dlc);
    }
    if (action->relay_requested)
        (void)can_comms_send_on(action->relay_bus, &action->relay_frame);
}

int front_zone_hw_init(struct front_zone *zone, uint32_t now_ms)
{
    if (zone == NULL) return -EINVAL;
    outputs_ready = false;
    int rc = front_blinkers_init();
    if (rc == 0) rc = servo_prepare(); /* PWM stays off until steering is requested. */
    outputs_ready = rc == 0;

    struct front_zone_action action;
    if (outputs_ready) front_zone_tick(zone, now_ms, &action);
    else front_zone_set_output_fault(zone, true, now_ms, &action);
    execute_action(zone, &action, now_ms);
    return rc;
}

void front_zone_handle(struct front_zone *zone, const struct can_comms_rx *rx,
                       uint32_t now_ms)
{
    if (zone == NULL || rx == NULL) return;
    struct front_zone_action action;
    (void)front_zone_receive(zone, rx, now_ms, &action);
    execute_action(zone, &action, now_ms);
}

void front_zone_run_tick(struct front_zone *zone, uint32_t now_ms)
{
    if (zone == NULL) return;
    struct front_zone_action action;
    struct front_zone_periodic_frames periodic;
    front_zone_tick(zone, now_ms, &action);
    execute_action(zone, &action, now_ms);
    front_zone_periodic(zone, now_ms, &periodic);
    if (periodic.heartbeat_due)
        (void)can_comms_send(periodic.heartbeat.id, periodic.heartbeat.data,
                             periodic.heartbeat.dlc);
    if (periodic.force_due)
        (void)can_comms_send(periodic.force.id, periodic.force.data,
                             periodic.force.dlc);
}
