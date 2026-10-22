#include "can_comms.h"

#include <string.h>

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>

/* The interrupt callback only queues raw frames; the main loop decodes them. */

#if !DT_NODE_EXISTS(DT_ALIAS(can_a)) || !DT_NODE_EXISTS(DT_ALIAS(can_b))
#error "N2 overlay must define can-a and can-b aliases"
#endif

static const struct device *const buses[2] = {
    DEVICE_DT_GET(DT_ALIAS(can_a)),
    DEVICE_DT_GET(DT_ALIAS(can_b)),
};
static const uint8_t bus_indices[2] = { 0, 1 };
static bool running[2];
struct can_comms_raw {
    struct front_frame frame;
    uint8_t bus;
};
K_MSGQ_DEFINE(rx_queue, sizeof(struct can_comms_raw), 32, 4);

static void received(const struct device *device, struct can_frame *frame,
                     void *user_data)
{
    /* CAN callbacks must not wait for the application to process a frame. */
    ARG_UNUSED(device);
    if (frame->dlc != CAN_FRAME_DLC ||
        (frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) return;

    struct can_comms_raw rx = {
        .frame = { .id = (uint16_t)frame->id, .dlc = frame->dlc },
        .bus = *(const uint8_t *)user_data,
    };
    memcpy(rx.frame.data, frame->data, CAN_FRAME_DLC);
    /* A full queue drops this frame; stale-command/heartbeat timeouts fail safe. */
    (void)k_msgq_put(&rx_queue, &rx, K_NO_WAIT);
}

int can_comms_init(void)
{
    /* Receive all standard IDs; can_comms_decode rejects unknown IDs. */
    const struct can_filter filter = { .id = 0, .mask = 0, .flags = 0 };
    int first_error = -ENODEV;
    bool any_running = false;

    for (uint8_t bus = 0; bus < 2; ++bus) {
        if (!device_is_ready(buses[bus])) continue;

        int filter_id = can_add_rx_filter(buses[bus], received,
                                          (void *)&bus_indices[bus], &filter);
        if (filter_id < 0) {
            first_error = filter_id;
            continue;
        }
        int rc = can_start(buses[bus]);
        if (rc != 0) {
            can_remove_rx_filter(buses[bus], filter_id);
            first_error = rc;
            continue;
        }
        running[bus] = true;
        any_running = true;
    }
    return any_running ? 0 : first_error;
}

int can_comms_receive(struct can_comms_rx *out, uint32_t timeout_ms)
{
    if (out == NULL) return -EINVAL;
    struct can_comms_raw raw;
    int rc = k_msgq_get(&rx_queue, &raw,
                        timeout_ms == 0 ? K_NO_WAIT : K_MSEC(timeout_ms));
    if (rc != 0) return rc;
    /* Decode in the main-loop context, never in the CAN interrupt callback. */
    return can_comms_decode(&raw.frame, raw.bus, out);
}

int can_comms_send_on(uint8_t bus, const struct front_frame *frame)
{
    if (bus > 1 || frame == NULL || frame->id > CAN_STD_ID_MASK ||
        frame->dlc > CAN_FRAME_DLC) return -EINVAL;
    if (!running[bus]) return -ENODEV;

    struct can_frame tx = { .id = frame->id, .dlc = frame->dlc };
    memcpy(tx.data, frame->data, frame->dlc);
    return can_send(buses[bus], &tx, K_MSEC(2), NULL, NULL);
}

int can_comms_send(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (id > CAN_STD_ID_MASK || len > CAN_FRAME_DLC || (len != 0 && data == NULL))
        return -EINVAL;

    /* Both copies keep the same ID, payload, and sequence number. */
    struct front_frame frame = { .id = (uint16_t)id, .dlc = len };
    if (len != 0) memcpy(frame.data, data, len);
    int a = can_comms_send_on(0, &frame);
    int b = can_comms_send_on(1, &frame);
    return a == 0 || b == 0 ? 0 : a;
}
int can_comms_decode(const struct front_frame *frame, uint8_t bus,
                     struct can_comms_rx *out)
{
    if (frame == NULL || out == NULL || bus > 1) return FRONT_DECODE_FORMAT;
    memset(out, 0, sizeof(*out));
    out->frame = *frame;
    out->bus = bus;
    if (frame->dlc > 0) out->sequence = can_seq_of(frame->data);

    switch (frame->id) {
    case CAN_ID_CTRL:
        out->kind = CAN_COMMS_CONTROL;
        out->decode_result = front_decode_control(frame, &out->value.command);
        break;
    case CAN_ID_BLINK:
        out->kind = CAN_COMMS_BLINK;
        if (frame->dlc != CAN_FRAME_DLC)
            out->decode_result = FRONT_DECODE_FORMAT;
        else if (frame->data[BLINK_MODE] > BLINK_MODE_HAZARD ||
                 frame->data[BLINK_PHASE] > 1)
            out->decode_result = FRONT_DECODE_RANGE;
        else {
            out->value.blink = (struct front_blink){
                .mode = frame->data[BLINK_MODE],
                .phase_on = frame->data[BLINK_PHASE] != 0,
                .sequence = can_seq_of(frame->data),
            };
            out->decode_result = FRONT_DECODE_OK;
        }
        break;
    case CAN_ID_HB_N1:
    case CAN_ID_HB_N3:
        out->kind = CAN_COMMS_HEARTBEAT;
        out->decode_result = front_decode_heartbeat(frame, &out->value.heartbeat);
        break;
    case CAN_ID_FAULT:
        out->kind = CAN_COMMS_FAULT;
        out->decode_result = front_decode_fault(frame, &out->value.fault);
        break;
    default:
        out->kind = CAN_COMMS_OTHER;
        out->decode_result = FRONT_DECODE_FORMAT;
        break;
    }
    return 0;
}

/* Packet checks and builders are pure C, so the CAN layout can be tested
 * without a connected Nucleo board. */
static bool frame_is(const struct front_frame *frame, uint16_t id)
{
    return frame != NULL && frame->id == id && frame->dlc == CAN_FRAME_DLC;
}

int front_decode_control(const struct front_frame *frame, struct front_command *out)
{
    if (out == NULL || !frame_is(frame, CAN_ID_CTRL)) return FRONT_DECODE_FORMAT;
    if (can_crc8(frame->data, CTRL_CRC) != frame->data[CTRL_CRC])
        return FRONT_DECODE_CRC;

    uint16_t bits = (uint16_t)frame->data[CTRL_STEER_LO] |
                    ((uint16_t)frame->data[CTRL_STEER_HI] << 8);
    int32_t steering = (bits & 0x8000u) ? (int32_t)bits - 65536 : (int32_t)bits;
    if (steering < CTRL_STEER_MIN || steering > CTRL_STEER_MAX ||
        frame->data[CTRL_THROTTLE] > CTRL_PEDAL_PRESSED ||
        frame->data[CTRL_BRAKE] > CTRL_PEDAL_PRESSED ||
        frame->data[CTRL_RESERVED] != 0)
        return FRONT_DECODE_RANGE;

    *out = (struct front_command){
        .steering = (int16_t)steering,
        .throttle = frame->data[CTRL_THROTTLE],
        .brake = frame->data[CTRL_BRAKE],
        .buttons = frame->data[CTRL_BUTTONS],
        .flags = can_flags_of(frame->data),
        .sequence = can_seq_of(frame->data),
        .stale = (can_flags_of(frame->data) & CTRL_FLAG_UDP_STALE) != 0,
    };
    return FRONT_DECODE_OK;
}

int front_decode_heartbeat(const struct front_frame *frame, struct front_heartbeat *out)
{
    if (out == NULL || frame == NULL || frame->dlc != CAN_FRAME_DLC)
        return FRONT_DECODE_FORMAT;
    uint8_t zone;
    if (frame->id == CAN_ID_HB_N1) zone = ZONE_COCKPIT;
    else if (frame->id == CAN_ID_HB_N2) zone = ZONE_STEERING;
    else if (frame->id == CAN_ID_HB_N3) zone = ZONE_DRIVETRAIN;
    else return FRONT_DECODE_FORMAT;

    if (frame->data[HB_STATE] > ZONE_STATE_LOCAL_FAULT ||
        (frame->data[HB_BUS] & (uint8_t)~(HB_BUS_A_DEGRADED | HB_BUS_B_DEGRADED)))
        return FRONT_DECODE_RANGE;
    *out = (struct front_heartbeat){
        .zone = zone,
        .state = frame->data[HB_STATE],
        .bus_health = frame->data[HB_BUS],
        .sequence = can_seq_of(frame->data),
    };
    return FRONT_DECODE_OK;
}

int front_decode_fault(const struct front_frame *frame, struct front_fault *out)
{
    if (out == NULL || !frame_is(frame, CAN_ID_FAULT)) return FRONT_DECODE_FORMAT;
    if (frame->data[FAULT_ZONE] < ZONE_COCKPIT ||
        frame->data[FAULT_ZONE] > ZONE_DRIVETRAIN ||
        frame->data[FAULT_REASON] < FAULT_R_SELF_TEST ||
        frame->data[FAULT_REASON] > FAULT_R_SHUTDOWN)
        return FRONT_DECODE_RANGE;
    *out = (struct front_fault){
        .zone = frame->data[FAULT_ZONE],
        .reason = frame->data[FAULT_REASON],
        .sequence = can_seq_of(frame->data),
    };
    return FRONT_DECODE_OK;
}

int front_build_heartbeat(uint8_t sequence, uint8_t state, uint8_t bus_health,
                          uint8_t reason, struct front_frame *out)
{
    if (out == NULL || sequence > CAN_SEQ_MASK || state > ZONE_STATE_LOCAL_FAULT ||
        (bus_health & (uint8_t)~(HB_BUS_A_DEGRADED | HB_BUS_B_DEGRADED)))
        return FRONT_DECODE_RANGE;
    *out = (struct front_frame){ .id = CAN_ID_HB_N2, .dlc = CAN_FRAME_DLC };
    out->data[CAN_B0] = can_byte0(sequence, state != ZONE_STATE_NORMAL ?
                                                CAN_FLAG_SYS_ERROR : 0);
    out->data[HB_STATE] = state;
    out->data[HB_BUS] = bus_health;
    out->data[HB_REASON] = reason;
    return FRONT_DECODE_OK;
}

int front_build_fault(uint8_t sequence, uint8_t reason, struct front_frame *out)
{
    if (out == NULL || sequence > CAN_SEQ_MASK ||
        reason < FAULT_R_SELF_TEST || reason > FAULT_R_SHUTDOWN)
        return FRONT_DECODE_RANGE;
    *out = (struct front_frame){ .id = CAN_ID_FAULT, .dlc = CAN_FRAME_DLC };
    out->data[CAN_B0] = can_byte0(sequence, CAN_FLAG_SYS_ERROR);
    out->data[FAULT_ZONE] = ZONE_STEERING;
    out->data[FAULT_REASON] = reason;
    return FRONT_DECODE_OK;
}

int front_build_force(uint8_t sequence, int8_t force, struct front_frame *out)
{
    if (out == NULL || sequence > CAN_SEQ_MASK || force < FORCE_MIN || force > FORCE_MAX)
        return FRONT_DECODE_RANGE;
    *out = (struct front_frame){ .id = CAN_ID_FORCE, .dlc = CAN_FRAME_DLC };
    out->data[CAN_B0] = can_byte0(sequence, 0);
    out->data[FORCE_VALUE] = (uint8_t)force;
    return FRONT_DECODE_OK;
}

static int slot_for_id(uint16_t id)
{
    switch (id) {
    case CAN_ID_CTRL: return 0;
    case CAN_ID_FAULT: return 1;
    case CAN_ID_HB_N1: return 2;
    case CAN_ID_HB_N2: return 3;
    case CAN_ID_HB_N3: return 4;
    case CAN_ID_BLINK: return 5;
    case CAN_ID_FORCE: return 6;
    default: return -1;
    }
}

void front_dedup_reset(struct front_dedup *dedup)
{
    if (dedup != NULL) memset(dedup, 0, sizeof(*dedup));
}

enum front_rx_result front_dedup_observe(struct front_dedup *dedup,
                                        uint16_t id, uint8_t sequence,
                                        uint8_t bus, uint32_t now_ms)
{
    int index = slot_for_id(id);
    if (dedup == NULL || index < 0 || bus > 1 || sequence > CAN_SEQ_MASK)
        return FRONT_RX_INVALID;

    struct front_dedup_slot *slot = &dedup->slot[index];
    uint8_t bus_bit = (uint8_t)(1u << bus);
    if (!slot->valid || (uint32_t)(now_ms - slot->last_ms) >= CAN_DEDUP_RESET_MS) {
        *slot = (struct front_dedup_slot){
            .last_ms = now_ms, .sequence = sequence,
            .seen_buses = bus_bit, .valid = true,
        };
        return FRONT_RX_NEW;
    }
    if (sequence == slot->sequence) {
        if (slot->seen_buses & bus_bit) return FRONT_RX_OLD;
        slot->seen_buses |= bus_bit;
        return FRONT_RX_DUPLICATE;
    }
    if (!can_seq_is_new(slot->sequence, sequence)) return FRONT_RX_OLD;
    slot->last_ms = now_ms;
    slot->sequence = sequence;
    slot->seen_buses = bus_bit;
    return FRONT_RX_NEW;
}
