#ifndef CAN_COMMS_H_
#define CAN_COMMS_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* N2's copy of the N3 CAN header. Keep the IDs and eight-byte payload layouts
 * together here; can_comms.c receives and validates them. */
#define CAN_ID_CTRL       0x010u
#define CAN_ID_FAULT      0x020u
#define CAN_ID_HB_N1      0x100u
#define CAN_ID_HB_N2      0x101u
#define CAN_ID_HB_N3      0x102u
#define CAN_ID_BLINK      0x200u
#define CAN_ID_FORCE      0x300u
#define CAN_FRAME_DLC     8u

/* Byte 0: upper nibble = flags, lower nibble = rolling sequence (0..15). */
#define CAN_B0             0u
#define CAN_SEQ_MASK       0x0fu
#define CAN_FLAG_SYS_ERROR 0x01u
#define CTRL_FLAG_UDP_STALE   0x01u
#define CTRL_FLAG_DUAL_ENABLE 0x08u
#define CAN_DEDUP_RESET_MS 100u

static inline uint8_t can_byte0(uint8_t seq, uint8_t flags)
{
    return (uint8_t)((seq & CAN_SEQ_MASK) | ((flags & 0x0fu) << 4));
}
static inline uint8_t can_seq_of(const uint8_t *data) { return data[CAN_B0] & CAN_SEQ_MASK; }
static inline uint8_t can_flags_of(const uint8_t *data) { return data[CAN_B0] >> 4; }
static inline int can_seq_is_new(uint8_t last, uint8_t next)
{
    uint8_t distance = (uint8_t)((next - last) & CAN_SEQ_MASK);
    return distance >= 1u && distance <= 7u;
}

/* Zone IDs carried in FAULT frames. On-wire states are not the old Lab 2
 * INIT/LINK_LOST enum: those are internal conditions, not CAN state values. */
#define ZONE_COCKPIT      1u
#define ZONE_STEERING     2u
#define ZONE_DRIVETRAIN   3u
#define ZONE_STATE_NORMAL      0u
#define ZONE_STATE_SYS_ERROR   1u
#define ZONE_STATE_LOCAL_FAULT 2u

/* CTRL (0x010): signed little-endian steering, normalized before CAN. */
#define CTRL_STEER_LO 1u
#define CTRL_STEER_HI 2u
#define CTRL_THROTTLE 3u
#define CTRL_BRAKE    4u
#define CTRL_BUTTONS  5u
#define CTRL_RESERVED 6u
#define CTRL_CRC      7u
#define CTRL_STEER_MIN (-1000)
#define CTRL_STEER_MAX 1000
#define CTRL_PEDAL_PRESSED 100u
struct __attribute__((packed)) ctrl_payload {
    uint8_t flags_seq;
    int16_t steering; /* Decode as little-endian bytes, not by casting a frame. */
    uint8_t throttle;
    uint8_t brake;
    uint8_t buttons;
    uint8_t reserved;
    uint8_t crc8;
};

static inline uint8_t can_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xffu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1du) : (uint8_t)(crc << 1);
    }
    return (uint8_t)(crc ^ 0xffu);
}

/* FAULT (0x020). */
#define FAULT_ZONE   1u
#define FAULT_REASON 2u
#define FAULT_R_SELF_TEST 1u
#define FAULT_R_BAD_INPUT 2u
#define FAULT_R_UDP_LOST  3u
#define FAULT_R_ZONE_LOST 4u
#define FAULT_R_ISOLATED  5u
#define FAULT_R_OUTPUT    6u
#define FAULT_R_SHUTDOWN  7u
struct __attribute__((packed)) fault_payload {
    uint8_t flags_seq;
    uint8_t failing_zone_id;
    uint8_t reason_code;
    uint8_t reserved[5];
};

/* HB (0x100..0x102): set bits indicate a degraded bus, not a healthy one. */
#define HB_STATE  1u
#define HB_BUS    2u
#define HB_REASON 3u
#define HB_BUS_A_DEGRADED 0x01u
#define HB_BUS_B_DEGRADED 0x02u
struct __attribute__((packed)) hb_payload {
    uint8_t flags_seq;
    uint8_t zone_state;
    uint8_t bus_health;
    uint8_t reserved[5];
};

/* BLINK (0x200): the sender supplies phase; receivers do not free-run. */
#define BLINK_MODE  1u
#define BLINK_PHASE 2u
#define BLINK_MODE_OFF    0u
#define BLINK_MODE_LEFT   1u
#define BLINK_MODE_RIGHT  2u
#define BLINK_MODE_HAZARD 3u
struct __attribute__((packed)) blink_payload {
    uint8_t flags_seq;
    uint8_t mode;
    uint8_t phase;
    uint8_t reserved[5];
};

/* FORCE (0x300): Lab 3 dummy feedback counter. */
#define FORCE_VALUE 1u
#define FORCE_MIN (-100)
#define FORCE_MAX 100
struct __attribute__((packed)) force_payload {
    uint8_t flags_seq;
    int8_t force;
    uint8_t reserved[6];
};

_Static_assert(sizeof(struct ctrl_payload) == CAN_FRAME_DLC, "CTRL must be 8 bytes");
_Static_assert(sizeof(struct fault_payload) == CAN_FRAME_DLC, "FAULT must be 8 bytes");
_Static_assert(sizeof(struct hb_payload) == CAN_FRAME_DLC, "HB must be 8 bytes");
_Static_assert(sizeof(struct blink_payload) == CAN_FRAME_DLC, "BLINK must be 8 bytes");
_Static_assert(sizeof(struct force_payload) == CAN_FRAME_DLC, "FORCE must be 8 bytes");

/* Pin-independent representation of one standard CAN frame. */
struct front_frame {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_FRAME_DLC];
};

/* Decoded values passed to the front zone; the zone never parses payload bytes. */
struct front_command {
    int16_t steering;
    uint8_t throttle;
    uint8_t brake;
    uint8_t buttons;
    uint8_t flags;
    uint8_t sequence;
    bool stale;
};
struct front_blink {
    uint8_t mode;
    bool phase_on;
    uint8_t sequence;
};
struct front_heartbeat {
    uint8_t zone;
    uint8_t state;
    uint8_t bus_health;
    uint8_t sequence;
};
struct front_fault {
    uint8_t zone;
    uint8_t reason;
    uint8_t sequence;
};

enum front_decode_result {
    FRONT_DECODE_OK = 0,
    FRONT_DECODE_FORMAT = -1,
    FRONT_DECODE_CRC = -2,
    FRONT_DECODE_RANGE = -3,
};
enum can_comms_kind {
    CAN_COMMS_CONTROL,
    CAN_COMMS_BLINK,
    CAN_COMMS_HEARTBEAT,
    CAN_COMMS_FAULT,
    CAN_COMMS_OTHER,
};
struct can_comms_rx {
    struct front_frame frame;
    uint8_t bus; /* 0 = CAN A, 1 = CAN B */
    uint8_t sequence; /* Decoded once here, not read from raw bytes by the zone. */
    enum can_comms_kind kind;
    int decode_result; /* A bad CTRL reaches the zone so it can enter fault. */
    union {
        struct front_command command;
        struct front_blink blink;
        struct front_heartbeat heartbeat;
        struct front_fault fault;
    } value;
};

/* One validated event per receive, or a timeout/no-frame error. */
int can_comms_init(void);
int can_comms_receive(struct can_comms_rx *out, uint32_t timeout_ms);
int can_comms_send(uint32_t id, const uint8_t *data, uint8_t len);
int can_comms_send_on(uint8_t bus, const struct front_frame *frame);

/* Exposed for host-side packet tests; can_comms_receive calls this internally. */
int can_comms_decode(const struct front_frame *frame, uint8_t bus,
                     struct can_comms_rx *out);

int front_decode_control(const struct front_frame *frame, struct front_command *out);
int front_decode_heartbeat(const struct front_frame *frame, struct front_heartbeat *out);
int front_decode_fault(const struct front_frame *frame, struct front_fault *out);
int front_build_heartbeat(uint8_t sequence, uint8_t state, uint8_t bus_health,
                          uint8_t reason, struct front_frame *out);
int front_build_fault(uint8_t sequence, uint8_t reason, struct front_frame *out);
int front_build_force(uint8_t sequence, int8_t force, struct front_frame *out);

/* One slot per CAN ID; zone logic uses this to ignore old/redundant commands. */
#define FRONT_DEDUP_SLOTS 7u
struct front_dedup_slot {
    uint32_t last_ms;
    uint8_t sequence;
    uint8_t seen_buses;
    bool valid;
};
struct front_dedup {
    struct front_dedup_slot slot[FRONT_DEDUP_SLOTS];
};
enum front_rx_result {
    FRONT_RX_INVALID = -1,
    FRONT_RX_OLD = 0,
    FRONT_RX_NEW = 1,
    FRONT_RX_DUPLICATE = 2,
};
void front_dedup_reset(struct front_dedup *dedup);
enum front_rx_result front_dedup_observe(struct front_dedup *dedup,
                                        uint16_t id, uint8_t sequence,
                                        uint8_t bus, uint32_t now_ms);

#endif /* CAN_COMMS_H_ */
