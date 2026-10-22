#ifndef BACK_CAN_COMMS_H_
#define BACK_CAN_COMMS_H_ /* Prevent duplicate inclusion. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Message catalog IDs: N1 = Pi, N2 = front STM32, N3 = back STM32. */
#define CAN_ID_CTRL  0x010u /* N1 control command to N2 and N3. */
#define CAN_ID_FAULT 0x020u /* Fault announcement from any node. */
#define CAN_ID_HB_N1 0x100u /* N1 cockpit heartbeat. */
#define CAN_ID_HB_N2 0x101u /* N2 front-zone heartbeat. */
#define CAN_ID_HB_N3 0x102u /* N3 rear-zone heartbeat. */
#define CAN_ID_BLINK 0x200u /* N1 blinker mode and phase. */
#define CAN_ID_FORCE 0x300u /* N2 dummy force feedback to N1. */
#define CAN_FRAME_DLC 8u    /* Data bytes in each catalog frame. */

/** Zone states: values follow N3's CAN header. */
enum zone_state {
    ZONE_STATE_INIT = 0,       /* Startup; not yet ready. */
    ZONE_STATE_NORMAL = 1,     /* Healthy and processing commands. */
    ZONE_STATE_LOCAL_FAIL = 2, /* This node's local self-test failure. */
    ZONE_STATE_LINK_LOST = 3,  /* Commands or peer heartbeats timed out. */
    ZONE_STATE_SYS_ERROR = 4,  /* Input, output, or remote system error. */
};

/** Packed CAN payloads; each is exactly eight bytes. */
struct __attribute__((packed)) ctrl_payload {
    uint8_t flags_seq; /* Flags high nibble; sequence low nibble. */
    int16_t steering;  /* Decode little-endian bytes explicitly. */
    uint8_t throttle;  /* 0..255 */
    uint8_t brake;     /* 0..255 */
    uint8_t buttons;
    uint8_t reserved;
    uint8_t crc8;
};

struct __attribute__((packed)) fault_payload {
    uint8_t flags_seq;
    uint8_t failing_zone_id;
    uint8_t reason_code;
    uint8_t reserved[5];
};

struct __attribute__((packed)) hb_payload {
    uint8_t flags_seq;
    uint8_t zone_state;
    uint8_t bus_health; /* Bit 0 = bus A OK; bit 1 = bus B OK. */
    uint8_t reserved[5];
};

struct __attribute__((packed)) blink_payload {
    uint8_t flags_seq;
    uint8_t mode;  /* OFF, LEFT, RIGHT, HAZARD. */
    uint8_t phase; /* Current on/off phase. */
    uint8_t reserved[5];
};

struct __attribute__((packed)) force_payload {
    uint8_t flags_seq;
    int8_t force;
    uint8_t reserved[6];
};

#define CAN_B0 0u                  /* Byte index of flags/sequence field. */
#define CAN_SEQ_MASK 0x0fu         /* Low four bits carry sequence 0..15. */
#define CAN_FLAG_SYS_ERROR 0x01u   /* Team-defined error flag in high nibble. */
#define CTRL_FLAG_UDP_STALE 0x01u  /* CTRL flag: upstream wheel/link data stale. */
#define ZONE_COCKPIT 1u            /* Numeric FAULT source ID for N1. */
#define ZONE_STEERING 2u           /* Numeric FAULT source ID for N2. */
#define ZONE_DRIVETRAIN 3u         /* Numeric FAULT source ID for N3. */
#define CTRL_STEER_LO 1u           /* CTRL byte index: steering low byte. */
#define CTRL_STEER_HI 2u           /* CTRL byte index: steering high byte. */
#define CTRL_THROTTLE 3u           /* CTRL byte index: throttle value. */
#define CTRL_BRAKE 4u              /* CTRL byte index: brake value. */
#define CTRL_BUTTONS 5u            /* CTRL byte index: wheel button bits. */
#define CTRL_RESERVED 6u           /* CTRL byte index: must be zero. */
#define CTRL_CRC 7u                /* CTRL byte index: CRC over bytes 0..6. */
#define CTRL_STEER_MIN (-1000)     /* Team-defined full-left command. */
#define CTRL_STEER_MAX 1000        /* Team-defined full-right command. */
#define FAULT_ZONE 1u              /* FAULT byte index: failing zone ID. */
#define FAULT_REASON 2u            /* FAULT byte index: reason code. */
#define FAULT_R_SELF_TEST 1u       /* Reason: physical self-test button. */
#define FAULT_R_BAD_INPUT 2u       /* Reason: malformed/out-of-range CTRL. */
#define FAULT_R_ZONE_LOST 4u       /* Reason: peer heartbeat timeout. */
#define FAULT_R_OUTPUT 6u          /* Reason: actuator/output failure. */
#define FAULT_R_SHUTDOWN 7u        /* Reason: cockpit shutdown. */
#define HB_STATE 1u                /* Heartbeat byte index: zone state. */
#define HB_BUS 2u                  /* Heartbeat byte index: bus-health bits. */
#define HB_BUS_A_OK 0x01u          /* Bit set when bus A is reachable. */
#define HB_BUS_B_OK 0x02u          /* Bit set when bus B is reachable. */
#define BLINK_MODE 1u              /* BLINK byte index: selected mode. */
#define BLINK_PHASE 2u             /* BLINK byte index: current on/off phase. */
#define BLINK_MODE_OFF 0u          /* Neither turn lamp requested. */
#define BLINK_MODE_LEFT 1u         /* Left turn lamps requested. */
#define BLINK_MODE_RIGHT 2u        /* Right turn lamps requested. */
#define BLINK_MODE_HAZARD 3u       /* Both sides requested. */
#define FORCE_VALUE 1u             /* FORCE byte index: signed dummy value. */
#define FORCE_MIN (-100)           /* Minimum dummy force value. */
#define FORCE_MAX 100              /* Maximum dummy force value. */

static inline uint8_t can_byte0(uint8_t seq, uint8_t flags)
{
    return (uint8_t)((seq & CAN_SEQ_MASK) | ((flags & 0x0fu) << 4));
}
static inline uint8_t can_seq_of(const uint8_t *data) { return data[CAN_B0] & CAN_SEQ_MASK; }
static inline uint8_t can_flags_of(const uint8_t *data) { return data[CAN_B0] >> 4; }
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

_Static_assert(sizeof(struct ctrl_payload) == CAN_FRAME_DLC, "CTRL must be 8 bytes");
_Static_assert(sizeof(struct fault_payload) == CAN_FRAME_DLC, "FAULT must be 8 bytes");
_Static_assert(sizeof(struct hb_payload) == CAN_FRAME_DLC, "HB must be 8 bytes");
_Static_assert(sizeof(struct blink_payload) == CAN_FRAME_DLC, "BLINK must be 8 bytes");
_Static_assert(sizeof(struct force_payload) == CAN_FRAME_DLC, "FORCE must be 8 bytes");
_Static_assert(offsetof(struct hb_payload, zone_state) == HB_STATE, "HB state offset");
_Static_assert(offsetof(struct hb_payload, bus_health) == HB_BUS, "HB bus offset");

/* N3's duplicate-tracking policy */
#define CAN_DEDUP_RESET_MS 100u /* Forget old sequence after this idle time. */
static inline int can_seq_is_new(uint8_t last, uint8_t next)
{
    uint8_t distance = (uint8_t)((next - last) & CAN_SEQ_MASK);
    return distance >= 1u && distance <= 7u;
}

/* Pin-independent representation of one standard CAN frame. */
struct back_frame {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[CAN_FRAME_DLC];
};

/* Decoded values passed to the back zone; the zone never parses payload bytes. */
struct back_command {
    int16_t steering;
    uint8_t throttle;
    uint8_t brake;
    uint8_t buttons;
    uint8_t flags;
    uint8_t sequence;
    bool stale;
};
struct back_blink {
    uint8_t mode;
    bool phase_on;
    uint8_t sequence;
};
struct back_heartbeat {
    uint8_t zone;
    uint8_t state;
    uint8_t bus_health;
    uint8_t sequence;
};
struct back_fault {
    uint8_t zone;
    uint8_t reason;
    uint8_t sequence;
};

enum back_decode_result {
    BACK_DECODE_OK = 0,
    BACK_DECODE_FORMAT = -1,
    BACK_DECODE_CRC = -2,
    BACK_DECODE_RANGE = -3,
};
enum can_comms_kind {
    CAN_COMMS_CONTROL,
    CAN_COMMS_BLINK,
    CAN_COMMS_HEARTBEAT,
    CAN_COMMS_FAULT,
    CAN_COMMS_OTHER,
};
struct can_comms_rx {
    struct back_frame frame;
    uint8_t bus; /* 0 = CAN A, 1 = CAN B */
    uint8_t sequence; /* Decoded once here, not read from raw bytes by the zone. */
    enum can_comms_kind kind;
    int decode_result; /* A bad CTRL reaches the zone so it can enter fault. */
    union {
        struct back_command command;
        struct back_blink blink;
        struct back_heartbeat heartbeat;
        struct back_fault fault;
    } value;
};

/* One validated event per receive, or a timeout/no-frame error. */
int can_comms_init(void);
int can_comms_receive(struct can_comms_rx *out, uint32_t timeout_ms);
int can_comms_send(uint32_t id, const uint8_t *data, uint8_t len);
int can_comms_send_on(uint8_t bus, const struct back_frame *frame);

/* Decode a received frame; can_comms_receive calls this internally. */
int can_comms_decode(const struct back_frame *frame, uint8_t bus,
                     struct can_comms_rx *out);

int back_decode_control(const struct back_frame *frame, struct back_command *out);
int back_decode_heartbeat(const struct back_frame *frame, struct back_heartbeat *out);
int back_decode_fault(const struct back_frame *frame, struct back_fault *out);
int back_build_heartbeat(uint8_t sequence, uint8_t state, uint8_t bus_health,
                          struct back_frame *out);
int back_build_fault(uint8_t sequence, uint8_t reason, struct back_frame *out);
int back_build_force(uint8_t sequence, int8_t force, struct back_frame *out);

/* One slot per CAN ID; zone logic uses this to ignore old/redundant commands. */
#define BACK_DEDUP_SLOTS 7u /* One duplicate-tracking slot per catalog ID. */
struct back_dedup_slot {
    uint32_t last_ms;
    uint8_t sequence;
    uint8_t seen_buses;
    bool valid;
};
struct back_dedup {
    struct back_dedup_slot slot[BACK_DEDUP_SLOTS];
};
enum back_rx_result {
    BACK_RX_INVALID = -1,
    BACK_RX_OLD = 0,
    BACK_RX_NEW = 1,
    BACK_RX_DUPLICATE = 2,
};
void back_dedup_reset(struct back_dedup *dedup);
enum back_rx_result back_dedup_observe(struct back_dedup *dedup,
                                        uint16_t id, uint8_t sequence,
                                        uint8_t bus, uint32_t now_ms);

#endif /* BACK_CAN_COMMS_H_ */