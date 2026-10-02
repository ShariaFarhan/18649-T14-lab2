/*
 * pi_link.h -- Pi <-> STM32 serial link: wire format, link watchdog and API.
 *
 * This header and pi_link.c replace six files:
 *   link_protocol.h  -> "Wire format" below
 *   link_watchdog.h  -> "Link watchdog" below
 *   pi_link.h        -> "API" below
 *   link_parser.h, link_parser.c, pi_link.c -> pi_link.c
 *
 * It has no Zephyr dependency, so host tests can include it directly.
 *
 * Frame on the wire, both directions:
 *   START0 | START1 | LEN | SEQ | PAYLOAD (LEN bytes) | CRC_LO | CRC_HI
 * The CRC covers LEN, SEQ and PAYLOAD. The peer is pi/link_forwarder.c.
 */

#ifndef PI_LINK_H
#define PI_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ----------------------------- Wire format ------------------------------ */

#define FRAME_START0    0x7E
#define FRAME_START1    0xA5
#define MAX_PAYLOAD     32

/* Payloads are packed so there is no padding between fields. */
#pragma pack(push, 1)

/* Pi -> STM32. Sent on every wheel update and at least every 20 ms. */
typedef struct {
#ifdef LAB2_RAW_WHEEL_LINK
    int32_t  steering_raw;     /* 0 left, 33151 center, 65535 right */
    int32_t  throttle_raw;     /* 65535 released, 0 fully pressed   */
    int32_t  brake_raw;        /* 65535 released, 0 fully pressed   */
#else
    int16_t  steering_raw;
    int16_t  throttle_raw;
    int16_t  brake_raw;
#endif
    uint16_t buttons;          /* bit 8 = RSB, bit 9 = LSB */
    uint8_t  flags;            /* FLAG_* below */
#ifdef LAB2_WHEEL_DUAL_DRIVE
    uint8_t  test_counter;     /* Debounced self-test press counter, modulo 32. */
    uint8_t  test_flags;       /* Bit 0 establishes a new sender's counter baseline. */
#endif
} cmd_payload_t;

/* STM32 -> Pi. Sent every 20 ms +-10 % by the heartbeat thread. */
typedef struct {
    uint8_t  zone_state;       /* enum pi_link_zone_state */
    uint8_t  bus_degraded;     /* No CAN bus in this lab. The dual-drive build
                                * writes its format marker (0xBA) here and the
                                * Pi forwarder checks it. */
    uint16_t current_motor_l;  /* Absolute current in mA; UINT16_MAX = unavailable. */
    uint16_t current_motor_r;
    uint16_t current_servo;
#ifdef LAB2_WHEEL_RIGHT_DRIVE
    int32_t  right_target_mrpm;
    int32_t  right_actual_mrpm;
    uint16_t right_duty_percent;
    uint8_t  right_flags;      /* armed, braking, fault, startup assist */
#endif
#ifdef LAB2_WHEEL_DUAL_DRIVE
    uint16_t dual_steering_raw;
    uint16_t dual_steering_packed;
    uint16_t dual_servo_pulse_us;
    int32_t  dual_target_mrpm;
    int32_t  dual_left_mrpm;
    int32_t  dual_right_mrpm;
    uint16_t dual_duty_percent;
    uint8_t  dual_flags;       /* armed/brake/fault/boost, then reason in high nibble */
    uint8_t  test_counter_ack;
    uint8_t  system_reason;
#endif
} hb_payload_t;

#pragma pack(pop)

/* The integrated build must match pi/link_forwarder.c byte for byte. */
#if defined(LAB2_RAW_WHEEL_LINK) && defined(LAB2_WHEEL_DUAL_DRIVE)
_Static_assert(sizeof(cmd_payload_t) == 17, "command payload must be 17 bytes");
_Static_assert(sizeof(hb_payload_t) == 31, "heartbeat payload must be 31 bytes");
#endif

#define FLAG_UDP_STALE    0x01 /* The Pi has no fresh wheel data or no fresh heartbeat. */
#define FLAG_RIGHT_ENABLE 0x02 /* Only the separate right-drive test profile accepts this. */
#define FLAG_DUAL_ENABLE  0x08 /* Only accepted by dual-motor firmware. */

enum pi_link_zone_state {
    ZONE_STATE_INIT        = 0,  /* startup */
    ZONE_STATE_NORMAL      = 1,  /* link alive, commands flowing */
    ZONE_STATE_LOCAL_FAIL  = 2,  /* self-test button */
    ZONE_STATE_LINK_LOST   = 3,  /* no fresh command within LINK_TIMEOUT_MS */
    ZONE_STATE_SYS_ERROR   = 4,  /* malformed or out-of-range input, output fault */
};

/* Accepted command ranges. Anything outside is reported as not plausible. */
#ifdef LAB2_RAW_WHEEL_LINK
#define RAW_STEER_MIN   0
#define RAW_STEER_MAX   65535
#define RAW_PEDAL_MIN   0
#define RAW_PEDAL_MAX   65535
#else
#define RAW_STEER_MIN   ((int16_t)-32767)
#define RAW_STEER_MAX   ((int16_t)32766)
#define RAW_PEDAL_MIN   ((int16_t)0)
#define RAW_PEDAL_MAX   ((int16_t)32767)
#endif

/* ---------------------------- Link watchdog ----------------------------- */

/* Passive deadline tracking. Polling a healthy link never changes actuator state. */
struct link_watchdog { uint64_t last_valid_ms; uint32_t timeout_ms; bool seen,reported; };
static inline void link_watchdog_init(struct link_watchdog *w,uint32_t timeout_ms)
{ *w=(struct link_watchdog){.timeout_ms=timeout_ms}; }
static inline void link_watchdog_feed(struct link_watchdog *w,uint64_t now)
{ w->last_valid_ms=now;w->seen=true;w->reported=false; }
static inline bool link_watchdog_poll(struct link_watchdog *w,uint64_t now)
{
    if(!w->seen || w->reported || now-w->last_valid_ms<w->timeout_ms)return false;
    w->reported=true;return true; /* One transition for a confirmed missed deadline. */
}

/* --------------------------------- API ---------------------------------- */

/* Starts the UART RX interrupt, the parser thread and the heartbeat thread.
 * Call once from main(). */
void pi_link_init(void);

/* Callbacks. pi_link.c has weak defaults; the application overrides them.
 * All of them except pi_link_get_currents() and pi_link_customize_heartbeat()
 * are called with the link's dispatch lock held, so they never overlap. */
void pi_link_on_command_received(const cmd_payload_t *cmd, bool plausible);
bool pi_link_command_is_fresh(const cmd_payload_t *cmd, bool plausible);
void pi_link_on_invalid_frame(void);
void pi_link_on_link_lost(void);
void pi_link_on_link_restored(void);
void pi_link_get_currents(uint16_t *motor_l, uint16_t *motor_r, uint16_t *servo);
void pi_link_customize_heartbeat(hb_payload_t *hb);

void pi_link_set_zone_state_override(enum pi_link_zone_state state);

/* Diagnostics. */
bool pi_link_is_alive(void);
uint32_t pi_link_rx_valid_count(void);
uint32_t pi_link_rx_crc_fail_count(void);
uint32_t pi_link_rx_seq_gap_count(void);
uint32_t pi_link_last_timeout_age_ms(void);

#endif /* PI_LINK_H */
