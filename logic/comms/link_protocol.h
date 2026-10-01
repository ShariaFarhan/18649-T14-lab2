
 // wire format shared between the Pi (link_forwarder.c) and pi_link

 // Frame on the wire:
 // START0 | START1 | LEN | SEQ | PAYLOAD (LEN bytes) | CRC_LO | CRC_HI
 

#ifndef LINK_PROTOCOL_H
#define LINK_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

// Framing constants

#define FRAME_START0    0x7E
#define FRAME_START1    0xA5
#define MAX_PAYLOAD     32



//PAYLOADS must be packed to avoid bit stuffing which could lead to bugs 
#pragma pack(push, 1)

//Pi to STM32, sent every <=50ms (CMD_PERIOD_MS on the Pi side) 

typedef struct {
#ifdef LAB2_RAW_WHEEL_LINK
    int32_t  steering_raw;
    int32_t  throttle_raw;
    int32_t  brake_raw;
#else
    int16_t  steering_raw;
    int16_t  throttle_raw;
    int16_t  brake_raw;
#endif
    uint16_t buttons;      /* bitmask, see extract_buttons_mask() on the Pi */
    uint8_t  flags;        /* bit0 = FLAG_UDP_STALE */
#ifdef LAB2_WHEEL_DUAL_DRIVE
    uint8_t test_counter;  /* Debounced wheel-button press counter, modulo 32. */
    uint8_t test_flags;    /* Bit 0 establishes a new sender's counter baseline. */
#endif
} cmd_payload_t;

/* STM32 -> Pi, sent every 20ms +-10% (this project's heartbeat thread) */
typedef struct {
    uint8_t  zone_state;       /* see enum pi_link_zone_state below */
    uint8_t  bus_degraded;     /* placeholder -- no CAN bus in this lab;
                                 * kept for wire-format continuity with
                                 * later labs that add one. Always 0 here. */
    uint16_t current_motor_l;  /* Absolute current in mA; UINT16_MAX = unavailable. */
    uint16_t current_motor_r;
    uint16_t current_servo;
#ifdef LAB2_WHEEL_RIGHT_DRIVE
    int32_t right_target_mrpm;
    int32_t right_actual_mrpm;
    uint16_t right_duty_percent;
    uint8_t right_flags; /* armed, braking, fault, startup assist */
#endif
#ifdef LAB2_WHEEL_DUAL_DRIVE
    uint16_t dual_steering_raw;
    uint16_t dual_steering_packed;
    uint16_t dual_servo_pulse_us;
    int32_t dual_target_mrpm;
    int32_t dual_left_mrpm;
    int32_t dual_right_mrpm;
    uint16_t dual_duty_percent;
    uint8_t dual_flags; /* armed/brake/fault/boost, then reason in high nibble */
    uint8_t test_counter_ack;
    uint8_t system_reason;
#endif
} hb_payload_t;

#pragma pack(pop)

#define FLAG_UDP_STALE  0x01
#define FLAG_RIGHT_ENABLE 0x02 /* Only the separate right-drive test profile accepts this. */
#define FLAG_DUAL_ENABLE 0x08 /* Only accepted by dual-motor firmware. */


enum pi_link_zone_state {
    ZONE_STATE_INIT        = 0,  // startup
    ZONE_STATE_NORMAL      = 1,  // link alive, commands flowing 
    ZONE_STATE_LOCAL_FAIL  = 2,  // self-test button path 
    ZONE_STATE_LINK_LOST   = 3,  // this module's own 150ms miss, watchdog tripped, etc.
    ZONE_STATE_SYS_ERROR   = 4,  // reserved for malformed/out-of-range
};


//Logitech wheel bounds
//TODO: replace with our vals


#ifdef LAB2_RAW_WHEEL_LINK
#define RAW_STEER_MIN 0
#define RAW_STEER_MAX 65535
#define RAW_PEDAL_MIN 0
#define RAW_PEDAL_MAX 65535
#else
#define RAW_STEER_MIN   ((int16_t)-32767)
#define RAW_STEER_MAX   ((int16_t)32766)
#define RAW_PEDAL_MIN   ((int16_t)0)
#define RAW_PEDAL_MAX   ((int16_t)32767)

#endif

#endif /* LINK_PROTOCOL_H */
