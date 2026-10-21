#ifndef CAN_COMMS_H_
#define CAN_COMMS_H_

#include <stdint.h>
#include <stdbool.h>

/** @brief Message Catalog IDs 
 * Nodes: N1 = Pi, N2 = Front STM32, N3 = Back STM32
 * **/
#define CAN_ID_CTRL      0x010  /* N1 -> N2, N3 (20 ms) */
#define CAN_ID_FAULT     0x020  /* Event-driven / System error */
#define CAN_ID_HB_N1     0x100  /* N1 Heartbeat (20 ms) */
#define CAN_ID_HB_N2     0x101  /* N2 Heartbeat (20 ms) */
#define CAN_ID_HB_N3     0x102  /* N3 Heartbeat (20 ms) */
#define CAN_ID_BLINK     0x200  /* Blinker command (500 ms / state change) */
#define CAN_ID_FORCE     0x300  /* N2 -> N1 Force feedback (50 ms) */

/** @brief Zone States **/
enum zone_state {
    ZONE_STATE_INIT        = 0,  /* Startup */
    ZONE_STATE_NORMAL      = 1,  /* Link alive, commands flowing */
    ZONE_STATE_LOCAL_FAIL  = 2,  /* Triggered by self-test button press */
    ZONE_STATE_LINK_LOST   = 3,  /* 3 missed heartbeats on both buses or CTRL timeout */
    ZONE_STATE_SYS_ERROR   = 4,  /* Out-of-range CTRL input or payload fault */ 
};

/** @brief Packed Structures for CAN ID Payload (8 Bytes) **/
/* 0x010 CTRL Payload */
struct __attribute__((packed)) ctrl_payload {
	uint8_t flags_seq;   /* High nibble = 4-bits of flags, Low nibble = 4-bit seq counter */
	int16_t steering;    /* Steering angle */
	uint8_t throttle;    /* 0 to 255 */
	uint8_t brake;       /* 0 to 255 */
	uint8_t buttons;     /* Button bitmask */
	uint8_t reserved;    /* Padding */
	uint8_t crc8;        /* CRC-8 byte, 8-bit Cyclic Redundancy Check */
};

/* 0x020 FAULT Payload */
struct __attribute__((packed)) fault_payload {
	uint8_t flags_seq;        /* High nibble = 4-bits of flags, Low nibble = 4-bit seq counter */
	uint8_t failing_zone_id;  /* 1 = N1, 2 = N2, 3 = N3 */
	uint8_t reason_code;      /* Fault reason identifier */
	uint8_t reserved[5];      /* Padding */
};

/* Heartbeat Payloads */
struct __attribute__((packed)) hb_payload {
	uint8_t flags_seq;   /* High nibble = 4-bits of flags, Low nibble = 4-bit seq counter */
	uint8_t zone_state;  /* enum zone_state */
	uint8_t bus_health;  /* Bit 0 = Bus A OK, Bit 1 = Bus B OK */
	uint8_t reserved[5]; /* Padding */
};

/* 0x200 BLINK Payload */
struct __attribute__((packed)) blink_payload {
	uint8_t flags_seq;   /* High nibble = 4-bits of flags, Low nibble = 4-bit seq counter */
	uint8_t mode;        /* 0=OFF, 1=LEFT, 2=RIGHT, 3=HAZARD */
	uint8_t phase;       /* Phase bit */
	uint8_t reserved[5]; /* Padding */
};

/* 0x300 FORCE Payload */
struct __attribute__((packed)) force_payload {
	uint8_t flags_seq;        /* High nibble = flags, Low nibble = 4-bit seq */
	int8_t  force;            /* Dummy force feedback counter */
	uint8_t reserved[6];      /* Padding */
};

/**
 * @brief Initialize dual CAN devices (can0 & can1), test point GPIOs, and RX filters.
 */
int can_comms_init(void);

/**
 * @brief Hot/parallel transmission helper: queues frame on CAN A and CAN B back-to-back.
 */
int can_comms_send(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Returns true if Node 3 is in any non-NORMAL state.
 */
bool can_comms_is_in_error(void);

/**
 * @brief Transition zone state, manage ERR test point output, and send FAULT frame.
 */
void can_comms_set_state(enum zone_state new_state, uint8_t reason_code);

#endif /* CAN_COMMS_H_ */