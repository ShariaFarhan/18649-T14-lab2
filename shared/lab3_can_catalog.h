
#ifndef LAB3_CAN_CATALOG_H
#define LAB3_CAN_CATALOG_H

#include <stddef.h>
#include <stdint.h>

// --------------------------------- IDs ----------------------------------- 
#define CAN_ID_CTRL         0x010u  //N1 to N2, N3 
#define CAN_ID_FAULT        0x020u  // any to all
#define CAN_ID_HB_N1        0x100u  // N1 to N2,N3 every 20 ms  
#define CAN_ID_HB_N2        0x101u  // N2 to N1, N3 20 ms  
#define CAN_ID_HB_N3        0x102u  // N3 to N1, N2 20 ms 
#define CAN_ID_BLINK        0x200u  // N1 to N2,N3
#define CAN_ID_FORCE        0x300u  // N2 to N1 50 ms 

#define CAN_FRAME_DLC       8u

/* ------------------------------- Byte 0 ---------------------------------- */
#define CAN_B0              0
#define CAN_SEQ_MASK        0x0Fu
#define CAN_FLAG_SYS_ERROR  0x1u    // sender is in the system error state 
#define CTRL_FLAG_UDP_STALE   0x1u  // wheel stream or drive heartbeat stale  
#define CTRL_FLAG_DUAL_ENABLE 0x8u  // cockpit asks N3 to arm the motors 

static inline uint8_t can_byte0(uint8_t seq, uint8_t flags)
{
    return (uint8_t)((seq & CAN_SEQ_MASK) | ((flags & 0x0Fu) << 4));
}
static inline uint8_t can_seq_of(const uint8_t *data)   { return data[0] & CAN_SEQ_MASK; }
static inline uint8_t can_flags_of(const uint8_t *data) { return (uint8_t)(data[0] >> 4); }

//duplicate filter with seq counters
#define CAN_DEDUP_RESET_MS  100u
static inline int can_seq_is_new(uint8_t last_seq, uint8_t seq)
{
    uint8_t d = (uint8_t)((seq - last_seq) & CAN_SEQ_MASK);
    return d >= 1u && d <= 7u;
}

/* ------------------------------- Zones ----------------------------------- */
#define ZONE_COCKPIT        1u
#define ZONE_STEERING       2u
#define ZONE_DRIVETRAIN     3u

/* ------------------------------ 0x010 CTRL ------------------------------- */
#define CTRL_STEER_LO       1       // int16 little-endian 
#define CTRL_STEER_HI       2
#define CTRL_THROTTLE       3       // uint8 
#define CTRL_BRAKE          4       // uint8 
#define CTRL_BUTTONS        5       // uint8 bitmask, below     
#define CTRL_RESERVED       6       // 0                               
#define CTRL_CRC            7       // CRC-8

//valid ranges 
#define CTRL_STEER_MIN      (-1000) // full left
#define CTRL_STEER_CENTER   0
#define CTRL_STEER_MAX      1000    // full right   
#define CTRL_PEDAL_RELEASED 0       // percent   
#define CTRL_PEDAL_PRESSED  100

#define CTRL_BTN_LSB        0x01u   // left blinker button
#define CTRL_BTN_RSB        0x02u   // right blinker button 
#define CTRL_BTN_TEST       0x04u   // wheel test button held

//crc
static inline uint8_t can_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x1Du) : (uint8_t)(crc << 1);
    }
    return (uint8_t)(crc ^ 0xFFu);
}

/* ------------------------------ 0x020 FAULT ------------------------------ */
#define FAULT_ZONE          1       /* failing zone id (ZONE_*)                 */
#define FAULT_REASON        2       /* FAULT_R_*                                */
#define FAULT_R_SELF_TEST   1u      /* test button single press                 */
#define FAULT_R_BAD_INPUT   2u      /* out-of-range or CRC-bad CTRL             */
#define FAULT_R_UDP_LOST    3u      /* cockpit: configured UDP timeout expired */
#define FAULT_R_ZONE_LOST   4u      /* 3 heartbeats missed on both buses        */
#define FAULT_R_ISOLATED    5u      /* node hears nobody on either bus          */
#define FAULT_R_OUTPUT      6u      /* actuator / driver failure                */
#define FAULT_R_SHUTDOWN    7u      /* cockpit program exiting                  */

/* --------------------------- 0x100..0x102 HB ----------------------------- */
#define HB_STATE            1       /* ZONE_STATE_*                             */
#define HB_BUS              2       /* HB_BUS_* bits                            */
#define HB_REASON           3       /* sender's system_reason (Lab 2 enum)      */
#define HB_DRIVE_FLAGS      4       /* N3: armed, brake, fault, reason << 4     */
#define HB_DUTY             5       /* N3: motor duty percent                   */

#define ZONE_STATE_NORMAL       0u
#define ZONE_STATE_SYS_ERROR    1u  /* following the system error state         */
#define ZONE_STATE_LOCAL_FAULT  2u  /* THIS zone has an active fault (bad input,
                                       output fault). Keeps the others in error
                                       until it clears. A failed zone (test
                                       button) sends no heartbeat at all.       */

#define HB_BUS_A_DEGRADED   0x01u
#define HB_BUS_B_DEGRADED   0x02u

/* ------------------------------ 0x200 BLINK ------------------------------ */
#define BLINK_MODE          1       /* BLINK_MODE_*                             */
#define BLINK_PHASE         2       /* 1 = lamps on now, 0 = lamps off now      */
#define BLINK_MODE_OFF      0u
#define BLINK_MODE_LEFT     1u
#define BLINK_MODE_RIGHT    2u
#define BLINK_MODE_HAZARD   3u
/* Turn: 500 ms on / 500 ms off (1 Hz). Hazard: 250 / 250 (2 Hz).
 * Receivers set their lamps from (mode, phase) on reception. They never run
 * their own blink timer while BLINK frames keep arriving. */

/* ------------------------------ 0x300 FORCE ------------------------------ */
#define FORCE_VALUE         1       /* int8, -100..100 (dummy counter in Lab 3) */
#define FORCE_MIN           (-100)
#define FORCE_MAX           100

#endif
