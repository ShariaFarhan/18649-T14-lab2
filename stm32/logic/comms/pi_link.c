/*
 * pi_link.c -- STM32 side of the Pi <-> STM32 serial link.
 *
 * One file for the whole link (it replaces link_parser.c and the old
 * pi_link.c; the wire format and watchdog are in pi_link.h):
 *
 *   UART RX interrupt -> ring buffer -> parser thread -> application callbacks
 *   heartbeat thread  -> link watchdog check, then one status frame every 20 ms
 *
 * Nothing here decides what the car does. The application (for example
 * wheel_dual_main.c) overrides the weak callbacks below.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include "pi_link.h"
#include "test_points.h"

/* ------------------------------ CONFIG ---------------------------------- */

#define PI_UART_NODE DT_CHOSEN(pilink_uart)
BUILD_ASSERT(DT_NODE_HAS_STATUS(PI_UART_NODE, okay),
             "pilink,uart chosen node missing/disabled");
static const struct device *const pi_uart_dev = DEVICE_DT_GET(PI_UART_NODE);

#define HEARTBEAT_PERIOD_MS   20
#ifndef LINK_TIMEOUT_MS
#define LINK_TIMEOUT_MS       150   /* the build normally sets this (75 for dual drive) */
#endif
#define MID_FRAME_TIMEOUT_MS  20    /* a frame that stalls this long is dropped */

#define LINK_FRAME_MAX_SIZE   (2 + 1 + 1 + MAX_PAYLOAD + 2)

/* ------------------------------- CRC ------------------------------------ */

/* CRC-16, reflected polynomial 0x8408. The link uses seed 0xFFFF and no final
 * XOR. Must match crc16() in pi/link_forwarder.c. */
static uint16_t link_crc16_ccitt(uint16_t seed, const uint8_t *data, size_t len)
{
    uint16_t crc = seed;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408)
                             : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/* --------------------------- FRAME ENCODER ------------------------------ */

/* Builds START0 START1 LEN SEQ PAYLOAD CRC_LO CRC_HI into out_buf.
 * Returns the number of bytes written, or 0 if it does not fit. */
static size_t link_frame_encode(uint8_t *out_buf, size_t out_buf_cap,
                                uint8_t seq, const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > MAX_PAYLOAD) {
        return 0;
    }
    size_t needed = 2 + 1 + 1 + (size_t)payload_len + 2;
    if (out_buf_cap < needed) {
        return 0;
    }

    uint8_t crc_region[1 + 1 + MAX_PAYLOAD]; /* LEN | SEQ | PAYLOAD */
    size_t idx = 0;

    out_buf[idx++] = FRAME_START0;
    out_buf[idx++] = FRAME_START1;
    out_buf[idx++] = payload_len;
    out_buf[idx++] = seq;
    memcpy(&out_buf[idx], payload, payload_len);
    idx += payload_len;

    crc_region[0] = payload_len;
    crc_region[1] = seq;
    memcpy(&crc_region[2], payload, payload_len);
    uint16_t crc = link_crc16_ccitt(0xFFFF, crc_region, 2 + (size_t)payload_len);

    out_buf[idx++] = (uint8_t)(crc & 0xFF);
    out_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    return idx;
}

/* ---------------------------- FRAME PARSER ------------------------------ */

typedef enum {
    LP_SCAN_START0,
    LP_SCAN_START1,
    LP_LEN,
    LP_SEQ,
    LP_PAYLOAD,
    LP_CRC0,
    LP_CRC1,
} link_parse_state_t;

typedef struct {
    link_parse_state_t state;
    uint8_t  payload_buf[MAX_PAYLOAD];
    uint8_t  len;
    uint8_t  seq;
    uint8_t  payload_idx;
    uint16_t crc_rx;
} link_parser_t;

/* LP_INCOMPLETE: byte consumed, no frame finished (also junk while resyncing,
 *                and a LEN above MAX_PAYLOAD, which cannot be a real header).
 * LP_VALID:      a full frame passed the CRC; the out_* parameters are filled.
 * LP_REJECTED:   a full frame failed the CRC. */
typedef enum {
    LP_INCOMPLETE,
    LP_VALID,
    LP_REJECTED,
} link_frame_result_t;

static void link_parser_init(link_parser_t *p)
{
    memset(p, 0, sizeof(*p));
    p->state = LP_SCAN_START0;
}

/* Drop any partial frame and go back to hunting for START0. */
static void link_parser_reset(link_parser_t *p)
{
    p->state = LP_SCAN_START0;
    p->payload_idx = 0;
}

/* True while a frame is partly received, so the caller knows to time it out. */
static bool link_parser_in_progress(const link_parser_t *p)
{
    return p->state != LP_SCAN_START0;
}

/* Feed one received byte. The parser resets itself after every finished frame. */
static link_frame_result_t link_parser_feed_byte(link_parser_t *p, uint8_t byte,
                                                 uint8_t *out_seq, uint8_t *out_payload,
                                                 uint8_t *out_payload_len)
{
    switch (p->state) {
    case LP_SCAN_START0:
        if (byte == FRAME_START0) {
            p->state = LP_SCAN_START1;
        }
        return LP_INCOMPLETE;

    case LP_SCAN_START1:
        if (byte == FRAME_START1) {
            p->state = LP_LEN;
        } else {
            /* False start. Keep this byte if it is itself a START0. */
            p->state = (byte == FRAME_START0) ? LP_SCAN_START1 : LP_SCAN_START0;
        }
        return LP_INCOMPLETE;

    case LP_LEN:
        if (byte > MAX_PAYLOAD) {
            p->state = LP_SCAN_START0;
            return LP_INCOMPLETE;
        }
        p->len = byte;
        p->payload_idx = 0;
        p->state = LP_SEQ;
        return LP_INCOMPLETE;

    case LP_SEQ:
        p->seq = byte;
        p->state = (p->len > 0) ? LP_PAYLOAD : LP_CRC0;
        return LP_INCOMPLETE;

    case LP_PAYLOAD:
        p->payload_buf[p->payload_idx++] = byte;
        if (p->payload_idx == p->len) {
            p->state = LP_CRC0;
        }
        return LP_INCOMPLETE;

    case LP_CRC0:
        p->crc_rx = byte;
        p->state = LP_CRC1;
        return LP_INCOMPLETE;

    case LP_CRC1: {
        p->crc_rx |= ((uint16_t)byte << 8);

        uint8_t crc_region[1 + 1 + MAX_PAYLOAD];
        crc_region[0] = p->len;
        crc_region[1] = p->seq;
        memcpy(&crc_region[2], p->payload_buf, p->len);
        uint16_t crc_calc = link_crc16_ccitt(0xFFFF, crc_region, 2 + (size_t)p->len);

        bool valid = (crc_calc == p->crc_rx);

        if (valid) {
            *out_seq = p->seq;
            *out_payload_len = p->len;
            memcpy(out_payload, p->payload_buf, p->len);
        }
        /* No NACK and no retransmit on this link. */

        p->state = LP_SCAN_START0;
        return valid ? LP_VALID : LP_REJECTED;
    }
    }

    /* Unreachable; keeps compilers that do not see the switch as exhaustive quiet. */
    p->state = LP_SCAN_START0;
    return LP_INCOMPLETE;
}

/* ---------------------- UART INTERRUPT + RING BUFFER -------------------- */

RING_BUF_DECLARE(rx_ring, 128);
K_SEM_DEFINE(rx_sem, 0, 1);

static void uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    /* The ISR only moves bytes from the UART FIFO into the ring buffer.
     * No parsing, no printing, no blocking, no allocation. */
    uart_irq_update(dev);
    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    uint8_t buf[32];
    int n;
    bool got_any = false;
    while ((n = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
        /* If the ring buffer is full, bytes are lost and that frame fails its CRC. */
        ring_buf_put(&rx_ring, buf, (uint32_t)n);
        got_any = true;
    }
    if (got_any) {
        k_sem_give(&rx_sem);
    }
}

/* ---------------------------- SHARED STATE ------------------------------ */

static struct k_mutex state_lock;

static K_MUTEX_DEFINE(dispatch_lock); /* Serialize command and timeout callbacks. */
static atomic_t link_alive;
static int64_t last_valid_cmd_uptime_ms; /* guarded by dispatch_lock */
static struct link_watchdog command_watchdog; /* Same dispatch_lock; sole UART deadline owner. */
static atomic_t last_timeout_age_ms;

static enum pi_link_zone_state zone_override = ZONE_STATE_INIT; /* guarded by state_lock */

static atomic_t rx_valid_count;
static atomic_t rx_crc_fail_count;
static atomic_t rx_seq_gap_count;
static bool have_last_seq;
static uint8_t last_seq;

/* --------------------------- DEFAULT CALLBACKS -------------------------- */

__weak void pi_link_on_command_received(const cmd_payload_t *cmd, bool plausible)
{
    printk("[pi_link] cmd steer=%d thr=%d brk=%d btn=0x%04x flags=0x%02x plausible=%d\n",
           cmd->steering_raw, cmd->throttle_raw, cmd->brake_raw,
           cmd->buttons, cmd->flags, plausible);
}

__weak bool pi_link_command_is_fresh(const cmd_payload_t *cmd, bool plausible)
{
    return plausible && !(cmd->flags & FLAG_UDP_STALE);
}

__weak void pi_link_on_invalid_frame(void) { }

__weak void pi_link_on_link_lost(void)
{
    printk("[pi_link] LINK LOST (no valid frame for %dms)\n", LINK_TIMEOUT_MS);
}

__weak void pi_link_on_link_restored(void)
{
    printk("[pi_link] link restored\n");
}

__weak void pi_link_get_currents(uint16_t *motor_l, uint16_t *motor_r, uint16_t *servo)
{
    *motor_l = 0;
    *motor_r = 0;
    *servo = 0;
}

__weak void pi_link_customize_heartbeat(hb_payload_t *hb)
{
    ARG_UNUSED(hb);
}

void pi_link_set_zone_state_override(enum pi_link_zone_state state)
{
    k_mutex_lock(&state_lock, K_FOREVER);
    zone_override = state;
    k_mutex_unlock(&state_lock);
}

/* ---------------------------- PARSER THREAD ----------------------------- */

#define PARSER_STACK_SIZE 1024
#define PARSER_PRIORITY   5

static void parser_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    link_parser_t parser;
    link_parser_init(&parser);

    while (1) {
        /* Idle: wait for data forever. Mid-frame: give up after the timeout. */
        k_timeout_t timeout = link_parser_in_progress(&parser)
                                   ? K_MSEC(MID_FRAME_TIMEOUT_MS)
                                   : K_FOREVER;
        int rc = k_sem_take(&rx_sem, timeout);
        if (rc != 0) {
            link_parser_reset(&parser);
            continue;
        }

        uint8_t byte;
        while (ring_buf_get(&rx_ring, &byte, 1) == 1) {
            uint8_t seq, payload[MAX_PAYLOAD], payload_len;
            link_frame_result_t result =
                link_parser_feed_byte(&parser, byte, &seq, payload, &payload_len);

            if (result == LP_INCOMPLETE) {
                continue;
            }
            if (result == LP_REJECTED) {
                atomic_inc(&rx_crc_fail_count);
                k_mutex_lock(&dispatch_lock, K_FOREVER);
                pi_link_on_invalid_frame();
                k_mutex_unlock(&dispatch_lock);
                continue;
            }

            /* LP_VALID from here down. */
            if (payload_len != sizeof(cmd_payload_t)) {
                /* Good CRC but the wrong size: reject it instead of reading
                 * it as a command. */
                atomic_inc(&rx_crc_fail_count);
                k_mutex_lock(&dispatch_lock, K_FOREVER);
                pi_link_on_invalid_frame();
                k_mutex_unlock(&dispatch_lock);
                continue;
            }

            cmd_payload_t cmd;
            memcpy(&cmd, payload, sizeof(cmd));
            bool plausible =
                cmd.steering_raw >= RAW_STEER_MIN && cmd.steering_raw <= RAW_STEER_MAX &&
                cmd.throttle_raw >= RAW_PEDAL_MIN && cmd.throttle_raw <= RAW_PEDAL_MAX &&
                cmd.brake_raw >= RAW_PEDAL_MIN && cmd.brake_raw <= RAW_PEDAL_MAX;

            k_mutex_lock(&dispatch_lock, K_FOREVER);
            uint8_t delta = (uint8_t)(seq - last_seq);
            if (have_last_seq && atomic_get(&link_alive) && (delta == 0 || delta >= 128)) {
                atomic_inc(&rx_seq_gap_count);
                k_mutex_unlock(&dispatch_lock);
                continue; /* Duplicate/old packets cannot keep control alive. */
            }
            if (have_last_seq && delta != 1) atomic_inc(&rx_seq_gap_count);
            last_seq = seq;
            have_last_seq = true;
            atomic_inc(&rx_valid_count);
            if (test_points_command_received()) {
                pi_link_on_invalid_frame();
                k_mutex_unlock(&dispatch_lock);
                continue;
            }
            if (pi_link_command_is_fresh(&cmd, plausible)) {
                last_valid_cmd_uptime_ms = k_uptime_get();
                link_watchdog_feed(&command_watchdog,(uint64_t)last_valid_cmd_uptime_ms);
                bool restored = !atomic_get(&link_alive);
                atomic_set(&link_alive, true);
                if (restored) pi_link_on_link_restored();
            }
            pi_link_on_command_received(&cmd, plausible);
            k_mutex_unlock(&dispatch_lock);
        }
    }
}

K_THREAD_DEFINE(parser_tid, PARSER_STACK_SIZE, parser_thread_fn, NULL, NULL, NULL,
                 PARSER_PRIORITY, 0, K_TICKS_FOREVER);

/* --------------------------- HEARTBEAT THREAD --------------------------- */

#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_PRIORITY   6  /* one below the parser */

static void send_heartbeat(void)
{
    hb_payload_t hb = {0};

    k_mutex_lock(&state_lock, K_FOREVER);
    if (zone_override != ZONE_STATE_INIT) {
        hb.zone_state = (uint8_t)zone_override;
    } else {
        hb.zone_state = (uint8_t)(atomic_get(&link_alive) ? ZONE_STATE_NORMAL : ZONE_STATE_LINK_LOST);
    }
    k_mutex_unlock(&state_lock);

    hb.bus_degraded = 0;

    pi_link_get_currents(&hb.current_motor_l, &hb.current_motor_r, &hb.current_servo);

    pi_link_customize_heartbeat(&hb);

    static uint8_t hb_seq;
    uint8_t frame[LINK_FRAME_MAX_SIZE];
    size_t n = link_frame_encode(frame, sizeof(frame), hb_seq++, (const uint8_t *)&hb, sizeof(hb));

    /* Blocking transmit is fine here: this is the lower-priority thread. */
    for (size_t i = 0; i < n; i++) {
        uart_poll_out(pi_uart_dev, frame[i]);
    }
}

static void heartbeat_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    int64_t next_wake = k_uptime_get();

    while (1) {
        /* Link watchdog: checked once per heartbeat tick. */
        k_mutex_lock(&dispatch_lock, K_FOREVER);
        int64_t now_check = k_uptime_get();
        int64_t age = now_check - last_valid_cmd_uptime_ms;
        if (link_watchdog_poll(&command_watchdog,(uint64_t)now_check)) {
            atomic_set(&link_alive, false);
            atomic_set(&last_timeout_age_ms, (atomic_val_t)age);
            pi_link_on_link_lost();
        }
        k_mutex_unlock(&dispatch_lock);

        send_heartbeat();

        next_wake += HEARTBEAT_PERIOD_MS;
        int64_t now = k_uptime_get();
        if (next_wake > now) {
            k_sleep(K_MSEC(next_wake - now));
        } else {
            next_wake = now; /* fell behind: resync instead of bursting */
        }
    }
}

K_THREAD_DEFINE(heartbeat_tid, HEARTBEAT_STACK_SIZE, heartbeat_thread_fn, NULL, NULL, NULL,
                 HEARTBEAT_PRIORITY, 0, K_TICKS_FOREVER);

/* ------------------------------ PUBLIC API ------------------------------ */

void pi_link_init(void)
{
    k_mutex_init(&state_lock);

    if (!device_is_ready(pi_uart_dev)) {
        printk("[pi_link] FATAL: pi-link UART device not ready -- check "
               "boards/<board>.overlay\n");
        return;
    }

    if (test_points_init()) {
        printk("[pi_link] FATAL: scope test-point GPIO initialization failed\n");
        return;
    }

    uart_irq_rx_disable(pi_uart_dev);
    uart_irq_tx_disable(pi_uart_dev);
    uart_irq_callback_user_data_set(pi_uart_dev, uart_isr, NULL);
    uart_irq_rx_enable(pi_uart_dev);

    last_valid_cmd_uptime_ms = k_uptime_get();
    link_watchdog_init(&command_watchdog,LINK_TIMEOUT_MS);
    atomic_set(&link_alive, false); /* No valid control source at startup. */

    k_thread_start(parser_tid);
    k_thread_start(heartbeat_tid);

    printk("[pi_link] initialized, waiting for first command frame\n");
}

bool pi_link_is_alive(void)
{
    return atomic_get(&link_alive) != 0;
}

uint32_t pi_link_rx_valid_count(void)     { return (uint32_t)atomic_get(&rx_valid_count); }
uint32_t pi_link_rx_crc_fail_count(void)  { return (uint32_t)atomic_get(&rx_crc_fail_count); }
uint32_t pi_link_rx_seq_gap_count(void)   { return (uint32_t)atomic_get(&rx_seq_gap_count); }

uint32_t pi_link_last_timeout_age_ms(void) { return (uint32_t)atomic_get(&last_timeout_age_ms); }
