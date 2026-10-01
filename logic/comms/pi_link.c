

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>
#include <string.h>

#include "pi_link.h"
#include "link_parser.h"
#include "link_watchdog.h"
#include "test_points.h"



//-----------------------CONFIG----------------------------------------
#define PI_UART_NODE DT_CHOSEN(pilink_uart)
BUILD_ASSERT(DT_NODE_HAS_STATUS(PI_UART_NODE, okay),
             "pilink,uart chosen node missing/disabled");
static const struct device *const pi_uart_dev = DEVICE_DT_GET(PI_UART_NODE);

//timing params
#define HEARTBEAT_PERIOD_MS   20   
#ifndef LINK_TIMEOUT_MS
#define LINK_TIMEOUT_MS       150
#endif
                                 
#define MID_FRAME_TIMEOUT_MS  20   

//-------------------------ISR+ ring buffer logic------------------------------------------

RING_BUF_DECLARE(rx_ring, 128);
K_SEM_DEFINE(rx_sem, 0, 1);

static void uart_isr(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    // ISR does the absolute minimum: drain the UART FIFO into the ring
    // buffer. No parsing, no printk, no blocking, nothing that allocs
    
    if (!uart_irq_update(dev)) {
        return;
    }
    if (!uart_irq_rx_ready(dev)) {
        return;
    }

    uint8_t buf[32];
    int n;
    bool got_any = false;
    while ((n = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
        //if the ring buffer is momentarily full, writes less than n
        //causing crc to fail and resend
        ring_buf_put(&rx_ring, buf, (uint32_t)n);
        got_any = true;
    }
    if (got_any) {
        k_sem_give(&rx_sem);
    }
}


//----------------------------shared state---------------------------------------
static struct k_mutex state_lock;

static K_MUTEX_DEFINE(dispatch_lock); /* Serialize command and timeout callbacks. */
static atomic_t link_alive;
static int64_t last_valid_cmd_uptime_ms; /* guarded by dispatch_lock */
static struct link_watchdog command_watchdog; /* Same dispatch_lock; sole UART deadline owner. */
static atomic_t last_timeout_age_ms;

static enum pi_link_zone_state zone_override = ZONE_STATE_INIT; //mutex-guarded 

static atomic_t rx_valid_count;
static atomic_t rx_crc_fail_count;
static atomic_t rx_seq_gap_count;
static bool have_last_seq;
static uint8_t last_seq;
//------------------------------------default callbacks----------------------------------------- 

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
    //TODO: wire up??
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




//------------------------------------parser thread-------------------------------------------
#define PARSER_STACK_SIZE 1024
#define PARSER_PRIORITY   5

static void parser_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    link_parser_t parser;
    link_parser_init(&parser);

    while (1) {
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

            //LP_VALID from here down
            if (payload_len != sizeof(cmd_payload_t)) {
                // Right CRC, wrong shape reject rather than reinterpret-cast garbage onto struct
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



//--------------------------heartbeat thread----------------------------------------
#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_PRIORITY   6  //one below the parser




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

    //simple blocking TX since it doesn't need to be interrupt driven like RX does
    for (size_t i = 0; i < n; i++) {
        uart_poll_out(pi_uart_dev, frame[i]);
    }
}



static void heartbeat_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

    int64_t next_wake = k_uptime_get();

    while (1) {
       
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
            //fell behind so resync
            next_wake = now;
        }
    }
}


K_THREAD_DEFINE(heartbeat_tid, HEARTBEAT_STACK_SIZE, heartbeat_thread_fn, NULL, NULL, NULL,
                 HEARTBEAT_PRIORITY, 0, K_TICKS_FOREVER);


//-------------------------public API------------------------------------------------------



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
    atomic_set(&link_alive, false); // No valid control source at startup

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
