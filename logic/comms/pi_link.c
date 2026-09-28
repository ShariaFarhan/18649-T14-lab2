

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include "pi_link.h"
#include "link_parser.h"



//-----------------------CONFIG----------------------------------------
#define PI_UART_NODE DT_CHOSEN(pilink_uart)
BUILD_ASSERT(DT_NODE_HAS_STATUS(PI_UART_NODE, okay),
             "pilink,uart chosen node missing/disabled");
static const struct device *const pi_uart_dev = DEVICE_DT_GET(PI_UART_NODE);

//TODO: CMD_RX scope test point 
#define TP_NODE DT_PATH(zephyr_user)
#if DT_NODE_EXISTS(TP_NODE) && DT_NODE_HAS_PROP(TP_NODE, cmd_rx_test_gpios)
#define HAVE_CMD_RX_TP 1
static const struct gpio_dt_spec cmd_rx_tp = GPIO_DT_SPEC_GET(TP_NODE, cmd_rx_test_gpios);
#else
#define HAVE_CMD_RX_TP 0
#endif

//timing params
#define HEARTBEAT_PERIOD_MS   20   
#define LINK_TIMEOUT_MS       150  
                                 
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

static volatile bool link_alive;             //watchdog-owned heartbeat thread only
static int64_t last_valid_cmd_uptime_ms;     // heartbeat thread only

static enum pi_link_zone_state zone_override = ZONE_STATE_INIT; //mutex-guarded 
static uint16_t reported_current_motor_l, reported_current_motor_r, reported_current_servo; //mutex-guarded via getter

static uint32_t rx_valid_count;
static uint32_t rx_crc_fail_count;
static uint32_t rx_seq_gap_count;
static bool have_last_seq;
static uint8_t last_seq;
//------------------------------------default callbacks----------------------------------------- 

__weak void pi_link_on_command_received(const cmd_payload_t *cmd, bool plausible)
{
    printk("[pi_link] cmd steer=%d thr=%d brk=%d btn=0x%04x flags=0x%02x plausible=%d\n",
           cmd->steering_raw, cmd->throttle_raw, cmd->brake_raw,
           cmd->buttons, cmd->flags, plausible);
}

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
                rx_crc_fail_count++;
                continue;
            }

            //LP_VALID from here down
            if (payload_len != sizeof(cmd_payload_t)) {
                // Right CRC, wrong shape reject rather than reinterpret-cast garbage onto struct
                rx_crc_fail_count++;
                continue;
            }

            cmd_payload_t cmd;
            memcpy(&cmd, payload, sizeof(cmd));
            rx_valid_count++;

            //CMD_RX test point, toggle asap after frame accepted
#if HAVE_CMD_RX_TP
            gpio_pin_toggle_dt(&cmd_rx_tp);
#endif

            // Watchdog reset 
            last_valid_cmd_uptime_ms = k_uptime_get();
            if (!link_alive) {
                link_alive = true;
                pi_link_on_link_restored();
            }

            // seq gap/dup detection 
            if (have_last_seq && (uint8_t)(seq - last_seq) != 1) {
                rx_seq_gap_count++;
            }
            last_seq = seq;
            have_last_seq = true;

            bool plausible =
                cmd.steering_raw >= RAW_STEER_MIN && cmd.steering_raw <= RAW_STEER_MAX &&
                cmd.throttle_raw >= RAW_PEDAL_MIN && cmd.throttle_raw <= RAW_PEDAL_MAX &&
                cmd.brake_raw    >= RAW_PEDAL_MIN && cmd.brake_raw    <= RAW_PEDAL_MAX;

            pi_link_on_command_received(&cmd, plausible);
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
        hb.zone_state = (uint8_t)(link_alive ? ZONE_STATE_NORMAL : ZONE_STATE_LINK_LOST);
    }
    k_mutex_unlock(&state_lock);

    hb.bus_degraded = 0;

    pi_link_get_currents(&hb.current_motor_l, &hb.current_motor_r, &hb.current_servo);

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
       
        if (link_alive) {
            int64_t elapsed = k_uptime_get() - last_valid_cmd_uptime_ms;
            if (elapsed > LINK_TIMEOUT_MS) {
                link_alive = false;
                pi_link_on_link_lost();
            }
        }

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

#if HAVE_CMD_RX_TP
    if (!gpio_is_ready_dt(&cmd_rx_tp)) {
        printk("[pi_link] WARNING: CMD_RX test-point GPIO not ready\n");
    } else {
        gpio_pin_configure_dt(&cmd_rx_tp, GPIO_OUTPUT_INACTIVE);
    }
#else
    printk("[pi_link] NOTE: no CMD_RX test point defined in devicetree -- "
           "add cmd-rx-test-gpios under zephyr,user in your board overlay "
           "if you need to scope it (see the Part 2 test-point table).\n");
#endif

    uart_irq_rx_disable(pi_uart_dev);
    uart_irq_tx_disable(pi_uart_dev);
    uart_irq_callback_user_data_set(pi_uart_dev, uart_isr, NULL);
    uart_irq_rx_enable(pi_uart_dev);

    last_valid_cmd_uptime_ms = k_uptime_get();
    link_alive = false; // ZONE_STATE_INIT equivalent until first valid frame 

    k_thread_start(parser_tid);
    k_thread_start(heartbeat_tid);

    printk("[pi_link] initialized, waiting for first command frame\n");
}

bool pi_link_is_alive(void)
{
    return link_alive;
}

uint32_t pi_link_rx_valid_count(void)     { return rx_valid_count; }
uint32_t pi_link_rx_crc_fail_count(void)  { return rx_crc_fail_count; }
uint32_t pi_link_rx_seq_gap_count(void)   { return rx_seq_gap_count; }
