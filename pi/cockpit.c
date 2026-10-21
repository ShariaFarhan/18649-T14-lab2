
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/gpio.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "lab3_can_catalog.h"  //for can bus and layout info 

// ------------------------------ CONFIG ---------------------------------- 

#define UDP_PORT            8000
#define PROXY_FORCE_PORT    8001 

// Proxy to Pi packet: 4-byte counter + 272-byte DIJOYSTATE2
#define WHEEL_PACKET_LEN    276
#define OFF_COUNTER         0
#define OFF_LX              4     // steering 
#define OFF_LY              8     // throttle 
#define OFF_LRZ             24    // brake  
#define OFF_BUTTONS         52    // rgbButtons[128], 0x80 = pressed 
#define NUM_BUTTONS         128


//raw values from udp stream 
#define IN_STEER_LEFT       (-32767)
#define IN_STEER_CENTER     0
#define IN_STEER_RIGHT      32767
#define IN_PEDAL_RELEASED   0
#define IN_PEDAL_PRESSED    32767

// Raw button indices in rgbButtons
#define RAW_BTN_LSB         9     // left blinker 
#define RAW_BTN_RSB         8     // right blinker
#define RAW_BTN_TEST        2     // self-test(X)

//normalized scale sent in CTRL 
#define FW_STEER_LEFT       CTRL_STEER_MIN      // -1000
#define FW_STEER_CENTER     CTRL_STEER_CENTER   //0
#define FW_STEER_RIGHT      CTRL_STEER_MAX      // +1000
#define FW_PEDAL_RELEASED   CTRL_PEDAL_RELEASED // 0 % 
#define FW_PEDAL_PRESSED    CTRL_PEDAL_PRESSED  // 100 % 
#define FW_PEDAL_ARM_MAX    2     // both pedals within 2% of released to arm
#define FW_BTN_LSB_BIT      CTRL_BTN_LSB
#define FW_BTN_RSB_BIT      CTRL_BTN_RSB
#define BAD_INPUT_STEERING  32767 // out of range 

//blinker self-cancel
#define BLINK_TURN_THRESHOLD    (FW_STEER_RIGHT * 35 / 100)
#define BLINK_RETURN_THRESHOLD  (FW_STEER_RIGHT * 10 / 100)

//noise threshold
#define CAL_TOLERANCE_PERCENT 2

// Deadlines in seconds
#define UDP_TIMEOUT_S       0.100 /* no wheel packet for this long  -> STALE, and
                                     the cockpit stops its heartbeat (Lab 1)  */
#define FEEDBACK_TIMEOUT_S  0.060 // 3 missed CAN heartbeats (R4.5)           
#define COMMAND_PERIOD_S    0.040 /* CTRL resent if no wheel packet for this
                                     long (R4.4: at least every 50 ms). Lab 2
                                     also sent on a fixed 20 ms grid, which on
                                     CAN doubled the CTRL rate.  */             
#define HB_PERIOD_S         0.020 // HB_N1 on both buses (R4.5)               
#define FORCE_PERIOD_S      0.050 // force byte to the proxy (R2.3)           
#define MONITOR_PERIOD_S    0.005 // heartbeat / bus-health check (Lab 3)     
#define FAULT_HOLD_S        0.100 // a received FAULT holds the error state   
#define BLINK_TURN_HALF_S   0.500 // 1 Hz turn signal, 50 % (R2.4)            
#define BLINK_HAZARD_HALF_S 0.250 // 2 Hz hazard, 50 % (S4)                   
#define BLINK_KEEPALIVE_S   0.500 // OFF re-sent this often                   
#define BLINK_MIN_GAP_S     0.020 // catalog minimum interarrival             
#define TEST_DEBOUNCE_S     0.050 // self-test button debounce               
#define TEST_DOUBLE_S       0.500 // second press within this restores       
#define ENABLE_ACK_S        0.500 // STM32 must arm within this, else retry  
#define ENABLE_RESET_S      0.100 // disabled frames sent before a re-arm    
#define STATUS_PERIOD_S     0.500 // status line on the terminal             

#define FLAG_UDP_STALE      CTRL_FLAG_UDP_STALE
#define FLAG_DUAL_ENABLE    CTRL_FLAG_DUAL_ENABLE

#define ZONE_NORMAL         ZONE_STATE_NORMAL
#define SYS_STARTUP         1
#define DRIVE_ARMED         0x01
#define DRIVE_BRAKING       0x02
#define DRIVE_FAULT         0x04
#define DRIVE_BOOST         0x08

// Scope test points and LEDs (BCM numbering). CMD_TX moved from GPIO24 to
//GPIO17: GPIO24 is now the CAN B MCP2515 interrupt (GPIO25 is CAN A).
//Lab 3 adds CAN_TX_A/B test points and the DEG_A/B LEDs (R4.7). 
#define TP_UDP_RX           0
#define TP_CMD_TX           1
#define TP_CAN_TX_A         2
#define TP_CAN_TX_B         3
#define TP_DEG_A            4
#define TP_DEG_B            5
#define TP_COUNT            6
static const unsigned TP_PIN[TP_COUNT] = { 23, 17, 27, 22, 5, 6 };

// ------------------------- WIRE FORMATS --------------------------------- 

// Pi -> STM32 is now the CTRL CAN frame (0x010), built in build_command_frame().
//STM32 -> Pi is now HB_N2 / HB_N3 (0x101 / 0x102). An 8-byte CAN frame has no
//room for the Lab 2 telemetry (currents, RPMs, servo pulse), so the heartbeat
//keeps only the fields the Pi's logic uses. 
typedef struct {
    uint8_t  zone_state;         // HB_STATE        
    uint8_t  bus_bits;           // HB_BUS          
    uint8_t  system_reason;      // HB_REASON       
    uint8_t  drive_flags;        // HB_DRIVE_FLAGS  armed, brake, fault, boost, reason << 4 
    uint8_t  duty_percent;       // HB_DUTY         
} hb_payload_t;

static const char *const SYSTEM_NAME[] = {
    "NORMAL", "STARTUP", "LINK_LOST", "BAD_INPUT", "SELF_TEST", "OUTPUT_FAULT"
};
#define SYSTEM_COUNT (sizeof(SYSTEM_NAME) / sizeof(SYSTEM_NAME[0]))

static const char *const DRIVE_REASON[] = {   // enum dual_reason in dual_drive.h 
    "ok", "command timeout", "left encoder invalid", "right encoder invalid",
    "left count reversed", "right count reversed", "left wheel stalled",
    "right wheel stalled", "drive output failed", "invalid input", "control loop overrun"
};
#define DRIVE_REASON_COUNT (sizeof(DRIVE_REASON) / sizeof(DRIVE_REASON[0]))

static const char *const LIGHT_NAME[] = { "off", "left", "right", "hazard" };


// ------------------------------ STATE ----------------------------------- 

typedef struct {
    int32_t steer_left, steer_center, steer_right;
    int32_t throttle_released, throttle_pressed;
    int32_t brake_released, brake_pressed;
    int btn_lsb, btn_rsb, btn_test;
} calibration_t;

typedef struct {
    bool     enabled;
    int      fd;                 // line-request fd from the GPIO chip 
    char     chip[PATH_MAX];
    bool     level[TP_COUNT];
} test_points_t;

typedef struct {
    // latest wheel input, raw and converted 
    int32_t  in_lx, in_ly, in_lrz;
    uint8_t  in_buttons[NUM_BUTTONS];
    int32_t  steering, throttle, brake;      // normalized CTRL scale 
    uint16_t buttons;                        // firmware bitmask 
    bool     have_rx, have_seq;
    double   last_rx;
    uint32_t last_seq;

    // self-test button: now fails the cockpit zone itself 
    bool     test_prev_known, test_prev_down;
    double   test_last_press;
    uint8_t  test_counter;
    bool     test_click_pending;
    bool     cockpit_failed;                 // silent on CAN until a double press 

    // drive-enable request 
    bool     drive_enable, drive_acknowledged;
    double   drive_requested_at, drive_reset_until;

    bool     bad_input;                      // invalid-input test active 

    // STM32 heartbeats: status/last_hb = N3 (drive zone, as the STM32 was in
    //Lab 2), n2_last_hb = steering zone 
    hb_payload_t status;
    bool     have_status;
    double   last_hb;
    bool     have_n2;
    double   n2_last_hb;
    uint8_t  n2_state;

    // system error state and per-bus health 
    bool     error, booted, isolated;
    const char *error_reason;
    bool     was_failed, was_wheel_fresh;
    uint8_t  fault_to_send;                  // FAULT_R_*, 0 = none 
    bool     have_fault_rx;
    double   fault_rx_at;
    bool     bus_have_rx[2], degraded[2];
    double   bus_last_rx[2];

    //blinker master (logic moved here from the STM32) 
    uint8_t  blink_sel, btn_prev, blink_mode;
    bool     turn_crossed, btn_synced, blink_on, blink_pending, blink_restart;
    double   btn_last[2], blink_next_edge, blink_last_tx;

    // CAN transmit: one 4-bit sequence counter per ID 
    bool     have_tx;
    double   last_tx;
    uint8_t  serial_seq;                     // CTRL  
    uint8_t  hb_seq;                         // HB_N1 
    uint8_t  blink_seq;                      // BLINK 
    uint8_t  fault_seq;                      // FAULT 

    // FORCE (0x300) -> proxy 
    int8_t   force;
    bool     have_force;
    struct sockaddr_in proxy;
    bool     have_proxy;
} state_t;

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig) { (void)sig; g_running = 0; }

// ------------------------------ UTIL ------------------------------------ 

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Keep a periodic schedule on a fixed grid: a late send does not shift later ones. 
static double advance_deadline(double deadline, double now, double period)
{
    if (now < deadline) return deadline;
    return deadline + ((double)(long)((now - deadline) / period) + 1.0)//period;
}

static bool wheel_fresh(const state_t *s, double now)
{
    return s->have_rx && now - s->last_rx <= UDP_TIMEOUT_S;
}

static bool heartbeat_fresh(const state_t *s, double now)
{
    return s->have_status && now - s->last_hb <= FEEDBACK_TIMEOUT_S;
}

static bool n2_fresh(const state_t *s, double now)
{
    return s->have_n2 && now - s->n2_last_hb <= FEEDBACK_TIMEOUT_S;
}

// --------------------------- AXIS CONVERSION ---------------------------- 

// Straight-line map: in_a -> out_a, in_b -> out_b. Works for either direction
//of travel. Slightly past an end clamps to that end; far past an end is
//extrapolated, which lands outside 0..65535 and is rejected by the STM32. 
static int32_t map_axis(int32_t raw, int32_t in_a, int32_t in_b, int32_t out_a, int32_t out_b)
{
    int64_t span = (int64_t)in_b - in_a;
    int64_t pos = (int64_t)raw - in_a;
    if (span < 0) { span = -span; pos = -pos; }
    int64_t tolerance = span//CAL_TOLERANCE_PERCENT / 100;
    if (pos < 0 && pos >= -tolerance) pos = 0;
    if (pos > span && pos <= span + tolerance) pos = span;
    int64_t out = out_a + pos//((int64_t)out_b - out_a) / span;
    if (out > INT32_MAX) out = INT32_MAX;
    if (out < INT32_MIN) out = INT32_MIN;
    return (int32_t)out;
}

static int32_t map_steering(int32_t raw, const calibration_t *c)
{
    bool rising = c->steer_right > c->steer_left;
    bool left_half = rising ? raw < c->steer_center : raw > c->steer_center;
    return left_half
        ? map_axis(raw, c->steer_left, c->steer_center, FW_STEER_LEFT, FW_STEER_CENTER)
        : map_axis(raw, c->steer_center, c->steer_right, FW_STEER_CENTER, FW_STEER_RIGHT);
}

static int32_t map_pedal(int32_t raw, int32_t released, int32_t pressed)
{
    return map_axis(raw, released, pressed, FW_PEDAL_RELEASED, FW_PEDAL_PRESSED);
}

// ---------------------------- TEST POINTS ------------------------------- 

// Uses the kernel GPIO character device directly, so no libgpiod is needed.
//Finds the header GPIO chip by label and by the line names (GPIO23 ...). 
static int test_points_open(test_points_t *tp)
{
    char seen[16][PATH_MAX];
    int seen_count = 0;

    tp->fd = -1;
    for (int i = 0; i < 16; i++) {
        char path[32], real[PATH_MAX];
        snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
        if (!realpath(path, real)) continue;

        bool duplicate = false;          // Pi 5 has symlinked aliases of one chip 
        for (int k = 0; k < seen_count; k++)
            if (strcmp(seen[k], real) == 0) duplicate = true;
        if (duplicate) continue;
        snprintf(seen[seen_count++], PATH_MAX, "%s", real);

        int chip = open(real, O_RDWR | O_CLOEXEC);
        if (chip < 0) continue;

        struct gpiochip_info info;
        memset(&info, 0, sizeof(info));
        bool match = ioctl(chip, GPIO_GET_CHIPINFO_IOCTL, &info) == 0 && info.lines > 24 &&
                     (strncmp(info.label, "pinctrl-rp1", 11) == 0 ||
                      strncmp(info.label, "pinctrl-bcm", 11) == 0);
        for (int p = 0; match && p < TP_COUNT; p++) {
            struct gpio_v2_line_info line;
            char want[16];
            memset(&line, 0, sizeof(line));
            line.offset = TP_PIN[p];
            snprintf(want, sizeof(want), "GPIO%u", TP_PIN[p]);
            if (ioctl(chip, GPIO_V2_GET_LINEINFO_IOCTL, &line) != 0 || strcmp(line.name, want) != 0)
                match = false;
        }
        if (!match) { close(chip); continue; }

        // Request all lines as outputs, starting low. Never fall back to
        //another chip if these lines are busy. 
        struct gpio_v2_line_request req;
        memset(&req, 0, sizeof(req));
        for (int p = 0; p < TP_COUNT; p++) req.offsets[p] = TP_PIN[p];
        req.num_lines = TP_COUNT;
        snprintf(req.consumer, sizeof(req.consumer), "lab3-test-points");
        req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        req.config.num_attrs = 1;
        req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
        req.config.attrs[0].attr.values = 0;
        req.config.attrs[0].mask = (1ULL << TP_COUNT) - 1;
        int rc = ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req);
        int saved = errno;
        close(chip);
        if (rc != 0) {
            fprintf(stderr, "test-point GPIOs on %s are busy: %s\n", real, strerror(saved));
            return -1;
        }
        tp->fd = req.fd;
        snprintf(tp->chip, sizeof(tp->chip), "%s", real);
        return 0;
    }
    fprintf(stderr, "Pi header GPIO chip with the test-point lines not found; test markers are required "
                    "(use --no-test-points to run without them)\n");
    return -1;
}

static void test_points_toggle(test_points_t *tp, int which)
{
    tp->level[which] = !tp->level[which];
    if (!tp->enabled) return;
    struct gpio_v2_line_values v;
    v.mask = 1ULL << which;
    v.bits = tp->level[which] ? (1ULL << which) : 0;
    if (ioctl(tp->fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) != 0)
        perror("test point write");
}

//LEDs are set to a level instead of toggled
static void test_points_set(test_points_t *tp, int which, bool level)
{
    if (tp->level[which] != level) test_points_toggle(tp, which);
}

static void test_points_close(test_points_t *tp)
{
    if (tp->enabled && tp->fd >= 0) close(tp->fd);
    tp->fd = -1;
}

// ------------------------------- CAN ------------------------------------ 

// One CAN_RAW socket per bus, receiving only the IDs the cockpit uses. 
static int can_open(const char *ifname)
{
    int fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) { perror("socket(PF_CAN)"); return -1; }

    struct can_filter filter[] = {
        { CAN_ID_FAULT, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG },
        { CAN_ID_HB_N2, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG },
        { CAN_ID_HB_N3, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG },
        { CAN_ID_FORCE, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG },
    };
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = (int)if_nametoindex(ifname);
    if (addr.can_ifindex == 0) {
        fprintf(stderr, "CAN interface %s not found (config.txt overlays, ip link set %s up)\n", ifname, ifname);
    } else if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filter, sizeof(filter)) != 0 ||
               bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        perror(ifname);
    } else {
        return fd;
    }
    close(fd);
    return -1;
}

// Hot/parallel transmit: the same frame on bus A and bus B, back to back.
//A pulled jumper means no ACK, the TX queue fills and write() fails with
//ENOBUFS. That bus is simply skipped; the other one keeps working. 
static void can_send_both(const int can_fd[2], test_points_t *tp, const struct can_frame *f)
{
    for (int b = 0; b < 2; b++) {
        test_points_toggle(tp, b == 0 ? TP_CAN_TX_A : TP_CAN_TX_B);
        (void)!write(can_fd[b], f, sizeof(*f));
    }
}

// Normalized values -> CAN field types. Out of range stays out of range so
//the STM32 rejects it (R4.5); pedals outside 0..100 are sent as 255. 
static int16_t to_steer16(int32_t v)
{
    return (int16_t)(v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : v);
}

static uint8_t to_pedal8(int32_t v)
{
    return (v < FW_PEDAL_RELEASED || v > FW_PEDAL_PRESSED) ? 255 : (uint8_t)v;
}

static void build_command_frame(struct can_frame *f, const state_t *s, bool stale)
{
    int16_t steer = to_steer16(s->bad_input ? BAD_INPUT_STEERING : s->steering);
    // A stale frame can never carry the drive-enable bit. 
    uint8_t flags = (uint8_t)((stale ? FLAG_UDP_STALE : 0) |
                              (s->drive_enable && s->have_status && !stale ? FLAG_DUAL_ENABLE : 0));

    memset(f, 0, sizeof(*f));
    f->can_id = CAN_ID_CTRL;
    f->can_dlc = CAN_FRAME_DLC;
    f->data[CAN_B0] = can_byte0(s->serial_seq, flags);
    f->data[CTRL_STEER_LO] = (uint8_t)((uint16_t)steer & 0xFF);
    f->data[CTRL_STEER_HI] = (uint8_t)((uint16_t)steer >> 8);
    f->data[CTRL_THROTTLE] = to_pedal8(s->throttle);
    f->data[CTRL_BRAKE] = to_pedal8(s->brake);
    f->data[CTRL_BUTTONS] = (uint8_t)s->buttons;
    f->data[CTRL_CRC] = can_crc8(f->data, CTRL_CRC);
}

// HB_N1 (0x100): sent only while the wheel stream is fresh (Lab 1: no UDP
//for 100 ms -> the cockpit counts as failed and stops its heartbeat) and the
//cockpit has not been failed with the test button. 
static void build_heartbeat_frame(struct can_frame *f, const state_t *s)
{
    memset(f, 0, sizeof(*f));
    f->can_id = CAN_ID_HB_N1;
    f->can_dlc = CAN_FRAME_DLC;
    f->data[CAN_B0] = can_byte0(s->hb_seq, s->error ? CAN_FLAG_SYS_ERROR : 0);
    f->data[HB_STATE] = s->error ? ZONE_STATE_SYS_ERROR : ZONE_STATE_NORMAL;
    f->data[HB_BUS] = (uint8_t)((s->degraded[0] ? HB_BUS_A_DEGRADED : 0) |
                                (s->degraded[1] ? HB_BUS_B_DEGRADED : 0));
}

// BLINK (0x200), the cockpit is the blinker master
static void build_blink_frame(struct can_frame *f, const state_t *s)
{
    memset(f, 0, sizeof(*f));
    f->can_id = CAN_ID_BLINK;
    f->can_dlc = CAN_FRAME_DLC;
    f->data[CAN_B0] = can_byte0(s->blink_seq, s->error ? CAN_FLAG_SYS_ERROR : 0);
    f->data[BLINK_MODE] = s->blink_mode;
    f->data[BLINK_PHASE] = s->blink_on ? 1 : 0;
}

// Lab 3: FAULT (0x020), sent once when the cockpit itself fails. 
static void build_fault_frame(struct can_frame *f, const state_t *s, uint8_t reason)
{
    memset(f, 0, sizeof(*f));
    f->can_id = CAN_ID_FAULT;
    f->can_dlc = CAN_FRAME_DLC;
    f->data[CAN_B0] = can_byte0(s->fault_seq, CAN_FLAG_SYS_ERROR);
    f->data[FAULT_ZONE] = ZONE_COCKPIT;
    f->data[FAULT_REASON] = reason;
}

// --------------------------- WHEEL PACKETS ------------------------------ 

static bool button_down(const state_t *s, int index)
{
    return index >= 0 && index < NUM_BUTTONS && (s->in_buttons[index] & 0x80);
}

// Count debounced presses of the self-test button. Lab 3: the wheel button is
//the cockpit zone's own test input, so the Pi decides here (same rule as the
//STM32 zones): a press fails the cockpit, a second press within 500 ms
//restores it. 
static void self_test_update(state_t *s, bool down, double now)
{
    bool rising = s->test_prev_known && !s->test_prev_down && down;
    s->test_prev_known = true;
    s->test_prev_down = down;
    if (rising && now - s->test_last_press >= TEST_DEBOUNCE_S) {
        bool recover = s->test_click_pending && now - s->test_last_press <= TEST_DOUBLE_S;
        s->test_last_press = now;
        s->test_counter = (uint8_t)((s->test_counter + 1) & 31);
        s->test_click_pending = !recover;
        s->cockpit_failed = !recover;
        printf("Cockpit test button: %s\n", recover ? "double press, zone restored" : "press, zone FAILED");
    }
}

// ------------------------------ BLINKERS -------------------------------- 
// Lab 3: the blinker logic of Lab 2 (logic/steering/wheel_buttons.c and
//logic/blinkers/blinkers.c) moved from the STM32 to the Pi. 

static double blink_step(uint8_t mode)
{
    return mode == BLINK_MODE_OFF ? BLINK_KEEPALIVE_S
         : mode == BLINK_MODE_HAZARD ? BLINK_HAZARD_HALF_S : BLINK_TURN_HALF_S;
}

// A press selects that side, pressing the active side again cancels, both
//held selects nothing, 50 ms debounce per button. 
static void blinker_buttons(state_t *s, double now)
{
    uint8_t held = (uint8_t)(s->buttons & (FW_BTN_LSB_BIT | FW_BTN_RSB_BIT));
    uint8_t pressed = held & (uint8_t)~s->btn_prev;
    s->btn_prev = held;
    if (!s->btn_synced) { s->btn_synced = true; return; }
    if (s->error || held == (FW_BTN_LSB_BIT | FW_BTN_RSB_BIT)) return;

    static const uint8_t mask[2] = { FW_BTN_LSB_BIT, FW_BTN_RSB_BIT };
    static const uint8_t mode[2] = { BLINK_MODE_LEFT, BLINK_MODE_RIGHT };
    for (int i = 0; i < 2; i++) {
        if ((pressed & mask[i]) && now - s->btn_last[i] >= TEST_DEBOUNCE_S) {
            s->btn_last[i] = now;
            s->blink_sel = s->blink_sel == mode[i] ? BLINK_MODE_OFF : mode[i];
            s->turn_crossed = false;
            return;
        }
    }
}

// Arm after the wheel passes 35 %, cancel when it comes back inside 10 %. 
static void blinker_self_cancel(state_t *s)
{
    int32_t st = s->steering;
    if (st < FW_STEER_LEFT || st > FW_STEER_RIGHT) return;
    if (s->blink_sel == BLINK_MODE_LEFT) {
        if (!s->turn_crossed && st <= -BLINK_TURN_THRESHOLD) s->turn_crossed = true;
        else if (s->turn_crossed && st >= -BLINK_RETURN_THRESHOLD) s->blink_sel = BLINK_MODE_OFF;
    } else if (s->blink_sel == BLINK_MODE_RIGHT) {
        if (!s->turn_crossed && st >= BLINK_TURN_THRESHOLD) s->turn_crossed = true;
        else if (s->turn_crossed && st <= BLINK_RETURN_THRESHOLD) s->blink_sel = BLINK_MODE_OFF;
    }
}

// New mode: blinkers on at once (R2.4, start within 100 ms). 
static void blinker_set_output(state_t *s, uint8_t mode, double now)
{
    if (mode == s->blink_mode && s->blink_next_edge > 0) return;
    s->blink_mode = mode;
    s->blink_on = mode != BLINK_MODE_OFF;
    s->blink_next_edge = now + blink_step(mode);
    s->blink_pending = true;
    s->blink_restart = true;
}

// Advance the blinkers edges on a fixed grid. Returns true when a BLINK frame is due. 
static bool blinker_due(state_t *s, double now)
{
    if (s->blink_next_edge > 0 && now >= s->blink_next_edge) {
        double step = blink_step(s->blink_mode);
        long edges = 1 + (long)((now - s->blink_next_edge) / step);
        if (s->blink_mode != BLINK_MODE_OFF && (edges & 1)) s->blink_on = !s->blink_on;
        s->blink_next_edge += (double)edges//step;
        s->blink_pending = true;
    }
    return s->blink_pending && now - s->blink_last_tx >= BLINK_MIN_GAP_S;
}

static void blinker_sent(state_t *s, double now)
{
    // The first frame of a mode may have waited out the 20 ms gap: time the
    //next edge from when the zones actually see it. 
    if (s->blink_restart) s->blink_next_edge = now + blink_step(s->blink_mode);
    s->blink_restart = false;
    s->blink_pending = false;
    s->blink_last_tx = now;
    s->blink_seq = (uint8_t)((s->blink_seq + 1) & CAN_SEQ_MASK);
}

// Take one 276-byte proxy packet and normalize it to the CTRL scale.
//Returns true if it was accepted as newer. 
static bool wheel_packet(state_t *s, const calibration_t *cal, const uint8_t *d, ssize_t n, double now)
{
    if (n != WHEEL_PACKET_LEN) return false;

    // Accept the first packet, a packet after a long silence (a restarted
    //proxy), or any packet newer than the last one. Drop duplicates and
    //reordered packets. 
    uint32_t counter = rd_u32le(&d[OFF_COUNTER]);
    uint32_t delta = counter - s->last_seq;
    bool resumed = !s->have_rx || now - s->last_rx > UDP_TIMEOUT_S;
    if (s->have_seq && !resumed && !(delta > 0 && delta < 0x80000000u)) return false;

    s->in_lx = (int32_t)rd_u32le(&d[OFF_LX]);
    s->in_ly = (int32_t)rd_u32le(&d[OFF_LY]);
    s->in_lrz = (int32_t)rd_u32le(&d[OFF_LRZ]);
    memcpy(s->in_buttons, &d[OFF_BUTTONS], NUM_BUTTONS);

    s->steering = map_steering(s->in_lx, cal);
    s->throttle = map_pedal(s->in_ly, cal->throttle_released, cal->throttle_pressed);
    s->brake = map_pedal(s->in_lrz, cal->brake_released, cal->brake_pressed);
    s->buttons = (uint16_t)((button_down(s, cal->btn_lsb) ? FW_BTN_LSB_BIT : 0) |
                            (button_down(s, cal->btn_rsb) ? FW_BTN_RSB_BIT : 0));

    // After a gap, a button that is already held must not count as a press. 
    if (resumed) { s->test_prev_known = false; s->btn_synced = false; }
    self_test_update(s, button_down(s, cal->btn_test), now);
    blinker_buttons(s, now);
    blinker_self_cancel(s);

    s->last_seq = counter;
    s->have_seq = true;
    s->last_rx = now;
    s->have_rx = true;
    return true;
}

// ----------------------------- HEARTBEAT -------------------------------- 

static bool heartbeat_valid(const hb_payload_t *hb)
{
    return hb->system_reason < SYSTEM_COUNT;
}

static void heartbeat_decode(hb_payload_t *hb, const uint8_t *d)
{
    hb->zone_state = d[HB_STATE];
    hb->bus_bits = d[HB_BUS];
    hb->system_reason = d[HB_REASON];
    hb->drive_flags = d[HB_DRIVE_FLAGS];
    hb->duty_percent = d[HB_DUTY];
}

// ------------------------ ZONE STATE (Lab 3) ----------------------------- 

// Bus health (R4.7), zone failure (R4.5) and the system error state (R3).
//Power-up is in error until HB_N2 and HB_N3 have been seen on either bus. 
static void zone_state_update(state_t *s, double now)
{
    bool silent[2];
    for (int b = 0; b < 2; b++)
        silent[b] = !s->bus_have_rx[b] || now - s->bus_last_rx[b] > FEEDBACK_TIMEOUT_S;
    s->isolated = silent[0] && silent[1];
    s->degraded[0] = silent[0] && !silent[1];
    s->degraded[1] = silent[1] && !silent[0];

    bool wheel_ok = wheel_fresh(s, now);
    bool n2 = n2_fresh(s, now), n3 = heartbeat_fresh(s, now);
    if (n2 && n3) s->booted = true;
    bool zone_fault = (n2 && s->n2_state == ZONE_STATE_LOCAL_FAULT) ||
                      (n3 && s->status.zone_state == ZONE_STATE_LOCAL_FAULT);
    bool fault_rx = s->have_fault_rx && now - s->fault_rx_at < FAULT_HOLD_S;

    const char *reason = NULL;
    if (s->cockpit_failed)  reason = "SELF_TEST";
    else if (!wheel_ok)     reason = "UDP_LOST";
    else if (s->isolated)   reason = "ISOLATED";
    else if (!n2 || !n3)    reason = !s->booted ? "STARTUP" : !n2 ? "N2_LOST" : "N3_LOST";
    else if (fault_rx)      reason = "FAULT_RX";
    else if (zone_fault)    reason = "ZONE_FAULT";

    // The cockpit itself failing: tell the others once, then go quiet. 
    if (s->cockpit_failed && !s->was_failed) s->fault_to_send = FAULT_R_SELF_TEST;
    else if (!s->cockpit_failed && s->was_wheel_fresh && !wheel_ok) s->fault_to_send = FAULT_R_UDP_LOST;
    s->was_failed = s->cockpit_failed;
    s->was_wheel_fresh = wheel_ok;

    bool error = reason != NULL;
    if (error != s->error || (error && strcmp(reason, s->error_reason) != 0))
        printf("Cockpit state: %s\n", error ? reason : "NORMAL");
    if (error && !s->error) { s->blink_sel = BLINK_MODE_OFF; s->turn_crossed = false; }
    s->error = error;
    s->error_reason = error ? reason : "NORMAL";
    blinker_set_output(s, error ? BLINK_MODE_HAZARD : s->blink_sel, now);
}

// ---------------------------- DRIVE ENABLE ------------------------------ 

// The STM32 only arms the motors when a command carries the enable bit. The Pi
//asks for it only when the wheel stream and heartbeat are fresh, the STM32
//reports NORMAL with no drive fault, and both pedals are released. If the
//STM32 disarms for any reason, the request is dropped and is not raised again
//until those conditions hold once more. 
static void drive_enable_update(state_t *s, double now)
{
    bool wheel_ok = wheel_fresh(s, now);
    bool hb_ok = heartbeat_fresh(s, now);
    bool fault = hb_ok && (s->status.drive_flags & DRIVE_FAULT);

    if (s->drive_enable && (!wheel_ok || !hb_ok || fault || s->bad_input || s->error)) {
        s->drive_enable = false;
        s->drive_reset_until = now + ENABLE_RESET_S;
    }

    if (!s->drive_enable) {
        s->drive_acknowledged = false;
    } else if (s->status.drive_flags & DRIVE_ARMED) {
        s->drive_acknowledged = true;
    } else if (s->drive_acknowledged || now - s->drive_requested_at >= ENABLE_ACK_S) {
        // It was armed and is not any more, or it never armed: stop asking. 
        s->drive_enable = false;
        s->drive_acknowledged = false;
        s->drive_reset_until = now + ENABLE_RESET_S;
    }

    if (!s->drive_enable && wheel_ok && hb_ok && !fault && !s->bad_input && !s->error &&
        now >= s->drive_reset_until && s->status.zone_state == ZONE_NORMAL &&
        s->throttle <= FW_PEDAL_ARM_MAX && s->brake <= FW_PEDAL_ARM_MAX) {
        s->drive_enable = true;
        s->drive_acknowledged = false;
        s->drive_requested_at = now;
    }
}

// ------------------------------ STATUS ---------------------------------- 

static void print_status(const state_t *s, double now)
{
    printf("[cockpit %s]", s->error_reason);
    if (heartbeat_fresh(s, now)) printf("[N3 %s]", SYSTEM_NAME[s->status.system_reason]);
    else printf("[NO N3 HEARTBEAT]");
    if (!n2_fresh(s, now)) printf("[NO N2 HEARTBEAT]");

    if (wheel_fresh(s, now)) {
        printf(" in lX=%d lY=%d lRz=%d btn=", s->in_lx, s->in_ly, s->in_lrz);
        bool any = false;
        for (int i = 0; i < NUM_BUTTONS; i++) {
            if (s->in_buttons[i] & 0x80) { printf(any ? ",%d" : "%d", i); any = true; }
        }
        if (!any) printf("-");
        printf(" | out steer=%d thr=%d brk=%d", s->bad_input ? BAD_INPUT_STEERING : s->steering,
               s->throttle, s->brake);
    } else {
        printf(" in: no wheel packets");
    }
    printf(" test=%u%s enable=%d%s", (unsigned)s->test_counter, s->cockpit_failed ? "(FAILED)" : "",
           s->drive_enable, s->bad_input ? " BAD-INPUT-TEST" : "");
    printf(" | lights=%s DEG A=%d B=%d", LIGHT_NAME[s->blink_mode & 3],
           s->degraded[0] || s->isolated, s->degraded[1] || s->isolated);

    if (heartbeat_fresh(s, now)) {
        const hb_payload_t *hb = &s->status;
        unsigned reason = hb->drive_flags >> 4;
        printf(" | n3 pwm=%u%% armed=%d brake=%d",
               (unsigned)hb->duty_percent, !!(hb->drive_flags & DRIVE_ARMED), !!(hb->drive_flags & DRIVE_BRAKING));
        if (hb->drive_flags & DRIVE_FAULT)
            printf(" FAULT(%s)", reason < DRIVE_REASON_COUNT ? DRIVE_REASON[reason] : "unknown");
        if (hb->bus_bits) printf(" n3-degraded=%s%s", hb->bus_bits & HB_BUS_A_DEGRADED ? "A" : "",
                                 hb->bus_bits & HB_BUS_B_DEGRADED ? "B" : "");
    }
    if (s->have_force) printf(" | force=%d", s->force);
    printf("\n");
}

// ------------------------------- MAIN ----------------------------------- 

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  --can-a IF            CAN A interface (default can0)\n"
        "  --can-b IF            CAN B interface (default can1)\n"
        "  --bind IP             local address for UDP port %d (default 0.0.0.0)\n"
        "  --pc-ip IP            accept wheel packets only from this address (default: any)\n"
        "  --steer L,C,R         raw steering at full left, center, full right (default %d,%d,%d)\n"
        "  --throttle REL,PRESS  raw throttle released, fully pressed (default %d,%d)\n"
        "  --brake REL,PRESS     raw brake released, fully pressed (default %d,%d)\n"
        "  --btn-lsb N           raw button index for the left blinker (default %d)\n"
        "  --btn-rsb N           raw button index for the right blinker (default %d)\n"
        "  --btn-test N          raw button index for the self-test button (default %d)\n"
        "  --quiet               no periodic status line\n"
        "  --no-test-points      run without the test points / DEG LEDs (GPIO 23,17,27,22,5,6)\n",
        prog, UDP_PORT, IN_STEER_LEFT, IN_STEER_CENTER, IN_STEER_RIGHT,
        IN_PEDAL_RELEASED, IN_PEDAL_PRESSED, IN_PEDAL_RELEASED, IN_PEDAL_PRESSED,
        RAW_BTN_LSB, RAW_BTN_RSB, RAW_BTN_TEST);
}

static bool calibration_ok(const calibration_t *c)
{
    bool steer_ok = (c->steer_left < c->steer_center && c->steer_center < c->steer_right) ||
                    (c->steer_left > c->steer_center && c->steer_center > c->steer_right);
    bool buttons_ok = c->btn_lsb >= 0 && c->btn_lsb < NUM_BUTTONS && c->btn_rsb >= 0 &&
                      c->btn_rsb < NUM_BUTTONS && c->btn_test >= 0 && c->btn_test < NUM_BUTTONS;
    return steer_ok && buttons_ok && c->throttle_released != c->throttle_pressed &&
           c->brake_released != c->brake_pressed;
}

int main(int argc, char **argv)
{
    const char *can_if[2] = { "can0", "can1" }, *bind_ip = "0.0.0.0", *pc_ip = NULL;
    bool use_test_points = true, quiet = false;
    calibration_t cal = {
        IN_STEER_LEFT, IN_STEER_CENTER, IN_STEER_RIGHT,
        IN_PEDAL_RELEASED, IN_PEDAL_PRESSED, IN_PEDAL_RELEASED, IN_PEDAL_PRESSED,
        RAW_BTN_LSB, RAW_BTN_RSB, RAW_BTN_TEST
    };

    static const struct option options[] = {
        { "can-a", required_argument, NULL, 'A' },
        { "can-b", required_argument, NULL, 'B' },
        { "bind", required_argument, NULL, 'b' },
        { "pc-ip", required_argument, NULL, 'p' },
        { "steer", required_argument, NULL, 's' },
        { "throttle", required_argument, NULL, 't' },
        { "brake", required_argument, NULL, 'k' },
        { "btn-lsb", required_argument, NULL, 'L' },
        { "btn-rsb", required_argument, NULL, 'R' },
        { "btn-test", required_argument, NULL, 'T' },
        { "quiet", no_argument, NULL, 'q' },
        { "no-test-points", no_argument, NULL, 'n' },
        { "help", no_argument, NULL, 'h' },
        { 0, 0, 0, 0 }
    };
    int opt;
    bool args_ok = true;
    while ((opt = getopt_long(argc, argv, "A:B:b:p:s:t:k:L:R:T:qnh", options, NULL)) != -1) {
        switch (opt) {
        case 'A': can_if[0] = optarg; break;
        case 'B': can_if[1] = optarg; break;
        case 'b': bind_ip = optarg; break;
        case 'p': pc_ip = optarg; break;
        case 's': args_ok &= sscanf(optarg, "%d,%d,%d", &cal.steer_left, &cal.steer_center, &cal.steer_right) == 3; break;
        case 't': args_ok &= sscanf(optarg, "%d,%d", &cal.throttle_released, &cal.throttle_pressed) == 2; break;
        case 'k': args_ok &= sscanf(optarg, "%d,%d", &cal.brake_released, &cal.brake_pressed) == 2; break;
        case 'L': cal.btn_lsb = atoi(optarg); break;
        case 'R': cal.btn_rsb = atoi(optarg); break;
        case 'T': cal.btn_test = atoi(optarg); break;
        case 'q': quiet = true; break;
        case 'n': use_test_points = false; break;
        default: usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }
    struct in_addr pc_addr = {0}, bind_addr;
    if (!args_ok || !calibration_ok(&cal) || inet_pton(AF_INET, bind_ip, &bind_addr) != 1 ||
        (pc_ip && inet_pton(AF_INET, pc_ip, &pc_addr) != 1)) {
        usage(argv[0]);
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    // Ctrl-C and SIGTERM end the loop so the final STALE frame is sent. 
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    int can_fd[2];
    can_fd[0] = can_open(can_if[0]);
    can_fd[1] = can_fd[0] >= 0 ? can_open(can_if[1]) : -1;
    if (can_fd[1] < 0) {
        if (can_fd[0] >= 0) close(can_fd[0]);
        return 1;
    }

    test_points_t tp;
    memset(&tp, 0, sizeof(tp));
    tp.fd = -1;
    tp.enabled = use_test_points;
    if (tp.enabled && test_points_open(&tp) != 0) {
        close(can_fd[0]);
        close(can_fd[1]);
        return 1;
    }

    int udp_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    struct sockaddr_in local;
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_addr = bind_addr;
    local.sin_port = htons(UDP_PORT);
    if (udp_fd < 0 || bind(udp_fd, (struct sockaddr *)&local, sizeof(local)) != 0) {
        perror("udp bind");
        test_points_close(&tp);
        close(can_fd[0]);
        close(can_fd[1]);
        if (udp_fd >= 0) close(udp_fd);
        return 1;
    }

    // Keyboard commands are read only when this program owns the terminal. 
    bool stdin_open = !isatty(STDIN_FILENO) || tcgetpgrp(STDIN_FILENO) == getpgrp();

    // Until the proxy sends something: centered, pedals released, and in the
    //error state (R3: every zone powers up in error). 
    state_t s;
    memset(&s, 0, sizeof(s));
    s.steering = FW_STEER_CENTER;
    s.throttle = FW_PEDAL_RELEASED;
    s.brake = FW_PEDAL_RELEASED;
    s.test_last_press = -1.0;
    s.error = true;
    s.error_reason = "STARTUP";
    s.btn_last[0] = s.btn_last[1] = s.blink_last_tx = -1.0;

    double next_send = now_s(), next_status = next_send;
    double next_hb = next_send, next_force = next_send;
    int exit_code = 0;

    printf("Cockpit: UDP/%d from %s -> CAN A=%s B=%s\n", UDP_PORT, pc_ip ? pc_ip : "any sender",
           can_if[0], can_if[1]);
    printf("Calibration: steer %d,%d,%d  throttle %d,%d  brake %d,%d  buttons LSB=%d RSB=%d test=%d\n",
           cal.steer_left, cal.steer_center, cal.steer_right, cal.throttle_released, cal.throttle_pressed,
           cal.brake_released, cal.brake_pressed, cal.btn_lsb, cal.btn_rsb, cal.btn_test);
    if (stdin_open) printf("Keys: b + Enter = toggle invalid-input test, q + Enter = quit\n");

    while (g_running) {
        bool send_update = false;

        // Sleep until a wheel packet, CAN frames, a key, or the next deadline. 
        double wake = next_send < next_hb ? next_send : next_hb;
        if (next_force < wake) wake = next_force;
        double wait = wake - now_s();
        if (wait < 0) wait = 0;
        if (wait > MONITOR_PERIOD_S) wait = MONITOR_PERIOD_S;   // Lab 3: heartbeat monitor tick 
        struct timespec ts = { (time_t)wait, (long)((wait - (double)(time_t)wait)//1e9) };
        struct pollfd fds[4] = {
            { can_fd[0], POLLIN, 0 }, { udp_fd, POLLIN, 0 }, { stdin_open ? STDIN_FILENO : -1, POLLIN, 0 },
            { can_fd[1], POLLIN, 0 }
        };
        if (ppoll(fds, 4, &ts, NULL) < 0) {
            if (errno == EINTR) continue;
            perror("ppoll");
            exit_code = 1;
            break;
        }

        // ---- proxy -> Pi: one wheel packet per pass ---- 
        if (fds[1].revents & POLLIN) {
            uint8_t data[2048];
            struct sockaddr_in from;
            socklen_t from_len = sizeof(from);
            ssize_t n = recvfrom(udp_fd, data, sizeof(data), 0, (struct sockaddr *)&from, &from_len);
            if (n >= 0 && (!pc_ip || from.sin_addr.s_addr == pc_addr.s_addr)) {
                test_points_toggle(&tp, TP_UDP_RX);
                // A newer packet is forwarded now, without waiting for the deadline. 
                if (wheel_packet(&s, &cal, data, n, now_s())) {
                    send_update = true;
                    s.proxy = from;                      // force goes back to the sender 
                    s.proxy.sin_port = htons(PROXY_FORCE_PORT);
                    s.have_proxy = true;
                }
            }
        }

        // ---- STM32s -> Pi: heartbeats and force, from both buses ---- 
        // Both copies of a frame are processed: they carry the same values, so
        //taking the second one again changes nothing (no dedup needed here). 
        for (int b = 0; b < 2; b++) {
            short ev = fds[b == 0 ? 0 : 3].revents;
            if (!(ev & (POLLIN | POLLERR))) continue;
            struct can_frame f;
            while (read(can_fd[b], &f, sizeof(f)) == (ssize_t)sizeof(f)) {
                if (f.can_dlc != CAN_FRAME_DLC) continue;
                double t = now_s();
                s.bus_have_rx[b] = true;             // R4.7: traffic on this bus 
                s.bus_last_rx[b] = t;
                if (f.can_id == CAN_ID_FAULT) {
                    if (!s.have_fault_rx || t - s.fault_rx_at >= FAULT_HOLD_S)
                        printf("FAULT received: zone %u reason %u\n", (unsigned)f.data[FAULT_ZONE],
                               (unsigned)f.data[FAULT_REASON]);
                    s.have_fault_rx = true;
                    s.fault_rx_at = t;
                    continue;
                }
                if (f.can_id == CAN_ID_FORCE) {
                    int v = (int8_t)f.data[FORCE_VALUE];
                    s.force = (int8_t)(v < FORCE_MIN ? FORCE_MIN : v > FORCE_MAX ? FORCE_MAX : v);
                    s.have_force = true;
                    continue;
                }
                if (f.can_id == CAN_ID_HB_N2) {
                    if (!s.have_n2) printf("N2 heartbeat received\n");
                    s.have_n2 = true;
                    s.n2_last_hb = t;
                    s.n2_state = f.data[HB_STATE];
                    continue;
                }
                if (f.can_id != CAN_ID_HB_N3) continue;

                hb_payload_t hb;
                heartbeat_decode(&hb, f.data);
                if (!heartbeat_valid(&hb)) continue;

                bool changed = !s.have_status || s.status.system_reason != hb.system_reason;
                if (!s.have_status) printf("N3 heartbeat received\n");
                s.status = hb;
                s.have_status = true;
                s.last_hb = t;
                if (changed) print_status(&s, t);
            }
        }

        // ---- keyboard ---- 
        if (fds[2].revents & (POLLIN | POLLHUP)) {
            char keys[64];
            ssize_t n = read(STDIN_FILENO, keys, sizeof(keys));
            if (n <= 0) stdin_open = false;
            for (ssize_t i = 0; i < n; i++) {
                if (keys[i] == 'b') {
                    s.bad_input = !s.bad_input;
                    printf("Invalid-input test %s\n", s.bad_input ? "ON: sending out-of-range steering" : "OFF");
                } else if (keys[i] == 'q') {
                    g_running = 0;
                }
            }
        }

        // ---- Pi -> STM32s: CTRL on a wheel update or the 20 ms deadline ---- 
        double now = now_s();
        drive_enable_update(&s, now);
        if (send_update || now >= next_send) {
            if (!s.cockpit_failed) {                 // a failed zone is silent 
                bool stale = !wheel_fresh(&s, now) || !heartbeat_fresh(&s, now);
                struct can_frame frame;
                build_command_frame(&frame, &s, stale);
                test_points_toggle(&tp, TP_CMD_TX);
                can_send_both(can_fd, &tp, &frame);
                s.last_tx = now_s();
                s.have_tx = true;
                s.serial_seq = (uint8_t)((s.serial_seq + 1) & CAN_SEQ_MASK);
            }
            next_send = now_s() + COMMAND_PERIOD_S;  // restarts on every CTRL 
        }

        // ---- Lab 3: zone state, DEG LEDs, FAULT ---- 
        now = now_s();
        zone_state_update(&s, now);
        test_points_set(&tp, TP_DEG_A, s.degraded[0] || s.isolated);
        test_points_set(&tp, TP_DEG_B, s.degraded[1] || s.isolated);
        if (s.fault_to_send) {
            struct can_frame ff;
            build_fault_frame(&ff, &s, s.fault_to_send);
            can_send_both(can_fd, &tp, &ff);
            s.fault_seq = (uint8_t)((s.fault_seq + 1) & CAN_SEQ_MASK);
            printf("FAULT sent: cockpit reason %u\n", (unsigned)s.fault_to_send);
            s.fault_to_send = 0;
        }

        // ---- Lab 3: BLINK on every blinkers edge / mode change ---- 
        if (blinker_due(&s, now) && !s.cockpit_failed) {
            struct can_frame bf;
            build_blink_frame(&bf, &s);
            can_send_both(can_fd, &tp, &bf);
            blinker_sent(&s, now);
        }

        // ---- HB_N1 every 20 ms while the wheel stream is fresh ---- 
        if (now >= next_hb) {
            if (wheel_fresh(&s, now) && !s.cockpit_failed) {
                struct can_frame hbf;
                build_heartbeat_frame(&hbf, &s);
                can_send_both(can_fd, &tp, &hbf);
                s.hb_seq = (uint8_t)((s.hb_seq + 1) & CAN_SEQ_MASK);
            }
            next_hb = advance_deadline(next_hb, now, HB_PERIOD_S);
        }

        // ---- latest FORCE value to the proxy every 50 ms (R2.3) ---- 
        if (now >= next_force) {
            if (s.have_proxy) {
                int8_t v = s.have_force ? s.force : 0;
                sendto(udp_fd, &v, 1, MSG_DONTWAIT, (struct sockaddr *)&s.proxy, sizeof(s.proxy));
            }
            next_force = advance_deadline(next_force, now, FORCE_PERIOD_S);
        }

        if (!quiet && now >= next_status) {
            print_status(&s, now);
            next_status = now + STATUS_PERIOD_S;
        }
    }

    // On the way out, tell the STM32 the source is gone so it brakes at once
    //instead of waiting for its own timeout. 
    s.drive_enable = false;
    s.bad_input = false;
    struct can_frame frame;
    build_command_frame(&frame, &s, true);
    test_points_toggle(&tp, TP_CMD_TX);
    can_send_both(can_fd, &tp, &frame);

    close(can_fd[0]);
    close(can_fd[1]);
    close(udp_fd);
    test_points_close(&tp);
    printf("Stopped.\n");
    return exit_code;
}
