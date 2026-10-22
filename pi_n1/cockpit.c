#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <linux/gpio.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define UDP_PORT            8000     
#define FORCE_PORT          8001

/* ------------------------------ CONFIG ---------------------------------- */

/* Proxy -> Pi packet: 4-byte counter + 272-byte DIJOYSTATE2 */
#define WHEEL_PACKET_LEN    276
#define OFF_COUNTER         0
#define OFF_LX              4     /* steering */
#define OFF_LY              8     /* throttle */
#define OFF_LRZ             24    /* brake    */
#define OFF_BUTTONS         52    /* rgbButtons[128], 0x80 = pressed */
#define NUM_BUTTONS         128

//raw vals sent by proxy
#define IN_STEER_LEFT       (-32767)
#define IN_STEER_CENTER     0
#define IN_STEER_RIGHT      32767
#define IN_PEDAL_RELEASED   0
#define IN_PEDAL_PRESSED    32767

/* Raw button indices in rgbButtons[]. Override with --btn-lsb/--btn-rsb/--btn-test. */
#define RAW_BTN_LSB         9     /* left blinker  */
#define RAW_BTN_RSB         8     /* right blinker */
#define RAW_BTN_TEST        2     /* self-test (X) */


#define CAL_TOLERANCE_PERCENT 2

//blinker self-cancel
#define TURN_THRESHOLD_PERMILLE    350
#define RETURN_THRESHOLD_PERMILLE  100

//------------------------CAN MESSAGE CATALOGUE----------------------------

#define ID_CTRL 0x010 
#define ID_FAULT 0x020
#define ID_HB_N1 0x100
#define ID_HB_N2 0x101
#define ID_HB_N3 0x102
#define ID_BLINK 0x200
#define ID_FORCE 0x300


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

// ---- copied from the STM32 side's defines (keep in sync) ----

// CAN Bounds 
#define CAN_B0 0                 /* Byte index of flags/sequence field. */
#define CAN_SEQ_MASK 0x0f        /* Low four bits carry sequence 0..15. */

// Flags
#define CAN_FLAG_SYS_ERROR 0x01  /* Team-defined error flag in high nibble. */
#define CTRL_FLAG_UDP_STALE 0x01 /* CTRL flag: upstream wheel/link data stale. */

// Zone Defs
#define ZONE_COCKPIT 1           /* Numeric FAULT source ID for N1. */
#define ZONE_STEERING 2          /* Numeric FAULT source ID for N2. */
#define ZONE_DRIVETRAIN 3        /* Numeric FAULT source ID for N3. */

// CTRL Payload idxs & bounds
#define CTRL_STEER_LO 1          /* CTRL byte index: steering low byte. */
#define CTRL_STEER_HI 2          /* CTRL byte index: steering high byte. */
#define CTRL_THROTTLE 3          /* CTRL byte index: throttle value. */
#define CTRL_BRAKE 4             /* CTRL byte index: brake value. */
#define CTRL_BUTTONS 5           /* CTRL byte index: wheel button bits. */
#define CTRL_RESERVED 6          /* CTRL byte index: must be zero. */
#define CTRL_CRC 7               /* CTRL byte index: CRC over bytes 0..6. */
#define CTRL_STEER_MIN (-1000)     /* Team-defined full-left command. */
#define CTRL_STEER_MAX 1000        /* Team-defined full-right command. */

// Fault Reasons
#define FAULT_ZONE 1             /* FAULT byte index: failing zone ID. */
#define FAULT_REASON 2           /* FAULT byte index: reason code. */
#define FAULT_R_SELF_TEST 1      /* Reason: physical self-test button. */
#define FAULT_R_BAD_INPUT 2      /* Reason: malformed/out-of-range CTRL. */
#define FAULT_R_ZONE_LOST 4      /* Reason: peer heartbeat timeout. */
#define FAULT_R_OUTPUT 6         /* Reason: actuator/output failure. */
#define FAULT_R_SHUTDOWN 7       /* Reason: cockpit shutdown. */

// Heartbeat i
#define HB_STATE 1               /* Heartbeat byte index: zone state. */
#define HB_BUS 2                 /* Heartbeat byte index: bus-health bits. */
#define HB_BUS_A_OK 0x01         /* Bit set when bus A is reachable. */
#define HB_BUS_B_OK 0x02         /* Bit set when bus B is reachable. */

#define BLINK_MODE 1             /* BLINK byte index: selected mode. */
#define BLINK_PHASE 2            /* BLINK byte index: current on/off phase. */
#define BLINK_MODE_OFF 0         /* Neither turn lamp requested. */
#define BLINK_MODE_LEFT 1        /* Left turn lamps requested. */
#define BLINK_MODE_RIGHT 2       /* Right turn lamps requested. */
#define BLINK_MODE_HAZARD 3      /* Both sides requested. */

#define FORCE_VALUE 1            /* FORCE byte index: signed dummy value. */
#define FORCE_MIN (-100)           /* Minimum dummy force value. */
#define FORCE_MAX 100              /* Maximum dummy force value. */


//flags live in the high nibble of byte 0
#define FLAG_HI(f) ((uint8_t)((f) << 4))

//ctrl ranges the defines above leave open (team-defined values) -- add to the STM side
#define CTRL_PEDAL_MAX      100   /* throttle / brake: percent pressed 0..100; above = bad input */
#define BTN_LEFT            0x01
#define BTN_RIGHT           0x02
#define BTN_TEST            0x04
#define BAD_STEER           30000
#define BAD_PEDAL           200

//why the pi is in its current state (internal, for the log); only some go on the bus
enum reason {
    R_NONE = 0, R_TEST_BUTTON = 1, R_HB_TIMEOUT = 2, R_BAD_RANGE = 3, R_BAD_CRC = 4,
    R_UDP_LOSS = 5, R_ISOLATED = 6, R_WATCHDOG = 7, R_STARTUP = 8, R_FAULT_RX = 9,
    R_SHUTDOWN = 10, R_OUTPUT = 11,
};

//internal reason -> FAULT reason code on the bus
static uint8_t fault_reason_code(uint8_t r)
{
    switch (r) {
    case R_TEST_BUTTON: return FAULT_R_SELF_TEST;
    case R_BAD_RANGE:
    case R_BAD_CRC:     return FAULT_R_BAD_INPUT;
    case R_HB_TIMEOUT:
    case R_UDP_LOSS:    return FAULT_R_ZONE_LOST;   /* cockpit stops its heartbeat on UDP loss */
    case R_OUTPUT:      return FAULT_R_OUTPUT;
    case R_SHUTDOWN:    return FAULT_R_SHUTDOWN;
    default:            return FAULT_R_ZONE_LOST;
    }
}

//FAULT reason code from the bus -> name for the log
static const char *fault_reason_str(uint8_t code)
{
    switch (code) {
    case FAULT_R_SELF_TEST: return "self-test";
    case FAULT_R_BAD_INPUT: return "bad-input";
    case FAULT_R_ZONE_LOST: return "zone-lost";
    case FAULT_R_OUTPUT:    return "output";
    case FAULT_R_SHUTDOWN:  return "shutdown";
    default:                return "unknown";
    }
}

static const char *const STATE_NAME[] = { "INIT", "NORMAL", "LOCAL_FAIL", "LINK_LOST", "SYS_ERROR" };

#define STATE_STR(z) ((unsigned)(z) < 5 ? STATE_NAME[z] : "?")

static const char *const REASON_NAME[] = {
    "none", "test-button", "hb-timeout", "bad-range", "bad-crc", "udp-loss",
    "isolated", "watchdog", "startup", "fault-rx", "shutdown", "output"
};

//safe for if undiagnosed
#define REASON_STR(r) ((unsigned)(r) < sizeof(REASON_NAME) / sizeof(REASON_NAME[0]) ? REASON_NAME[r] : "unknown")

//------------------------------TIMING PARAMS (s)--------------------------------

#define HB_PERIOD_S         0.020 
#define HB_TIMEOUT_S        0.060 
#define UDP_TIMEOUT_S       0.100 
#define CTRL_BACKSTOP_S     0.045
#define FORCE_PERIOD_S      0.050
#define FORCE_STALE_S       0.150
#define SPORADIC_MIN_S      0.020  
#define BLINK_HALF_S        0.500 
#define HAZARD_HALF_S       0.250 
#define FAULT_HOLD_S        0.100 
#define TEST_DEBOUNCE_S     0.050
#define DOUBLE_PRESS_S      0.500
#define DUP_WINDOW_S        0.010 
#define STALE_WINDOW_S      0.100
#define MON_PERIOD_S        0.005

//------------------------------------TESTPOINTS AND LEDS------------------------

enum { TP_UDP_RX, TP_CMD_TX, TP_CAN_TX_A,
       TP_CAN_TX_B, TP_DEG_A, TP_DEG_B, TP_ERR, TP_COUNT };

static const char *const TP_NAME[TP_COUNT] = {
     "udp_rx", "cmd_tx", "can_tx_a", "can_tx_b", "deg_a", "deg_b", "err" };


static unsigned TP_PIN[TP_COUNT] = {
    17,   /* UDP_RX    header pin 11 */
    27,   /* CMD_TX    header pin 13 */
    22,   /* CAN_TX_A  header pin 15 */
    23,   /* CAN_TX_B  header pin 16 */
    5,    /* DEG_A LED header pin 29 */
    6,    /* DEG_B LED header pin 31 */
    26,   /* ERR LED   header pin 37 */
};    
/* ------------------------------ STATE ----------------------------------- */

typedef struct {
    int32_t steer_left, steer_center, steer_right;
    int32_t throttle_released, throttle_pressed;
    int32_t brake_released, brake_pressed;
    int btn_lsb, btn_rsb, btn_test;
} calibration_t;

typedef struct {
    bool     enabled;
    int      fd;                 /* line-request fd from the GPIO chip */
    char     chip[PATH_MAX];
    bool     level[TP_COUNT];
} test_points_t;


typedef struct {
    const char *ifname;
    int fd;
    uint64_t tx_ok, tx_fail, rx, err_frames;
    int last_errno;

    bool busoff;
    double last_traffic;
    double last_hb[4];
} canbus_t;


//dup struct
typedef struct{
    double seen[16];
    uint8_t last_seq;
    double last_t;

} dedup_t;

enum { SF_TEST = 1, SF_UDP = 2 };    /* why the cockpit has failed itself */
enum { INJ_NONE, INJ_STEER, INJ_THROTTLE, INJ_CRC };



typedef struct {
    calibration_t cal;
    canbus_t bus[2];                 // [0] for CAN A [1] for CAN_B
    int      udp_fd, force_fd;
    struct sockaddr_in proxy;
    bool     have_proxy;
    test_points_t tp;
    int      force_gain;


    //latest wheel input
    int32_t  in_lx, in_ly, in_lrz;
    uint8_t  in_buttons[NUM_BUTTONS];

    int16_t steer;
    uint8_t throttle, brake;
    uint8_t buttons;

    bool     have_rx, have_seq;
    double   last_rx;
    uint32_t last_seq;

    /* blinkers */
    uint8_t  user_mode;              /* from the buttons */
    bool     turn_crossed, lsb_prev, rsb_prev, btn_prev_known;
    uint8_t  tx_mode;                /* what the bus was last told */
    bool     lamps, blink_due;
    double   next_toggle, last_blink_tx;

    bool     test_prev_known, test_prev_down;
    double   test_last_press, restore_first;
    

    //zone state
    uint8_t state, reason;
    unsigned self_fail;
    bool ever_normal;
    double fault_hold_until;
    bool ever_heard[4], zone_alive[4], zone_failed[4];
    double fault_pending[4];
    bool deg[2], isolated;
    uint8_t zone_hb_state[4], zone_hb_health[4];

    //sequence counters and dedup per id
    uint8_t  seq_ctrl, seq_fault, seq_hb, seq_blink;
    dedup_t  dd_hb2, dd_hb3, dd_force;
    uint64_t dup_dropped;

    //fault transmit queue
    uint8_t  fault_q[8][2];          /* failing zone, reason */
    unsigned fault_q_len;
    double   last_fault_tx;

    /* force feedback */
    int8_t   force;
    double   force_rx;

    /* bad-input injection */
    int      inject_next;
    bool     inject_persistent;
 

    //schedule
    double next_hb, next_mon, next_force, last_ctrl_tx;

    //measurement 
    uint32_t lat_hist[20001];
    uint64_t lat_n, ctrl_sent, udp_packets;
    double   lat_max, gap_min, gap_max;


} state_t;

static volatile sig_atomic_t g_running = 1;
static void on_signal(int sig) { (void)sig; g_running = 0; }

/* ------------------------------ UTIL ------------------------------------ */

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static double g_t0;
#define EVT(t, ...) do { printf("[%9.3f] ", (t) - g_t0); printf(__VA_ARGS__); printf("\n"); } while (0)



static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}



/* Keep the send schedule on a fixed 20 ms grid: a late send does not shift later ones. */
static double advance_deadline(double deadline, double now, double period)
{
    if (now < deadline) return deadline;
    return deadline + ((double)(long)((now - deadline) / period) + 1.0) * period;
}

static bool fresh(double t, double now, double limit) { return t > 0 && now - t <= limit; }
 
//CRC-8/SAE-J1850: poly 0x1D, init 0xFF, final XOR 0xFF. 
static uint8_t crc8(const uint8_t *d, size_t n) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
    }
    return crc ^ 0xFF;
}

/* --------------------------- AXIS CONVERSION ---------------------------- */

/* Straight-line map: in_a -> out_a, in_b -> out_b. Works for either direction
 * of travel. Slightly past an end clamps to that end; far past an end is
 * extrapolated, which lands outside 0..65535 and is rejected by the zones. */
static int32_t map_axis(int32_t raw, int32_t in_a, int32_t in_b, int32_t out_a, int32_t out_b)
{
    int64_t span = (int64_t)in_b - in_a;
    int64_t pos = (int64_t)raw - in_a;
    if (span < 0) { span = -span; pos = -pos; }
    int64_t tolerance = span * CAL_TOLERANCE_PERCENT / 100;
    if (pos < 0 && pos >= -tolerance) pos = 0;
    if (pos > span && pos <= span + tolerance) pos = span;
    int64_t out = out_a + pos * ((int64_t)out_b - out_a) / span;
    if (out > INT32_MAX) out = INT32_MAX;
    if (out < INT32_MIN) out = INT32_MIN;
    return (int32_t)out;
}

static int16_t map_steering(int32_t raw, const calibration_t *c)
{
    bool rising = c->steer_right > c->steer_left;
    bool left_half = rising ? raw < c->steer_center : raw > c->steer_center;
    int32_t v = left_half ? map_axis(raw, c->steer_left, c->steer_center, CTRL_STEER_MIN, 0)
                : map_axis(raw, c->steer_center, c->steer_right, 0, CTRL_STEER_MAX);    
    return (int16_t)(v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : v);

}
static int32_t map_pedal(int32_t raw, int32_t released, int32_t pressed)
{
    int32_t v = map_axis(raw, released, pressed, 0, CTRL_PEDAL_MAX);
    if (v < 0) return 0xFF;           /* below released: out of range on purpose */
    return (uint8_t)(v > 254 ? 254 : v);
}

/* ---------------------------- TEST POINTS ------------------------------- */

/* Kernel GPIO character device directly (no libgpiod). Finds the header GPIO
 * chip by label and by the line names GPIOnn, requests all lines as outputs. */

static int test_points_open(test_points_t *tp)
{
    char seen[16][PATH_MAX];
    int seen_count = 0;

    tp->fd = -1;
    for (int i = 0; i < 16; i++) {
        char path[32], real[PATH_MAX];
        snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
        if (!realpath(path, real)) continue;

        bool duplicate = false;          /* Pi 5 has symlinked aliases */
        for (int k = 0; k < seen_count; k++)
            if (strcmp(seen[k], real) == 0) duplicate = true;
        if (duplicate) continue;
        snprintf(seen[seen_count++], PATH_MAX, "%s", real);

        int chip = open(real, O_RDWR | O_CLOEXEC);
        if (chip < 0) continue;

        struct gpiochip_info info;
        memset(&info, 0, sizeof(info));
        bool match = ioctl(chip, GPIO_GET_CHIPINFO_IOCTL, &info) == 0 && info.lines >= 28 &&
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


        struct gpio_v2_line_request req;
        memset(&req, 0, sizeof(req));

        for (int p = 0; p < TP_COUNT; p++) req.offsets[p] = TP_PIN[p];
        req.num_lines = TP_COUNT;
        snprintf(req.consumer, sizeof(req.consumer), "lab3-cockpit");

       
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
    fprintf(stderr, "Pi header GPIO chip not found (use --no-test-points to run without them)\n");
    return -1;
}

static void test_points_set(test_points_t *tp, int which, bool level)
{
    if (tp->level[which] == level) return;
    tp->level[which] = level;
    if (!tp->enabled || tp->fd < 0) return;
    struct gpio_v2_line_values v = { .bits = level ? (1ULL << which) : 0, .mask = 1ULL << which };
    if (ioctl(tp->fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v) != 0) perror("test point write");
 }

static void test_points_toggle(test_points_t *tp, int which) {
     test_points_set(tp, which, !tp->level[which]); }

static void test_points_close(test_points_t *tp)
{
    if (tp->fd >= 0) {
        struct gpio_v2_line_values v = { .bits = 0, .mask = (1ULL << TP_COUNT) - 1 };
        if (tp->enabled) (void)ioctl(tp->fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &v);
        close(tp->fd);
    }
    tp->fd = -1;
}

/* ------------------------------- CAN ----------------------------------- */

static int can_open(canbus_t *b)
{
    b->fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (b->fd < 0) { perror("socket(PF_CAN)"); return -1; }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", b->ifname);
    if (ioctl(b->fd, SIOCGIFINDEX, &ifr) != 0) {
        fprintf(stderr, "%s: no such interface (config.txt overlays? ip link set %s up ...)\n",
               b->ifname, b->ifname);
        return -1;
    }
    /* Standard-ID data frames, plus error frames for bus-off / error-passive. */
    struct can_filter f = { .can_id = 0, .can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG };
    can_err_mask_t err = CAN_ERR_BUSOFF | CAN_ERR_CRTL | CAN_ERR_RESTARTED;
    setsockopt(b->fd, SOL_CAN_RAW, CAN_RAW_FILTER, &f, sizeof(f));
    setsockopt(b->fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err, sizeof(err));

    struct sockaddr_can addr = { .can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex };
    if (bind(b->fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) { perror("bind(can)"); return -1; }
    return 0;

}

static void can_write(canbus_t *b, uint32_t id, const void *d)
{
    struct can_frame f;
    memset(&f, 0, sizeof(f));
    f.can_id = id;
    f.can_dlc = 8;
    memcpy(f.data, d, 8);
    if (send(b->fd, &f, sizeof(f), MSG_DONTWAIT) == (ssize_t)sizeof(f)) b->tx_ok++;
    else { b->tx_fail++; b->last_errno = errno; }
}


static void send_both(state_t *s, uint32_t id, const void *d)
{
    can_write(&s->bus[0], id, d);
    test_points_toggle(&s->tp, TP_CAN_TX_A);
    can_write(&s->bus[1], id, d);
    test_points_toggle(&s->tp, TP_CAN_TX_B);
}

//byte 0: this id's sequence counter in the low nibble, flags in the high nibble
static uint8_t byte0(uint8_t *counter, uint8_t flags)
{
    uint8_t b = (uint8_t)((*counter & CAN_SEQ_MASK) | flags);
    *counter = (uint8_t)((*counter + 1) & CAN_SEQ_MASK);
    return b;
}

//CAN_FLAG_SYS_ERROR on every frame except CTRL while the pi is not NORMAL
static uint8_t err_flag(const state_t *s)
{
    return s->state == ZONE_STATE_NORMAL ? 0 : FLAG_HI(CAN_FLAG_SYS_ERROR);
}



static bool dedup_accept(dedup_t *d, uint8_t seq, double now)
{
    seq &= CAN_SEQ_MASK;
    if (d->seen[seq] > 0 && now - d->seen[seq] < DUP_WINDOW_S) return false;
    if (d->last_t > 0 && now - d->last_t < STALE_WINDOW_S) {
        uint8_t behind = (uint8_t)((d->last_seq - seq) & CAN_SEQ_MASK);
        if (behind >= 1 && behind <= 7) return false;
    }
    d->seen[seq] = now;
    d->last_seq = seq;
    d->last_t = now;
    return true;
}



//------------------------------TRANSMIT TASKS------------------------------

/*
 * CTRL 0x010 -- N1 -> N2, N3, on every UDP update. struct ctrl_payload:
 *   flags_seq  seq | CTRL_FLAG_UDP_STALE when this is a resend without a new UDP update
 *   steering   int16 LE, per mille: -1000 full left .. +1000 full right
 *   throttle   percent pressed 0..100
 *   brake      percent pressed 0..100
 *   buttons    BTN_LEFT | BTN_RIGHT | BTN_TEST
 *   reserved   0
 *   crc8       CRC-8/SAE-J1850 (poly 0x1D, init 0xFF, xor 0xFF) over bytes 0..6
 */
static void ctrl_tx(state_t *s, double now, bool stale)
{
    int16_t steer = s->steer;
    uint8_t throttle = s->throttle, brake = s->brake;
    if (s->inject_persistent || s->inject_next == INJ_STEER) steer = BAD_STEER;
    if (s->inject_next == INJ_THROTTLE) throttle = BAD_PEDAL;

    struct ctrl_payload p;
    memset(&p, 0, sizeof(p));
    p.flags_seq = byte0(&s->seq_ctrl, stale ? FLAG_HI(CTRL_FLAG_UDP_STALE) : 0);
    p.steering  = steer;
    p.throttle  = throttle;
    p.brake     = brake;
    p.buttons   = s->buttons;
    p.reserved  = 0;
    p.crc8 = crc8((const uint8_t *)&p, CTRL_CRC);   /* bytes 0..6 */
    if (s->inject_next == INJ_CRC) p.crc8 ^= 0x5A;
    if (s->inject_next != INJ_NONE) {
        EVT(now, "injected bad CTRL (%s)", s->inject_next == INJ_STEER ? "steering out of range" :
            s->inject_next == INJ_THROTTLE ? "throttle out of range" : "bad CRC");
        s->inject_next = INJ_NONE;
    }

    send_both(s, ID_CTRL, &p);
    test_points_toggle(&s->tp, TP_CMD_TX);

    if (s->ctrl_sent) {
        double gap = now - s->last_ctrl_tx;
        if (s->gap_min == 0 || gap < s->gap_min) s->gap_min = gap;
        if (gap > s->gap_max) s->gap_max = gap;
    }
    s->last_ctrl_tx = now;
    s->ctrl_sent++;
}

/*
 * FAULT 0x020 -- any -> all, on a failure, never closer than 20 ms.
 * struct fault_payload:
 *   flags_seq        seq | flags
 *   failing_zone_id  1..3
 *   reason_code      FAULT_R_* (fault_reason_code())
 *   reserved[5]      0
 */
static void fault_enqueue(state_t *s, uint8_t zone, uint8_t reason)
{
    if (s->fault_q_len < 8) { s->fault_q[s->fault_q_len][0] = zone; s->fault_q[s->fault_q_len][1] = reason; s->fault_q_len++; }
}

static void fault_tx(state_t *s, double now)
{
    while (s->fault_q_len > 0) {
        if (s->last_fault_tx > 0 && now - s->last_fault_tx < SPORADIC_MIN_S) return;
        uint8_t zone = s->fault_q[0][0], reason = s->fault_q[0][1];
        memmove(s->fault_q[0], s->fault_q[1], (s->fault_q_len - 1) * sizeof(s->fault_q[0]));
        s->fault_q_len--;
        /* A cockpit failed by its test button only announces its own failure. */
        if ((s->self_fail & SF_TEST) && zone != ZONE_COCKPIT) continue;

        struct fault_payload p;
        memset(&p, 0, sizeof(p));
        p.flags_seq = byte0(&s->seq_fault, err_flag(s));
        p.failing_zone_id = zone;
        p.reason_code = fault_reason_code(reason);
        send_both(s, ID_FAULT, &p);
        s->last_fault_tx = now;
        EVT(now, "FAULT tx: zone %u %s", zone, REASON_STR(reason));
        return;
    }
}

/*
 * HB_N1 0x100 -- N1 -> N2, N3, every 20 ms, not while the cockpit has failed. struct hb_payload:
 *   flags_seq    seq | flags
 *   zone_state   enum zone_state
 *   bus_health   HB_BUS_A_OK | HB_BUS_B_OK  (bit set = that bus is healthy at N1)
 *   reserved[5]  0
 */
static void hb_tx(state_t *s, double now)
{
    (void)now;
    if (s->self_fail) return;

    struct hb_payload p;
    memset(&p, 0, sizeof(p));
    p.flags_seq = byte0(&s->seq_hb, err_flag(s));
    p.zone_state = s->state;
    p.bus_health = (uint8_t)((s->deg[0] || s->isolated ? 0 : HB_BUS_A_OK) |
                             (s->deg[1] || s->isolated ? 0 : HB_BUS_B_OK));
    send_both(s, ID_HB_N1, &p);
}

/*
 * BLINK 0x200 -- N1 -> N2, N3. The cockpit is the blinker master: it sends
 * every lamp edge and N2/N3 set their lamps when the frame arrives, so the
 * front/rear offset is only the difference of their receive paths (R2.4).
 * struct blink_payload:
 *   flags_seq    seq | flags
 *   mode         0 off, 1 left, 2 right, 3 hazard
 *   phase        1 = lamps on, 0 = lamps off
 *   reserved[5]  0
 */
static double half_period(uint8_t mode) { return mode == BLINK_MODE_HAZARD ? HAZARD_HALF_S : BLINK_HALF_S; }

static void blink_tx(state_t *s, double now)
{
    if (s->self_fail & SF_TEST) return;            /* failed zone: silent */

    uint8_t m = s->state == ZONE_STATE_NORMAL ? s->user_mode : BLINK_MODE_HAZARD;
    if (m != s->tx_mode) {
        s->tx_mode = m;
        s->lamps = m != BLINK_MODE_OFF;                  /* new mode starts with lamps on */
        s->next_toggle = now + half_period(m);
        s->blink_due = true;
    } else if (m != BLINK_MODE_OFF && now >= s->next_toggle) {
        s->lamps = !s->lamps;
        s->next_toggle = advance_deadline(s->next_toggle, now, half_period(m));
        s->blink_due = true;
    } else if (m == BLINK_MODE_OFF && now - s->last_blink_tx >= BLINK_HALF_S) {
        s->blink_due = true;                        /* refresh while off */
    }
    if (!s->blink_due || (s->last_blink_tx > 0 && now - s->last_blink_tx < SPORADIC_MIN_S)) return;

    struct blink_payload p;
    memset(&p, 0, sizeof(p));
    p.flags_seq = byte0(&s->seq_blink, err_flag(s));
    p.mode = s->tx_mode;
    p.phase = s->lamps ? 1 : 0;
    send_both(s, ID_BLINK, &p);
    s->last_blink_tx = now;
    s->blink_due = false;
}

/* Force to the proxy: one signed byte, -100..100, every 50 ms (R2.3). */
static void force_udp(state_t *s, double now)
{
    if (!s->have_proxy) return;
    int v = 0;
    if (!(s->self_fail & SF_TEST) && fresh(s->force_rx, now, FORCE_STALE_S)) v = s->force * s->force_gain / 100;
    int8_t byte = (int8_t)(v > 100 ? 100 : v < -100 ? -100 : v);
    struct sockaddr_in to = s->proxy;
    to.sin_port = htons(FORCE_PORT);
    (void)sendto(s->force_fd, &byte, 1, MSG_DONTWAIT, (struct sockaddr *)&to, sizeof(to));
}
//-----------------------------STATE MACHINE--------------------------------

static void set_self_fail(state_t *s, unsigned bit, bool on, uint8_t reason, double now)
{
    unsigned before = s->self_fail;
    if (on) s->self_fail |= bit; else s->self_fail &= ~bit;
    if (before == s->self_fail) return;
    if (on) {
        EVT(now, "cockpit FAILED (%s): heartbeat stopped", REASON_STR(reason));
        fault_enqueue(s, ZONE_COCKPIT, reason);     /* announce once, then go quiet */
        fault_tx(s, now);
    } else {
        EVT(now, "cockpit restored (%s cleared)", bit == SF_TEST ? "test button" : "udp loss");
    }
}

static void update_state(state_t *s, double now)
{
    bool all_alive = s->zone_alive[ZONE_STEERING] && s->zone_alive[ZONE_DRIVETRAIN];
    bool udp_ok = fresh(s->last_rx, now, UDP_TIMEOUT_S);
    bool hold = now < s->fault_hold_until;
    uint8_t st, why;

    if (s->self_fail & SF_TEST) {
        st = ZONE_STATE_LOCAL_FAIL;                /* own test button */
        why = R_TEST_BUTTON;
    } else if (s->self_fail) {
        st = ZONE_STATE_LINK_LOST;                 /* no UDP for 100 ms */
        why = R_UDP_LOSS;
    } else if (all_alive && udp_ok && !hold) {
        st = ZONE_STATE_NORMAL;
        why = R_NONE;
    } else {
        bool lost = s->isolated || s->zone_failed[2] || s->zone_failed[3];
        why = s->isolated ? R_ISOLATED
            : lost ? R_HB_TIMEOUT
            : hold ? R_FAULT_RX
            : !s->ever_normal ? R_STARTUP : R_HB_TIMEOUT;
        st = !s->ever_normal ? ZONE_STATE_INIT     /* power-up stays INIT */
           : lost ? ZONE_STATE_LINK_LOST           /* 3 HBs missed on both buses */
           : ZONE_STATE_SYS_ERROR;                 /* FAULT received */
    }
    if (st == ZONE_STATE_NORMAL) s->ever_normal = true;
    if (st != s->state) EVT(now, "STATE %s -> %s (%s)", STATE_STR(s->state), STATE_STR(st), REASON_STR(why));
    s->state = st;
    s->reason = why;

    test_points_set(&s->tp, TP_ERR, s->state != ZONE_STATE_NORMAL);
    test_points_set(&s->tp, TP_DEG_A, s->deg[0]);
    test_points_set(&s->tp, TP_DEG_B, s->deg[1]);
}

/* HB_MON, every 5 ms. */
static void hb_mon(state_t *s, double now)
{
    /* Lab 1: no UDP update for 100 ms -> the cockpit treats itself as failed. */
    set_self_fail(s, SF_UDP, !fresh(s->last_rx, now, UDP_TIMEOUT_S), R_UDP_LOSS, now);

    /* R4.7: degraded = nothing from any other node on this bus for 60 ms
     * while the other bus still has traffic. */
    bool traffic[2];
    for (int b = 0; b < 2; b++) traffic[b] = fresh(s->bus[b].last_traffic, now, HB_TIMEOUT_S);
    for (int b = 0; b < 2; b++) {
        bool d = !traffic[b] && traffic[!b];
        if (d != s->deg[b]) EVT(now, "DEG_%c %s", b ? 'B' : 'A', d ? "ON" : "off");
        s->deg[b] = d;
    }

    /* Isolated: no heartbeat from any zone on either bus. */
    bool any = false;
    for (int z = 2; z <= 3; z++)
        for (int b = 0; b < 2; b++) any |= fresh(s->bus[b].last_hb[z], now, HB_TIMEOUT_S);
    bool iso = !any && (s->ever_heard[2] || s->ever_heard[3]);
    if (iso && !s->isolated) EVT(now, "ISOLATED: no heartbeats on either bus");
    s->isolated = iso;

    /* R4.5: failed = three heartbeats missed on BOTH buses. */
    for (int z = 2; z <= 3; z++) {
        bool alive = fresh(s->bus[0].last_hb[z], now, HB_TIMEOUT_S) || fresh(s->bus[1].last_hb[z], now, HB_TIMEOUT_S);
        s->zone_alive[z] = alive;
        if (alive) {
            s->zone_failed[z] = false;
            s->fault_pending[z] = 0;
        } else if (s->ever_heard[z] && !s->zone_failed[z]) {
            s->zone_failed[z] = true;
            s->fault_pending[z] = now;
            EVT(now, "zone %d FAILED: 3 heartbeats missed on both buses", z);
        }
    }
    /* Send FAULT for a failed zone only once the other zone's heartbeat has
     * arrived after the failure (proves we are still on a bus). If the cockpit
     * itself is isolated, the FAULT is dropped instead of waiting in a TX
     * buffer until the jumper goes back in. */
    for (int z = 2; z <= 3; z++) {
        if (s->fault_pending[z] == 0) continue;
        int o = z == 2 ? 3 : 2;
        if (s->bus[0].last_hb[o] > s->fault_pending[z] || s->bus[1].last_hb[o] > s->fault_pending[z]) {
            fault_enqueue(s, (uint8_t)z, R_HB_TIMEOUT);
            s->fault_pending[z] = 0;
        } else if (s->zone_failed[o] || s->isolated) {
            s->fault_pending[z] = 0;
        }
    }
    update_state(s, now);
}

//----------------------------CAN_RX----------------------------------------

/*
 * One dispatch path for both buses. Received payloads:
 *   HB_N2/HB_N3  struct hb_payload    zone_state, bus_health (bit = bus OK)
 *   FAULT        struct fault_payload failing_zone_id, reason_code
 *   FORCE 0x300  struct force_payload force (dummy counter this lab, clamped to +-100)
 */
static void can_frame_rx(state_t *s, int bus, const struct can_frame *f, double now)
{
    canbus_t *b = &s->bus[bus];
    if (f->can_id & CAN_ERR_FLAG) {
        b->err_frames++;
        if ((f->can_id & CAN_ERR_BUSOFF) && !b->busoff) { b->busoff = true; EVT(now, "%s: BUS-OFF", b->ifname); }
        if (f->can_id & CAN_ERR_RESTARTED) { b->busoff = false; EVT(now, "%s: restarted", b->ifname); }
        return;
    }
    if (f->can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG) || f->can_dlc != 8) return;
    uint32_t id = f->can_id & CAN_SFF_MASK;

    b->rx++;
    b->last_traffic = now;                         /* any frame from another node, relayed ones too */
    b->busoff = false;

    switch (id) {
    case ID_HB_N2:
    case ID_HB_N3: {
        struct hb_payload hb;
        memcpy(&hb, f->data, sizeof(hb));
        int z = (int)(id - ID_HB_N1) + 1;
        b->last_hb[z] = now;                       /* per bus, before dedup */
        s->ever_heard[z] = true;
        s->zone_alive[z] = true;
        if (!dedup_accept(z == 2 ? &s->dd_hb2 : &s->dd_hb3, hb.flags_seq, now)) { s->dup_dropped++; break; }
        s->zone_hb_state[z] = hb.zone_state;
        s->zone_hb_health[z] = hb.bus_health;
        break;
    }
    case ID_FAULT: {
        /* No dedup: FAULT has no sender field, so two zones' FAULTs can share
         * a seq; acting on a copy twice only extends the hold. */
        struct fault_payload fp;
        memcpy(&fp, f->data, sizeof(fp));
        EVT(now, "FAULT rx on %s: zone %u %s", b->ifname, fp.failing_zone_id, fault_reason_str(fp.reason_code));
        s->fault_hold_until = now + FAULT_HOLD_S;
        update_state(s, now);                      /* error state now, not at the next tick */
        break;
    }
    case ID_FORCE: {
        struct force_payload fp;
        memcpy(&fp, f->data, sizeof(fp));
        if (!dedup_accept(&s->dd_force, fp.flags_seq, now)) { s->dup_dropped++; break; }
        int v = fp.force;
        s->force = (int8_t)(v > FORCE_MAX ? FORCE_MAX : v < FORCE_MIN ? FORCE_MIN : v);
        s->force_rx = now;
        break;
    }
    default:                                       /* our own IDs coming back through the relay */
        break;
    }
}

static void can_rx(state_t *s, int bus)
{
    for (int i = 0; i < 64; i++) {
        struct can_frame f;
        if (recv(s->bus[bus].fd, &f, sizeof(f), MSG_DONTWAIT) != (ssize_t)sizeof(f)) return;
        can_frame_rx(s, bus, &f, now_s());
    }
}


/* --------------------------- WHEEL PACKETS ------------------------------ */

static bool button_down(const state_t *s, int index)
{
    return index >= 0 && index < NUM_BUTTONS && (s->in_buttons[index] & 0x80);
}

/* Cockpit test button: single press fails the zone, two presses within 500 ms
 * restore it; presses closer than 50 ms are one press (R2.5, R3). */

static void self_test_press(state_t *s, double now)
{
    if (s->test_last_press > 0 && now - s->test_last_press < TEST_DEBOUNCE_S) return;
    s->test_last_press = now;
    if (!(s->self_fail & SF_TEST)) {
        s->restore_first = 0;
        set_self_fail(s, SF_TEST, true, R_TEST_BUTTON, now);
    } else if (s->restore_first > 0 && now - s->restore_first <= DOUBLE_PRESS_S) {
        s->restore_first = 0;
        set_self_fail(s, SF_TEST, false, R_NONE, now);
    } else {
        s->restore_first = now;
    }
    update_state(s, now);                          /* ERR within 10 ms */
}


static void blinker_input(state_t *s, bool resumed)
{
    bool l = button_down(s, s->cal.btn_lsb), r = button_down(s, s->cal.btn_rsb);
    if (resumed) s->btn_prev_known = false;        /* a held button is not a press */
    if (s->btn_prev_known && l && !s->lsb_prev) {
        s->user_mode = s->user_mode == BLINK_MODE_LEFT ? BLINK_MODE_OFF : BLINK_MODE_LEFT;
        s->turn_crossed = false;
    } else if (s->btn_prev_known && r && !s->rsb_prev) {
        s->user_mode = s->user_mode == BLINK_MODE_RIGHT ? BLINK_MODE_OFF : BLINK_MODE_RIGHT;
        s->turn_crossed = false;
    }
    s->lsb_prev = l;
    s->rsb_prev = r;
    s->btn_prev_known = true;

    if (s->user_mode == BLINK_MODE_LEFT) {
        if (s->steer <= -TURN_THRESHOLD_PERMILLE) s->turn_crossed = true;
        else if (s->turn_crossed && s->steer >= -RETURN_THRESHOLD_PERMILLE) { s->user_mode = BLINK_MODE_OFF; s->turn_crossed = false; }
    } else if (s->user_mode == BLINK_MODE_RIGHT) {
        if (s->steer >= TURN_THRESHOLD_PERMILLE) s->turn_crossed = true;
        else if (s->turn_crossed && s->steer <= RETURN_THRESHOLD_PERMILLE) { s->user_mode = BLINK_MODE_OFF; s->turn_crossed = false; }
    }
}






/* Take one 276-byte proxy packet. Returns true if it was accepted as newer. */
static bool wheel_packet(state_t *s, const uint8_t *d, ssize_t n, double now, bool *resumed){
    if (n != WHEEL_PACKET_LEN) return false;

    /* Accept the first packet, a packet after a long silence (a restarted
     * proxy), or any packet newer than the last one. Drop duplicates and
     * reordered packets. */
    uint32_t counter = rd_u32le(&d[OFF_COUNTER]);
    uint32_t delta = counter - s->last_seq;
    *resumed = !s->have_rx || now - s->last_rx > UDP_TIMEOUT_S;
    if (s->have_seq && !*resumed && !(delta > 0 && delta < 0x80000000u)) return false;


    s->in_lx = (int32_t)rd_u32le(&d[OFF_LX]);
    s->in_ly = (int32_t)rd_u32le(&d[OFF_LY]);
    s->in_lrz = (int32_t)rd_u32le(&d[OFF_LRZ]);
    memcpy(s->in_buttons, &d[OFF_BUTTONS], NUM_BUTTONS);

    s->steer = map_steering(s->in_lx, &s->cal);
    s->throttle = map_pedal(s->in_ly, s->cal.throttle_released, s->cal.throttle_pressed);
    s->brake = map_pedal(s->in_lrz, s->cal.brake_released, s->cal.brake_pressed);
    s->buttons = (uint8_t)((button_down(s, s->cal.btn_lsb) ? BTN_LEFT : 0) |
                           (button_down(s, s->cal.btn_rsb) ? BTN_RIGHT : 0) |
                           (button_down(s, s->cal.btn_test) ? BTN_TEST : 0));

   
    s->last_seq = counter;
    s->have_seq = true;
    s->last_rx = now;
    s->have_rx = true;
    return true;
}



/* UDP_RX -> CTRL_TX: the brake path, serviced first in the loop. */
static void udp_rx(state_t *s, const struct in_addr *pc_addr)
{
    for (int i = 0; i < 16; i++) {
        uint8_t data[2048];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t n = recvfrom(s->udp_fd, data, sizeof(data), MSG_DONTWAIT, (struct sockaddr *)&from, &from_len);
        if (n < 0) return;
        double t_rx = now_s();
        if (pc_addr && from.sin_addr.s_addr != pc_addr->s_addr) continue;
        test_points_toggle(&s->tp, TP_UDP_RX);
        s->udp_packets++;

        bool resumed = false;
        if (!wheel_packet(s, data, n, t_rx, &resumed)) continue;
        s->proxy = from;
        s->have_proxy = true;
        if (s->self_fail & SF_UDP) { set_self_fail(s, SF_UDP, false, R_NONE, t_rx); update_state(s, t_rx); }

        if (!(s->self_fail & SF_TEST)) {           /* R4.4: CTRL on every UDP update */
            ctrl_tx(s, t_rx, false);
            double us = (now_s() - t_rx) * 1e6;
            s->lat_hist[us > 20000 ? 20000 : (int)us]++;
            s->lat_n++;
            if (us > s->lat_max) s->lat_max = us;
        }

        blinker_input(s, resumed);
        bool down = button_down(s, s->cal.btn_test);
        if (resumed) s->test_prev_known = false;
        bool rising = s->test_prev_known && !s->test_prev_down && down;
        s->test_prev_known = true;
        s->test_prev_down = down;
        if (rising) self_test_press(s, t_rx);
        blink_tx(s, now_s());                      /* a new blinker mode goes out now */
    }
}

/* ------------------------------ STATUS ---------------------------------- */

static void print_stats(const state_t *s)
{
    long p50 = -1, p99 = -1;
    uint64_t acc = 0;
    for (int i = 0; i <= 20000 && s->lat_n; i++) {
        acc += s->lat_hist[i];
        if (p50 < 0 && acc * 2 >= s->lat_n) p50 = i;
        if (p99 < 0 && acc * 100 >= s->lat_n * 99) p99 = i;
    }
    printf("---- UDP read -> CTRL queued (scope UDP_RX -> CMD_TX for the report) ----\n");
    printf("  n=%llu p50=%ld us p99=%ld us max=%.0f us | CTRL gap min=%.2f max=%.2f ms | UDP=%llu CTRL=%llu\n",
           (unsigned long long)s->lat_n, p50, p99, s->lat_max, s->gap_min * 1e3, s->gap_max * 1e3,
           (unsigned long long)s->udp_packets, (unsigned long long)s->ctrl_sent);
}

/* ------------------------------- MAIN ----------------------------------- */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  --can-a IF            CAN A interface (default can0, SPI0 CE0)\n"
        "  --can-b IF            CAN B interface (default can1, SPI0 CE1)\n"
        "  --bind IP             local address for UDP port %d (default 0.0.0.0)\n"
        "  --pc-ip IP            accept wheel packets only from this address (default: any)\n"
        "  --steer L,C,R         raw steering at full left, center, full right (default %d,%d,%d)\n"
        "  --throttle REL,PRESS  raw throttle released, fully pressed (default %d,%d)\n"
        "  --brake REL,PRESS     raw brake released, fully pressed (default %d,%d)\n"
        "  --btn-lsb N           raw button index for the left blinker (default %d)\n"
        "  --btn-rsb N           raw button index for the right blinker (default %d)\n"
        "  --btn-test N          raw button index for the self-test button (default %d)\n"
        "  --force-gain PCT      scale the force sent to the proxy, 0 = off (default 100)\n"
        "  --gpio NAME=BCM       move a test point (udp_rx cmd_tx can_tx_a can_tx_b deg_a deg_b err)\n"
        "  --rt PRIO             SCHED_FIFO priority, below 50 (the mcp251x IRQ threads)\n"
        "  --cpu N               pin to one CPU\n"
        "  --no-test-points      run without the GPIO test points / LEDs\n"
        "Keys (+ Enter): b/t/c next CTRL bad steer/throttle/CRC, B bad steer on/off,\n"
        "                x test button, s latency stats, q quit\n",
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

static bool parse_gpio(const char *arg)
{
    char name[32];
    unsigned pin;
    if (sscanf(arg, "%31[^=]=%u", name, &pin) != 2 || pin > 27) return false;
    for (int i = 0; i < TP_COUNT; i++)
        if (strcmp(name, TP_NAME[i]) == 0) { TP_PIN[i] = pin; return true; }
    return false;
}

/*
 * Bus bring-up, once per boot (Lab 3 Appendix B):
 *   /boot/firmware/config.txt (oscillator = the crystal on the module, then reboot):
 *     dtparam=spi=on
 *     dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
 *     dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=24
 *   sudo ip link set can0 up type can bitrate 500000 restart-ms 50
 *   sudo ip link set can1 up type can bitrate 500000 restart-ms 50
 * restart-ms gives automatic bus-off recovery (R4.8: within 100 ms).
 */
int main(int argc, char **argv)
{
    static state_t st;                 /* large (histogram): not on the stack */
    state_t *s = &st;
    const char *bind_ip = "0.0.0.0", *pc_ip = NULL;
    bool use_test_points = true;
    int rt_prio = 0, cpu = -1;

    s->bus[0].ifname = "can0";
    s->bus[1].ifname = "can1";
    s->bus[0].fd = s->bus[1].fd = -1;
    s->force_gain = 100;
    s->cal = (calibration_t){
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
        { "force-gain", required_argument, NULL, 'g' },
        { "gpio", required_argument, NULL, 'G' },
        { "rt", required_argument, NULL, 'r' },
        { "cpu", required_argument, NULL, 'c' },
        { "no-test-points", no_argument, NULL, 'n' },
        { "help", no_argument, NULL, 'h' },
        { 0, 0, 0, 0 }
    };
    int opt;
    bool args_ok = true;
    while ((opt = getopt_long(argc, argv, "h", options, NULL)) != -1) {
        switch (opt) {
        case 'A': s->bus[0].ifname = optarg; break;
        case 'B': s->bus[1].ifname = optarg; break;
        case 'b': bind_ip = optarg; break;
        case 'p': pc_ip = optarg; break;
        case 's': args_ok &= sscanf(optarg, "%d,%d,%d", &s->cal.steer_left, &s->cal.steer_center, &s->cal.steer_right) == 3; break;
        case 't': args_ok &= sscanf(optarg, "%d,%d", &s->cal.throttle_released, &s->cal.throttle_pressed) == 2; break;
        case 'k': args_ok &= sscanf(optarg, "%d,%d", &s->cal.brake_released, &s->cal.brake_pressed) == 2; break;
        case 'L': s->cal.btn_lsb = atoi(optarg); break;
        case 'R': s->cal.btn_rsb = atoi(optarg); break;
        case 'T': s->cal.btn_test = atoi(optarg); break;
        case 'g': s->force_gain = atoi(optarg); break;
        case 'G': args_ok &= parse_gpio(optarg); break;
        case 'r': rt_prio = atoi(optarg); break;
        case 'c': cpu = atoi(optarg); break;
        case 'n': use_test_points = false; break;
        default: usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }
    struct in_addr pc_addr = {0}, bind_addr;
    if (!args_ok || !calibration_ok(&s->cal) || inet_pton(AF_INET, bind_ip, &bind_addr) != 1 ||
        (pc_ip && inet_pton(AF_INET, pc_ip, &pc_addr) != 1)) {
        usage(argv[0]);
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    for (int b = 0; b < 2; b++)
        if (can_open(&s->bus[b]) != 0) return 1;

    s->tp.fd = -1;
    s->tp.enabled = use_test_points;
    if (use_test_points && test_points_open(&s->tp) != 0) return 1;

    s->udp_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    s->force_fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    struct sockaddr_in local = { .sin_family = AF_INET, .sin_addr = bind_addr, .sin_port = htons(UDP_PORT) };
    if (s->udp_fd < 0 || s->force_fd < 0 || bind(s->udp_fd, (struct sockaddr *)&local, sizeof(local)) != 0) {
        perror("udp bind (is wheel_monitor or another forwarder still running?)");
        return 1;
    }
    if (pc_ip) { s->proxy.sin_family = AF_INET; s->proxy.sin_addr = pc_addr; s->have_proxy = true; }

    if (cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        if (sched_setaffinity(0, sizeof(set), &set) != 0) { perror("sched_setaffinity"); return 1; }
    }
    if (rt_prio > 0) {
        struct sched_param sp = { .sched_priority = rt_prio };
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) { perror("sched_setscheduler (run as root)"); return 1; }
        mlockall(MCL_CURRENT | MCL_FUTURE);
    }

    /* Keyboard commands are read only when this program owns the terminal. */
    bool stdin_open = !isatty(STDIN_FILENO) || tcgetpgrp(STDIN_FILENO) == getpgrp();

    /* Power-up: error state (R3). */
    double now = now_s();
    g_t0 = now;
    s->state = ZONE_STATE_INIT;
    s->reason = R_STARTUP;
    s->tx_mode = 0xFF;                             /* forces the first BLINK out */
    s->next_hb = s->next_mon = s->next_force = now;
    test_points_set(&s->tp, TP_ERR, true);


    int exit_code = 0;
    while (g_running) {
        /* Sleep until a packet, a frame, a key, or the next deadline. */
        double next = s->next_mon;
        if (s->next_hb < next) next = s->next_hb;
        if (s->next_force < next) next = s->next_force;
        if (s->tx_mode != BLINK_MODE_OFF && s->tx_mode != 0xFF && s->next_toggle < next) next = s->next_toggle;
        if (s->blink_due && s->last_blink_tx + SPORADIC_MIN_S < next) next = s->last_blink_tx + SPORADIC_MIN_S;
        if (s->fault_q_len && s->last_fault_tx + SPORADIC_MIN_S < next) next = s->last_fault_tx + SPORADIC_MIN_S;
        if (s->ctrl_sent && !s->self_fail && s->last_ctrl_tx + CTRL_BACKSTOP_S < next) next = s->last_ctrl_tx + CTRL_BACKSTOP_S;
        double wait = next - now_s();
        if (wait < 0) wait = 0;
        struct timespec ts = { (time_t)wait, (long)((wait - (double)(time_t)wait) * 1e9) };
        struct pollfd fds[4] = {
            { s->udp_fd, POLLIN, 0 }, { s->bus[0].fd, POLLIN, 0 }, { s->bus[1].fd, POLLIN, 0 },
            { stdin_open ? STDIN_FILENO : -1, POLLIN, 0 },
        };
        if (ppoll(fds, 4, &ts, NULL) < 0) {
            if (errno == EINTR) continue;
            perror("ppoll");
            exit_code = 1;
            break;
        }

        /* Brake path first. */
        if (fds[0].revents & POLLIN) udp_rx(s, pc_ip ? &pc_addr : NULL);
        for (int b = 0; b < 2; b++) {
            if (fds[1 + b].revents & POLLERR) {        /* e.g. interface taken down: log, keep going */
                int err = 0;
                socklen_t len = sizeof(err);
                getsockopt(s->bus[b].fd, SOL_SOCKET, SO_ERROR, &err, &len);
                if (err) EVT(now_s(), "%s: %s", s->bus[b].ifname, strerror(err));
            }
            if (fds[1 + b].revents & POLLIN) can_rx(s, b);
        }

        if (fds[3].revents & (POLLIN | POLLHUP)) {
            char keys[64];
            ssize_t n = read(STDIN_FILENO, keys, sizeof(keys));
            if (n <= 0) stdin_open = false;
            for (ssize_t i = 0; i < n; i++) {
                switch (keys[i]) {
                case 'b': s->inject_next = INJ_STEER; break;
                case 't': s->inject_next = INJ_THROTTLE; break;
                case 'c': s->inject_next = INJ_CRC; break;
                case 'B': s->inject_persistent = !s->inject_persistent;
                          printf("continuous bad steering %s\n", s->inject_persistent ? "ON" : "OFF"); break;
                case 'x': self_test_press(s, now_s()); break;
                case 's': print_stats(s); break;
                case 'q': g_running = 0; break;
                default: break;
                }
            }
        }

        now = now_s();
        if (now >= s->next_mon) {
            hb_mon(s, now);
            s->next_mon = advance_deadline(s->next_mon, now, MON_PERIOD_S);
        }
        fault_tx(s, now);
        blink_tx(s, now);
        /* R4.4: CTRL at least every 50 ms even if UDP packets go missing. */
        if (s->ctrl_sent && !s->self_fail && now - s->last_ctrl_tx >= CTRL_BACKSTOP_S) ctrl_tx(s, now, true);
        if (now >= s->next_hb) {
            hb_tx(s, now);
            s->next_hb = advance_deadline(s->next_hb, now, HB_PERIOD_S);
        }
        if (now >= s->next_force) {
            force_udp(s, now);
            s->next_force = advance_deadline(s->next_force, now, FORCE_PERIOD_S);
        }
    }

    /* On the way out, tell the other zones at once instead of letting them time out. */
    if (!(s->self_fail & SF_TEST)) {
        struct fault_payload p;
        memset(&p, 0, sizeof(p));
        p.flags_seq = byte0(&s->seq_fault, FLAG_HI(CAN_FLAG_SYS_ERROR));
        p.failing_zone_id = ZONE_COCKPIT;
        p.reason_code = FAULT_R_SHUTDOWN;
        send_both(s, ID_FAULT, &p);
    }
    print_stats(s);
    close(s->bus[0].fd);
    close(s->bus[1].fd);
    close(s->udp_fd);
    close(s->force_fd);
    test_points_close(&s->tp);
    return exit_code;
}
