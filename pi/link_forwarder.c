
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <linux/gpio.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "The wire format is little-endian and this file copies packed structs directly."
#endif

/* ------------------------------ CONFIG ---------------------------------- */

#define UDP_PORT            8000
#define UART_BAUD           B115200

/* Proxy -> Pi packet: 4-byte counter + 272-byte DIJOYSTATE2 (handout format). */
#define WHEEL_PACKET_LEN    276
#define OFF_COUNTER         0
#define OFF_LX              4     /* steering */
#define OFF_LY              8     /* throttle */
#define OFF_LRZ             24    /* brake    */
#define OFF_BUTTONS         52    /* rgbButtons[128], 0x80 = pressed */
#define NUM_BUTTONS         128

/* Raw values the proxy sends (your Part 1 numbers). These defaults are the
 * handout's documented Mac-proxy convention. Override on the command line with
 * --steer L,C,R  --throttle RELEASED,PRESSED  --brake RELEASED,PRESSED.
 * Read your real values from the "in" part of the status line. */
#define IN_STEER_LEFT       (-32767)
#define IN_STEER_CENTER     0
#define IN_STEER_RIGHT      32767
#define IN_PEDAL_RELEASED   0
#define IN_PEDAL_PRESSED    32767

/* Raw button indices in rgbButtons[]. Override with --btn-lsb/--btn-rsb/--btn-test. */
#define RAW_BTN_LSB         9     /* left blinker  */
#define RAW_BTN_RSB         8     /* right blinker */
#define RAW_BTN_TEST        2     /* self-test (X) */

/* Scale the STM32 firmware expects (logic/steering/wheel_mapping.h and
 * logic/drive/right_speed_model.c). Do not change without changing the firmware. */
#define FW_STEER_LEFT       0
#define FW_STEER_CENTER     33151
#define FW_STEER_RIGHT      65535
#define FW_PEDAL_RELEASED   65535
#define FW_PEDAL_PRESSED    0
#define FW_PEDAL_ARM_MIN    64225 /* both pedals within 2 % of released to arm */
#define FW_BTN_LSB_BIT      (1u << 9)
#define FW_BTN_RSB_BIT      (1u << 8)
#define BAD_INPUT_STEERING  70000 /* deliberately out of range, for checkoff */

/* A raw value up to this far past a calibrated end is treated as that end
 * (calibration noise). Further out, it is passed on out of range so the STM32
 * rejects it as bad input. */
#define CAL_TOLERANCE_PERCENT 2

/* Deadlines (seconds). */
#define UDP_TIMEOUT_S       0.150 /* no wheel packet for this long  -> STALE */
#define FEEDBACK_TIMEOUT_S  0.150 /* no heartbeat for this long     -> STALE */
#define COMMAND_PERIOD_S    0.020 /* command frame at least this often       */
#define TEST_DEBOUNCE_S     0.050 /* self-test button debounce               */
#define ENABLE_ACK_S        0.500 /* STM32 must arm within this, else retry  */
#define ENABLE_RESET_S      0.100 /* disabled frames sent before a re-arm    */
#define STATUS_PERIOD_S     0.500 /* status line on the terminal             */

/* Pi <-> STM32 frame: START0 START1 LEN SEQ PAYLOAD CRC_LO CRC_HI. */
#define FRAME_START0        0x7E
#define FRAME_START1        0xA5
#define MAX_PAYLOAD         32
#define FRAME_OVERHEAD      6
#define HB_MARKER           0xBA  /* identifies the dual-drive firmware */

#define FLAG_UDP_STALE      0x01
#define FLAG_DUAL_ENABLE    0x08

#define ZONE_NORMAL         1
#define SYS_STARTUP         1
#define DRIVE_ARMED         0x01
#define DRIVE_BRAKING       0x02
#define DRIVE_FAULT         0x04
#define DRIVE_BOOST         0x08

/* Scope test points (BCM numbering). */
#define TP_UDP_RX           0
#define TP_CMD_TX           1
static const unsigned TP_PIN[2] = { 23, 24 };

/* ------------------------- WIRE FORMATS --------------------------------- */

#pragma pack(push, 1)
typedef struct {                 /* Pi -> STM32, 17 bytes */
    int32_t  steering_raw;       /* 0 left, 33151 center, 65535 right */
    int32_t  throttle_raw;       /* 65535 released, 0 fully pressed   */
    int32_t  brake_raw;          /* 65535 released, 0 fully pressed   */
    uint16_t buttons;            /* bit 8 = RSB, bit 9 = LSB          */
    uint8_t  flags;              /* FLAG_UDP_STALE, FLAG_DUAL_ENABLE  */
    uint8_t  test_counter;       /* self-test presses, modulo 32      */
    uint8_t  test_flags;         /* bit 0 = new sender, take baseline */
} cmd_payload_t;

typedef struct {                 /* STM32 -> Pi, 31 bytes */
    uint8_t  zone_state;
    uint8_t  marker;             /* must be HB_MARKER */
    uint16_t current_motor_l;    /* mA, 0xFFFF = unavailable */
    uint16_t current_motor_r;
    uint16_t current_servo;
    uint16_t steering_raw;
    uint16_t steering_packed;    /* %+100 | light mode << 8 | LED pins << 10 */
    uint16_t servo_pulse_us;
    int32_t  target_mrpm;
    int32_t  left_mrpm;
    int32_t  right_mrpm;
    uint16_t duty_percent;
    uint8_t  drive_flags;        /* armed, brake, fault, boost, reason << 4 */
    uint8_t  test_counter_ack;
    uint8_t  system_reason;
} hb_payload_t;
#pragma pack(pop)

_Static_assert(sizeof(cmd_payload_t) == 17, "command payload must be 17 bytes");
_Static_assert(sizeof(hb_payload_t) == 31, "heartbeat payload must be 31 bytes");

static const char *const SYSTEM_NAME[] = {
    "NORMAL", "STARTUP", "LINK_LOST", "BAD_INPUT", "SELF_TEST", "OUTPUT_FAULT"
};
#define SYSTEM_COUNT (sizeof(SYSTEM_NAME) / sizeof(SYSTEM_NAME[0]))

static const char *const DRIVE_REASON[] = {   /* enum dual_reason in dual_drive.h */
    "ok", "command timeout", "left encoder invalid", "right encoder invalid",
    "left count reversed", "right count reversed", "left wheel stalled",
    "right wheel stalled", "drive output failed", "invalid input", "control loop overrun"
};
#define DRIVE_REASON_COUNT (sizeof(DRIVE_REASON) / sizeof(DRIVE_REASON[0]))

static const char *const LIGHT_NAME[] = { "off", "left", "right", "hazard" };

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
    bool     level[2];
} test_points_t;

typedef struct {
    /* latest wheel input, raw and converted */
    int32_t  in_lx, in_ly, in_lrz;
    uint8_t  in_buttons[NUM_BUTTONS];
    int32_t  steering, throttle, brake;      /* firmware scale */
    uint16_t buttons;                        /* firmware bitmask */
    bool     have_rx, have_seq;
    double   last_rx;
    uint32_t last_seq;

    /* self-test button counter */
    bool     test_prev_known, test_prev_down;
    double   test_last_press;
    uint8_t  test_counter;
    bool     test_sync;                      /* true until the STM32 has our baseline */
    uint8_t  sync_counter;

    /* drive-enable request */
    bool     drive_enable, drive_acknowledged;
    double   drive_requested_at, drive_reset_until;

    bool     bad_input;                      /* invalid-input test active */

    /* STM32 heartbeat */
    hb_payload_t status;
    bool     have_status;
    double   last_hb;

    /* UART transmit */
    bool     have_tx;
    double   last_tx;
    uint8_t  serial_seq;
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

static uint32_t rd_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* CRC-16, reflected polynomial 0x8408, init 0xFFFF, no final XOR.
 * Must match link_crc16_ccitt() in logic/comms/link_parser.c. */
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
    }
    return crc;
}

/* Keep the send schedule on a fixed 20 ms grid: a late send does not shift later ones. */
static double advance_deadline(double deadline, double now)
{
    if (now < deadline) return deadline;
    return deadline + ((double)(long)((now - deadline) / COMMAND_PERIOD_S) + 1.0) * COMMAND_PERIOD_S;
}

static bool wheel_fresh(const state_t *s, double now)
{
    return s->have_rx && now - s->last_rx <= UDP_TIMEOUT_S;
}

static bool heartbeat_fresh(const state_t *s, double now)
{
    return s->have_status && now - s->last_hb <= FEEDBACK_TIMEOUT_S;
}

/* --------------------------- AXIS CONVERSION ---------------------------- */

/* Straight-line map: in_a -> out_a, in_b -> out_b. Works for either direction
 * of travel. Slightly past an end clamps to that end; far past an end is
 * extrapolated, which lands outside 0..65535 and is rejected by the STM32. */
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

/* ---------------------------- TEST POINTS ------------------------------- */

/* Uses the kernel GPIO character device directly, so no libgpiod is needed.
 * Finds the header GPIO chip by label and by the line names GPIO23 / GPIO24. */
static int test_points_open(test_points_t *tp)
{
    char seen[16][PATH_MAX];
    int seen_count = 0;

    tp->fd = -1;
    for (int i = 0; i < 16; i++) {
        char path[32], real[PATH_MAX];
        snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
        if (!realpath(path, real)) continue;

        bool duplicate = false;          /* Pi 5 has symlinked aliases of one chip */
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
        for (int p = 0; match && p < 2; p++) {
            struct gpio_v2_line_info line;
            char want[16];
            memset(&line, 0, sizeof(line));
            line.offset = TP_PIN[p];
            snprintf(want, sizeof(want), "GPIO%u", TP_PIN[p]);
            if (ioctl(chip, GPIO_V2_GET_LINEINFO_IOCTL, &line) != 0 || strcmp(line.name, want) != 0)
                match = false;
        }
        if (!match) { close(chip); continue; }

        /* Request both lines as outputs, starting low. Never fall back to
         * another chip if these lines are busy. */
        struct gpio_v2_line_request req;
        memset(&req, 0, sizeof(req));
        req.offsets[0] = TP_PIN[0];
        req.offsets[1] = TP_PIN[1];
        req.num_lines = 2;
        snprintf(req.consumer, sizeof(req.consumer), "lab2-test-points");
        req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
        req.config.num_attrs = 1;
        req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
        req.config.attrs[0].attr.values = 0;
        req.config.attrs[0].mask = 0x3;
        int rc = ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req);
        int saved = errno;
        close(chip);
        if (rc != 0) {
            fprintf(stderr, "GPIO23/GPIO24 on %s are busy: %s\n", real, strerror(saved));
            return -1;
        }
        tp->fd = req.fd;
        snprintf(tp->chip, sizeof(tp->chip), "%s", real);
        return 0;
    }
    fprintf(stderr, "Pi header GPIO23/24 chip not found; test markers are required "
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

static void test_points_close(test_points_t *tp)
{
    if (tp->enabled && tp->fd >= 0) close(tp->fd);
    tp->fd = -1;
}

/* ------------------------------- UART ----------------------------------- */

static int uart_open(const char *dev, struct termios *saved)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open uart"); return -1; }

    /* Only one program may own the UART. A second forwarder fails here. */
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr, "%s is in use by another program (check: fuser -v %s)\n", dev, dev);
        close(fd);
        return -1;
    }
    if (tcgetattr(fd, saved) != 0) { perror("tcgetattr"); close(fd); return -1; }

    struct termios tty = *saved;
    cfmakeraw(&tty);                              /* 8 data bits, no parity, binary */
    cfsetispeed(&tty, UART_BAUD);
    cfsetospeed(&tty, UART_BAUD);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(CRTSCTS | CSTOPB);           /* no flow control, one stop bit */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { perror("tcsetattr"); close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static size_t build_command_frame(uint8_t *out, const state_t *s, bool stale)
{
    cmd_payload_t p;
    p.steering_raw = s->bad_input ? BAD_INPUT_STEERING : s->steering;
    p.throttle_raw = s->throttle;
    p.brake_raw = s->brake;
    p.buttons = s->buttons;
    /* A stale frame can never carry the drive-enable bit. */
    p.flags = (uint8_t)((stale ? FLAG_UDP_STALE : 0) |
                        (s->drive_enable && s->have_status && !stale ? FLAG_DUAL_ENABLE : 0));
    /* While syncing, keep sending the baseline so presses are not miscounted. */
    p.test_counter = s->test_sync ? s->sync_counter : s->test_counter;
    p.test_flags = s->test_sync ? 1 : 0;

    size_t i = 0;
    out[i++] = FRAME_START0;
    out[i++] = FRAME_START1;
    out[i++] = (uint8_t)sizeof(p);                /* LEN */
    out[i++] = s->serial_seq;                     /* SEQ */
    memcpy(&out[i], &p, sizeof(p));
    i += sizeof(p);
    uint16_t crc = crc16(&out[2], 2 + sizeof(p)); /* CRC covers LEN, SEQ, payload */
    out[i++] = (uint8_t)(crc & 0xFF);
    out[i++] = (uint8_t)(crc >> 8);
    return i;
}

/* --------------------------- WHEEL PACKETS ------------------------------ */

static bool button_down(const state_t *s, int index)
{
    return index >= 0 && index < NUM_BUTTONS && (s->in_buttons[index] & 0x80);
}

/* Count debounced presses of the self-test button. The STM32 decides what a
 * single or double press means; the Pi only counts rising edges. */
static void self_test_update(state_t *s, bool down, double now)
{
    bool rising = s->test_prev_known && !s->test_prev_down && down;
    s->test_prev_known = true;
    s->test_prev_down = down;
    if (rising && now - s->test_last_press >= TEST_DEBOUNCE_S) {
        s->test_last_press = now;
        s->test_counter = (uint8_t)((s->test_counter + 1) & 31);
    }
}

/* Take one 276-byte proxy packet. Returns true if it was accepted as newer. */
static bool wheel_packet(state_t *s, const calibration_t *cal, const uint8_t *d, ssize_t n, double now)
{
    if (n != WHEEL_PACKET_LEN) return false;

    /* Accept the first packet, a packet after a long silence (a restarted
     * proxy), or any packet newer than the last one. Drop duplicates and
     * reordered packets. */
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

    /* After a gap, a button that is already held must not count as a press. */
    if (resumed) s->test_prev_known = false;
    self_test_update(s, button_down(s, cal->btn_test), now);

    s->last_seq = counter;
    s->have_seq = true;
    s->last_rx = now;
    s->have_rx = true;
    return true;
}

/* ----------------------------- HEARTBEAT -------------------------------- */

static bool heartbeat_valid(const hb_payload_t *hb)
{
    return hb->marker == HB_MARKER && hb->test_counter_ack <= 31 && hb->system_reason < SYSTEM_COUNT;
}

/* ---------------------------- DRIVE ENABLE ------------------------------ */

/* The STM32 only arms the motors when a command carries the enable bit. The Pi
 * asks for it only when the wheel stream and heartbeat are fresh, the STM32
 * reports NORMAL with no drive fault, and both pedals are released. If the
 * STM32 disarms for any reason, the request is dropped and is not raised again
 * until those conditions hold once more. */
static void drive_enable_update(state_t *s, double now)
{
    bool wheel_ok = wheel_fresh(s, now);
    bool hb_ok = heartbeat_fresh(s, now);
    bool fault = hb_ok && (s->status.drive_flags & DRIVE_FAULT);

    if (s->drive_enable && (!wheel_ok || !hb_ok || fault || s->bad_input)) {
        s->drive_enable = false;
        s->drive_reset_until = now + ENABLE_RESET_S;
    }

    if (!s->drive_enable) {
        s->drive_acknowledged = false;
    } else if (s->status.drive_flags & DRIVE_ARMED) {
        s->drive_acknowledged = true;
    } else if (s->drive_acknowledged || now - s->drive_requested_at >= ENABLE_ACK_S) {
        /* It was armed and is not any more, or it never armed: stop asking. */
        s->drive_enable = false;
        s->drive_acknowledged = false;
        s->drive_reset_until = now + ENABLE_RESET_S;
    }

    if (!s->drive_enable && wheel_ok && hb_ok && !fault && !s->bad_input && !s->test_sync &&
        now >= s->drive_reset_until && s->status.zone_state == ZONE_NORMAL &&
        s->throttle >= FW_PEDAL_ARM_MIN && s->brake >= FW_PEDAL_ARM_MIN) {
        s->drive_enable = true;
        s->drive_acknowledged = false;
        s->drive_requested_at = now;
    }
}

/* ------------------------------ STATUS ---------------------------------- */

static void print_current(uint16_t ma)
{
    if (ma == 0xFFFF) printf("n/a");
    else printf("%u", (unsigned)ma);
}

static void print_status(const state_t *s, double now)
{
    if (heartbeat_fresh(s, now)) printf("[%s]", SYSTEM_NAME[s->status.system_reason]);
    else printf("[NO STM32 HEARTBEAT]");

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
    printf(" test=%u%s enable=%d%s", (unsigned)s->test_counter, s->test_sync ? "(sync)" : "",
           s->drive_enable, s->bad_input ? " BAD-INPUT-TEST" : "");

    if (heartbeat_fresh(s, now)) {
        const hb_payload_t *hb = &s->status;
        unsigned reason = hb->drive_flags >> 4;
        printf(" | stm steer=%d%% %uus target=%.1f L=%.1f R=%.1f rpm pwm=%u%% armed=%d brake=%d",
               (int)(hb->steering_packed & 255) - 100, (unsigned)hb->servo_pulse_us,
               hb->target_mrpm / 1000.0, hb->left_mrpm / 1000.0, hb->right_mrpm / 1000.0,
               (unsigned)hb->duty_percent, !!(hb->drive_flags & DRIVE_ARMED), !!(hb->drive_flags & DRIVE_BRAKING));
        if (hb->drive_flags & DRIVE_FAULT)
            printf(" FAULT(%s)", reason < DRIVE_REASON_COUNT ? DRIVE_REASON[reason] : "unknown");
        printf(" | I L/R/servo=");
        print_current(hb->current_motor_l); printf("/");
        print_current(hb->current_motor_r); printf("/");
        print_current(hb->current_servo);
        printf(" mA | lights=%s", LIGHT_NAME[(hb->steering_packed >> 8) & 3]);
    }
    printf("\n");
}

/* ------------------------------- MAIN ----------------------------------- */

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s [options]\n"
        "  --uart DEV            UART to the STM32 (default /dev/serial0)\n"
        "  --bind IP             local address for UDP port %d (default 0.0.0.0)\n"
        "  --pc-ip IP            accept wheel packets only from this address (default: any)\n"
        "  --steer L,C,R         raw steering at full left, center, full right (default %d,%d,%d)\n"
        "  --throttle REL,PRESS  raw throttle released, fully pressed (default %d,%d)\n"
        "  --brake REL,PRESS     raw brake released, fully pressed (default %d,%d)\n"
        "  --btn-lsb N           raw button index for the left blinker (default %d)\n"
        "  --btn-rsb N           raw button index for the right blinker (default %d)\n"
        "  --btn-test N          raw button index for the self-test button (default %d)\n"
        "  --quiet               no periodic status line\n"
        "  --no-test-points      run without the GPIO23/GPIO24 scope markers\n",
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
    const char *uart_dev = "/dev/serial0", *bind_ip = "0.0.0.0", *pc_ip = NULL;
    bool use_test_points = true, quiet = false;
    calibration_t cal = {
        IN_STEER_LEFT, IN_STEER_CENTER, IN_STEER_RIGHT,
        IN_PEDAL_RELEASED, IN_PEDAL_PRESSED, IN_PEDAL_RELEASED, IN_PEDAL_PRESSED,
        RAW_BTN_LSB, RAW_BTN_RSB, RAW_BTN_TEST
    };

    static const struct option options[] = {
        { "uart", required_argument, NULL, 'u' },
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
    while ((opt = getopt_long(argc, argv, "u:b:p:s:t:k:L:R:T:qnh", options, NULL)) != -1) {
        switch (opt) {
        case 'u': uart_dev = optarg; break;
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

    /* Ctrl-C and SIGTERM end the loop so the final STALE frame is sent. */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    struct termios saved_tty;
    int uart_fd = uart_open(uart_dev, &saved_tty);
    if (uart_fd < 0) return 1;

    test_points_t tp;
    memset(&tp, 0, sizeof(tp));
    tp.fd = -1;
    tp.enabled = use_test_points;
    if (tp.enabled && test_points_open(&tp) != 0) {
        tcsetattr(uart_fd, TCSANOW, &saved_tty);
        close(uart_fd);
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
        tcsetattr(uart_fd, TCSANOW, &saved_tty);
        close(uart_fd);
        if (udp_fd >= 0) close(udp_fd);
        return 1;
    }

    /* Keyboard commands are read only when this program owns the terminal. */
    bool stdin_open = !isatty(STDIN_FILENO) || tcgetpgrp(STDIN_FILENO) == getpgrp();

    /* Until the proxy sends something: centered, pedals released, and "new
     * sender" set so the STM32 takes a counter baseline. */
    state_t s;
    memset(&s, 0, sizeof(s));
    s.steering = FW_STEER_CENTER;
    s.throttle = FW_PEDAL_RELEASED;
    s.brake = FW_PEDAL_RELEASED;
    s.test_sync = true;
    s.test_last_press = -1.0;

    uint8_t pending[4096];                        /* UART bytes not yet framed */
    size_t pending_len = 0;
    double next_send = now_s(), next_status = next_send;
    int exit_code = 0;

    printf("Link forwarder: UDP/%d from %s -> %s\n", UDP_PORT, pc_ip ? pc_ip : "any sender", uart_dev);
    printf("Calibration: steer %d,%d,%d  throttle %d,%d  brake %d,%d  buttons LSB=%d RSB=%d test=%d\n",
           cal.steer_left, cal.steer_center, cal.steer_right, cal.throttle_released, cal.throttle_pressed,
           cal.brake_released, cal.brake_pressed, cal.btn_lsb, cal.btn_rsb, cal.btn_test);
    if (stdin_open) printf("Keys: b + Enter = toggle invalid-input test, q + Enter = quit\n");

    while (g_running) {
        bool send_update = false;

        /* Sleep until a wheel packet, UART bytes, a key, or the next 20 ms deadline. */
        double wait = next_send - now_s();
        if (wait < 0) wait = 0;
        struct timespec ts = { (time_t)wait, (long)((wait - (double)(time_t)wait) * 1e9) };
        struct pollfd fds[3] = {
            { uart_fd, POLLIN, 0 }, { udp_fd, POLLIN, 0 }, { stdin_open ? STDIN_FILENO : -1, POLLIN, 0 }
        };
        if (ppoll(fds, 3, &ts, NULL) < 0) {
            if (errno == EINTR) continue;
            perror("ppoll");
            exit_code = 1;
            break;
        }

        /* ---- proxy -> Pi: one wheel packet per pass ---- */
        if (fds[1].revents & POLLIN) {
            uint8_t data[2048];
            struct sockaddr_in from;
            socklen_t from_len = sizeof(from);
            ssize_t n = recvfrom(udp_fd, data, sizeof(data), 0, (struct sockaddr *)&from, &from_len);
            if (n >= 0 && (!pc_ip || from.sin_addr.s_addr == pc_addr.s_addr)) {
                test_points_toggle(&tp, TP_UDP_RX);
                /* A newer packet is forwarded now, without waiting for the deadline. */
                if (wheel_packet(&s, &cal, data, n, now_s())) send_update = true;
            }
        }

        /* ---- STM32 -> Pi: heartbeat frames ---- */
        if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fprintf(stderr, "UART closed\n");
            exit_code = 1;
            break;
        }
        if (fds[0].revents & POLLIN) {
            ssize_t n = read(uart_fd, &pending[pending_len], sizeof(pending) - pending_len);
            if (n > 0) pending_len += (size_t)n;

            while (pending_len >= 4) {
                /* Hunt for the two start bytes and a plausible length. */
                if (pending[0] != FRAME_START0 || pending[1] != FRAME_START1 || pending[2] > MAX_PAYLOAD) {
                    memmove(pending, pending + 1, --pending_len);
                    continue;
                }
                size_t size = (size_t)pending[2] + FRAME_OVERHEAD;
                if (pending_len < size) break;    /* wait for the rest of the frame */

                uint8_t frame[MAX_PAYLOAD + FRAME_OVERHEAD];
                memcpy(frame, pending, size);
                pending_len -= size;
                memmove(pending, pending + size, pending_len);

                uint16_t crc_rx = (uint16_t)(frame[size - 2] | (frame[size - 1] << 8));
                if (crc16(&frame[2], size - 4) != crc_rx) continue;      /* drop, no NACK */
                if (frame[2] != sizeof(hb_payload_t)) continue;

                hb_payload_t hb;
                memcpy(&hb, &frame[4], sizeof(hb));
                if (!heartbeat_valid(&hb)) continue;

                double t = now_s();
                bool changed = !s.have_status || s.status.system_reason != hb.system_reason;
                if (!s.have_status) printf("STM32 heartbeat received\n");
                s.status = hb;
                s.have_status = true;
                s.last_hb = t;

                /* The STM32 has taken our counter baseline: start reporting presses. */
                if (s.test_sync && hb.test_counter_ack == s.sync_counter && hb.system_reason != SYS_STARTUP)
                    s.test_sync = false;
                if (changed) print_status(&s, t);
            }
            if (pending_len == sizeof(pending)) pending_len = 0;         /* cannot happen with valid frames */
        }

        /* ---- keyboard ---- */
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

        /* ---- Pi -> STM32: command frame on a wheel update or the 20 ms deadline ---- */
        double now = now_s();
        drive_enable_update(&s, now);
        if (!quiet && now >= next_status) {
            print_status(&s, now);
            next_status = now + STATUS_PERIOD_S;
        }
        if (!send_update && now < next_send) continue;

        bool stale = !wheel_fresh(&s, now) || !heartbeat_fresh(&s, now);
        uint8_t frame[MAX_PAYLOAD + FRAME_OVERHEAD];
        size_t frame_len = build_command_frame(frame, &s, stale);
        test_points_toggle(&tp, TP_CMD_TX);
        if (write(uart_fd, frame, frame_len) != (ssize_t)frame_len) {
            fprintf(stderr, "Short UART write\n");
            exit_code = 1;
            break;
        }
        s.last_tx = now_s();
        s.have_tx = true;
        s.serial_seq++;
        next_send = advance_deadline(next_send, s.last_tx);
    }

    /* On the way out, tell the STM32 the source is gone so it brakes at once
     * instead of waiting for its own timeout. */
    s.drive_enable = false;
    s.bad_input = false;
    uint8_t frame[MAX_PAYLOAD + FRAME_OVERHEAD];
    size_t frame_len = build_command_frame(frame, &s, true);
    test_points_toggle(&tp, TP_CMD_TX);
    if (write(uart_fd, frame, frame_len) == (ssize_t)frame_len) tcdrain(uart_fd);

    tcsetattr(uart_fd, TCSANOW, &saved_tty);
    close(uart_fd);
    close(udp_fd);
    test_points_close(&tp);
    printf("Stopped.\n");
    return exit_code;
}
