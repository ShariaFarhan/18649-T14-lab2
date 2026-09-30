#include <errno.h>
#include <string.h>
#include <zephyr/shell/shell.h>
#include <zephyr/kernel.h>
#include "encoder.h"

static int cmd_encoder_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    struct encoder_snapshot s;
    int rc = encoders_get(&s);
    if (rc) { shell_error(sh, "Encoder unavailable: %d", rc); return rc; }
    for (unsigned i = 0; i < ENCODER_WHEEL_COUNT; i++) {
        struct encoder_sample *w = &s.wheel[i];
        shell_print(sh, "%s count=%lld raw=%u delta=%d rpm_milli=%d dt_us=%llu sign=%d valid=%d dropped=%u",
                    i == ENCODER_LEFT ? "left" : "right",
                    (long long)w->count, w->raw_counter, w->delta, w->rpm_milli,
                    (unsigned long long)w->elapsed_us, w->direction_sign, w->valid,
                    w->dropped_samples);
    }
    shell_print(sh, "mean_rpm_milli=%d valid=%d; CPR=%u (10-turn calibration)",
                s.average_rpm_milli, s.valid, COUNTS_PER_WHEEL_REV);
    return 0;
}

static int cmd_encoder_zero(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    int rc = encoders_zero();
    if (rc) shell_error(sh, "Zero failed: %d", rc);
    else shell_print(sh, "Both software counts zeroed; next sample refreshes speed.");
    return rc;
}

static int cmd_encoder_sign(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    enum encoder_wheel w;
    if (!strcmp(argv[1], "left")) w = ENCODER_LEFT;
    else if (!strcmp(argv[1], "right")) w = ENCODER_RIGHT;
    else { shell_error(sh, "Use left or right"); return -EINVAL; }
    int sign;
    if (!strcmp(argv[2], "1")) sign = 1;
    else if (!strcmp(argv[2], "-1")) sign = -1;
    else { shell_error(sh, "Sign must be 1 or -1"); return -EINVAL; }
    int rc = encoders_set_sign(w, sign);
    if (rc) shell_error(sh, "Sign change failed: %d", rc);
    else shell_print(sh, "%s sign=%d; its count baseline was reset", argv[1], sign);
    return rc;
}

/* Low-priority display task; encoder sampling remains independent. */
static K_MUTEX_DEFINE(watch_lock);
static const struct shell *watch_shell;

static void encoder_watch_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    for (;;) {
        k_sleep(K_MSEC(200));
        k_mutex_lock(&watch_lock, K_FOREVER);
        const struct shell *sh = watch_shell;
        k_mutex_unlock(&watch_lock);
        /* Never hold watch_lock while printing: the shell command path can hold its own lock. */
        if (sh) {
            struct encoder_snapshot s;
            int rc = encoders_get(&s);
            if (rc) shell_error(sh, "Encoder unavailable: %d", rc);
            else {
                int64_t l = s.wheel[0].rpm_milli, r = s.wheel[1].rpm_milli;
                int64_t lm = l < 0 ? -l : l, rm = r < 0 ? -r : r;
                shell_print(sh,
                    "L count=%lld rpm=%s%lld.%03lld | R count=%lld rpm=%s%lld.%03lld | valid=%u/%u",
                    (long long)s.wheel[0].count, l < 0 ? "-" : "", (long long)(lm/1000), (long long)(lm%1000),
                    (long long)s.wheel[1].count, r < 0 ? "-" : "", (long long)(rm/1000), (long long)(rm%1000),
                    s.wheel[0].valid, s.wheel[1].valid);
            }
        }
    }
}
K_THREAD_DEFINE(encoder_watch_tid, 1536, encoder_watch_thread, NULL, NULL, NULL, 10, 0, 0);

static int cmd_encoder_watch(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    struct encoder_snapshot s;
    int rc = encoders_get(&s);
    if (rc) { shell_error(sh, "Encoder unavailable: %d", rc); return rc; }
    shell_print(sh, "Live encoder output every 200 ms. Use encoder stop to stop.");
    k_mutex_lock(&watch_lock, K_FOREVER);
    watch_shell = sh;
    k_mutex_unlock(&watch_lock);
    return 0;
}

static int cmd_encoder_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc); ARG_UNUSED(argv);
    k_mutex_lock(&watch_lock, K_FOREVER);
    watch_shell = NULL;
    k_mutex_unlock(&watch_lock);
    shell_print(sh, "Live encoder output stopped.");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(encoder_commands,
    SHELL_CMD_ARG(watch, NULL, "Stream counts and RPM every 200 ms", cmd_encoder_watch, 1, 0),
    SHELL_CMD_ARG(stop, NULL, "Stop live encoder output", cmd_encoder_stop, 1, 0),
    SHELL_CMD_ARG(status, NULL, "Read counts, signed rpm*1000 and mean", cmd_encoder_status, 1, 0),
    SHELL_CMD_ARG(zero, NULL, "Zero both software counts", cmd_encoder_zero, 1, 0),
    SHELL_CMD_ARG(sign, NULL, "Set direction: encoder sign <left|right> <1|-1>", cmd_encoder_sign, 3, 0),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(encoder, &encoder_commands, "Dual wheel encoder checks", NULL);
