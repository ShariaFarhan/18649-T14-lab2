#ifndef LAB2_ENCODER_H_
#define LAB2_ENCODER_H_
#include <stdbool.h>
#include <stdint.h>

/* Table-based assumption: 90:1 gearbox * 11 pulses * x4 = 3960.
 * Validate with marked wheel turns; this is not a hardware measurement. */
#define COUNTS_PER_WHEEL_REV 3960U
#define ENCODER_LEFT_DEFAULT_SIGN 1
#define ENCODER_RIGHT_DEFAULT_SIGN 1
#define ENCODER_SAMPLE_PERIOD_MS 10U
#define ENCODER_MAX_SAMPLE_GAP_US 250000U
enum encoder_wheel { ENCODER_LEFT, ENCODER_RIGHT, ENCODER_WHEEL_COUNT };

struct encoder_sample {
    int64_t count;             /* Signed cumulative count since zero/sign change */
    uint32_t raw_counter;
    int32_t delta;             /* Signed count change over elapsed_us */
    int32_t rpm_milli;         /* Signed wheel rpm * 1000, no filtering */
    uint64_t elapsed_us;      /* Measured sample duration, not assumed 10ms */
    int direction_sign;       /* +1 or -1; set so forward is positive */
    bool valid;
    uint32_t dropped_samples;
};
struct encoder_snapshot {
    struct encoder_sample wheel[ENCODER_WHEEL_COUNT];
    int32_t average_rpm_milli; /* Arithmetic mean, only valid if both valid */
    bool valid;
};

int encoders_init(void);
int encoders_get(struct encoder_snapshot *out);
int encoders_zero(void); /* Changes software baselines; does not reset hardware */
int encoders_set_sign(enum encoder_wheel wheel, int sign);
#endif
