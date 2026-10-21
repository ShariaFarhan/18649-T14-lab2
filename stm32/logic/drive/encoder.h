#ifndef LAB2_ENCODER_H_
#define LAB2_ENCODER_H_
#include <stdbool.h>
#include <stdint.h>

/* Bench calibration: 10 wheel turns gave L=13186 and R=13222 counts.
 * Use nominal 1320 counts/turn (x4), consistent with 30:1 * 11 * 4. */
#define COUNTS_PER_WHEEL_REV 1320U
#ifndef ENCODER_LEFT_DEFAULT_SIGN
#define ENCODER_LEFT_DEFAULT_SIGN 1
#endif
#ifndef ENCODER_RIGHT_DEFAULT_SIGN
#define ENCODER_RIGHT_DEFAULT_SIGN (-1) /* Right wheel polarity confirmed on the bench. */
#endif
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
