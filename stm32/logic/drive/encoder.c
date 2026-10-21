#include "encoder.h"
#include "encoder_math.h"
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stm32_ll_tim.h>

#define LEFT_QDEC DT_ALIAS(encoder_left)
#define RIGHT_QDEC DT_ALIAS(encoder_right)
BUILD_ASSERT(DT_NODE_HAS_STATUS(LEFT_QDEC, okay), "Left encoder missing");
BUILD_ASSERT(DT_NODE_HAS_STATUS(RIGHT_QDEC, okay), "Right encoder missing");
BUILD_ASSERT(DT_PROP(LEFT_QDEC, st_encoder_mode) == 3, "Left requires x4 mode");
BUILD_ASSERT(DT_PROP(RIGHT_QDEC, st_encoder_mode) == 3, "Right requires x4 mode");
BUILD_ASSERT(DT_PROP(LEFT_QDEC, st_counts_per_revolution) == COUNTS_PER_WHEEL_REV, "Left CPR mismatch");
BUILD_ASSERT(DT_PROP(RIGHT_QDEC, st_counts_per_revolution) == COUNTS_PER_WHEEL_REV, "Right CPR mismatch");

/* Zephyr QDEC owns initialization/pinmux/clocks. Its sensor API returns only
 * integer degrees modulo one turn, so read CNT/ARR (read-only) for full counts.
 * Neither timer may also be used by a PWM/counter driver. */
static const struct device *const qdec[] = {
    DEVICE_DT_GET(LEFT_QDEC), DEVICE_DT_GET(RIGHT_QDEC)
};
static TIM_TypeDef *const timers[] = {
    (TIM_TypeDef *)DT_REG_ADDR(DT_PARENT(LEFT_QDEC)),
    (TIM_TypeDef *)DT_REG_ADDR(DT_PARENT(RIGHT_QDEC))
};
BUILD_ASSERT(ENCODER_LEFT_DEFAULT_SIGN == 1 || ENCODER_LEFT_DEFAULT_SIGN == -1, "Left sign");
BUILD_ASSERT(ENCODER_RIGHT_DEFAULT_SIGN == 1 || ENCODER_RIGHT_DEFAULT_SIGN == -1, "Right sign");
static K_MUTEX_DEFINE(encoder_lock);
static bool initialized;
static struct encoder_sample samples[ENCODER_WHEEL_COUNT];
static uint32_t previous[ENCODER_WHEEL_COUNT];
static uint64_t previous_us[ENCODER_WHEEL_COUNT];

static uint64_t sample_time_us(void)
{
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

static void baseline(unsigned i, bool clear_count)
{
    previous[i] = LL_TIM_GetCounter(timers[i]);
    previous_us[i] = sample_time_us();
    samples[i].raw_counter = previous[i];
    samples[i].elapsed_us = 0;
    samples[i].delta = 0;
    samples[i].rpm_milli = 0;
    samples[i].valid = false;
    if (clear_count) {
        samples[i].count = 0;
        samples[i].dropped_samples = 0;
    }
}

static void sample_one(unsigned i)
{
    uint32_t raw = LL_TIM_GetCounter(timers[i]);
    uint64_t now = sample_time_us();
    uint64_t elapsed = now - previous_us[i];
    uint64_t modulus = (uint64_t)LL_TIM_GetAutoReload(timers[i]) + 1U;
    int32_t delta = 0, rpm = 0;
    bool valid = elapsed > 0 && elapsed <= ENCODER_MAX_SAMPLE_GAP_US &&
        encoder_count_delta(raw, previous[i], modulus, &delta);
    if (valid) {
        delta *= samples[i].direction_sign;
        valid = encoder_rpm_milli(delta, COUNTS_PER_WHEEL_REV, elapsed, &rpm);
    }
    samples[i].raw_counter = raw;
    samples[i].elapsed_us = elapsed;
    samples[i].valid = valid;
    samples[i].delta = valid ? delta : 0;
    samples[i].rpm_milli = valid ? rpm : 0;
    if (valid) samples[i].count += delta;
    else samples[i].dropped_samples++;
    previous[i] = raw;
    previous_us[i] = now;
}

static void sample_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    for (;;) {
        k_sleep(K_MSEC(ENCODER_SAMPLE_PERIOD_MS));
        k_mutex_lock(&encoder_lock, K_FOREVER);
        for (unsigned i = 0; i < ENCODER_WHEEL_COUNT; i++) sample_one(i);
        k_mutex_unlock(&encoder_lock);
    }
}
K_THREAD_DEFINE(encoder_tid, 1536, sample_thread, NULL, NULL, NULL,
                5, 0, K_TICKS_FOREVER);

int encoders_init(void)
{
    k_mutex_lock(&encoder_lock, K_FOREVER);
    if (initialized) { k_mutex_unlock(&encoder_lock); return -EALREADY; }
    for (unsigned i = 0; i < ENCODER_WHEEL_COUNT; i++) {
        if (!device_is_ready(qdec[i])) { k_mutex_unlock(&encoder_lock); return -ENODEV; }
    }
    for (unsigned i = 0; i < ENCODER_WHEEL_COUNT; i++) {
        samples[i].direction_sign = i == ENCODER_LEFT ? ENCODER_LEFT_DEFAULT_SIGN : ENCODER_RIGHT_DEFAULT_SIGN;
        baseline(i, true);
    }
    initialized = true;
    k_mutex_unlock(&encoder_lock);
    k_thread_start(encoder_tid);
    return 0;
}

int encoders_get(struct encoder_snapshot *out)
{
    if (!out) return -EINVAL;
    k_mutex_lock(&encoder_lock, K_FOREVER);
    if (!initialized) { k_mutex_unlock(&encoder_lock); return -EACCES; }
    uint64_t now = sample_time_us();
    out->valid = true;
    for (unsigned i = 0; i < ENCODER_WHEEL_COUNT; i++) {
        out->wheel[i] = samples[i];
        if (now - previous_us[i] > ENCODER_MAX_SAMPLE_GAP_US) out->wheel[i].valid = false;
        out->valid = out->valid && out->wheel[i].valid;
    }
    out->average_rpm_milli = out->valid ?
        (int32_t)(((int64_t)out->wheel[0].rpm_milli + out->wheel[1].rpm_milli) / 2) : 0;
    k_mutex_unlock(&encoder_lock);
    return 0;
}

int encoders_zero(void)
{
    k_mutex_lock(&encoder_lock, K_FOREVER);
    if (!initialized) { k_mutex_unlock(&encoder_lock); return -EACCES; }
    for (unsigned i = 0; i < ENCODER_WHEEL_COUNT; i++) baseline(i, true);
    k_mutex_unlock(&encoder_lock);
    return 0;
}

int encoders_set_sign(enum encoder_wheel wheel, int sign)
{
    if ((unsigned)wheel >= ENCODER_WHEEL_COUNT || (sign != 1 && sign != -1))
        return -EINVAL;
    k_mutex_lock(&encoder_lock, K_FOREVER);
    if (!initialized) { k_mutex_unlock(&encoder_lock); return -EACCES; }
    samples[wheel].direction_sign = sign;
    baseline((unsigned)wheel, true);
    k_mutex_unlock(&encoder_lock);
    return 0;
}
