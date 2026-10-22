#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include "servo.h"
#include "steer_task.h"
#include "steer_mapping.h"
#include "runtime.h"

/* Priority 5 preempts the priority-6 CAN/application dispatcher. Actual WCET
 * and the 1 ms local / 50 ms end-to-end deadlines still require measurement. */
K_MUTEX_DEFINE(steer_lock);
K_SEM_DEFINE(steer_event, 0, 1);
static int16_t target_percent;
static bool last_normal, submitted;
static uint32_t last_generation;
static uint32_t command_at;
static atomic_t output_failed, pulse_us;

static void steer_worker(void *a, void *b, void *c)
{
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    for (;;) {
        int wake=k_sem_take(&steer_event, K_MSEC(20));
        k_mutex_lock(&steer_lock, K_FOREVER);
        int16_t target = target_percent;
        uint32_t at=command_at;
        k_mutex_unlock(&steer_lock);
        if (lab3_local_failed() || k_uptime_get_32()-at>=100) target=0;
        if (atomic_get(&output_failed) || (wake && target!=0) ||
            (wake && atomic_get(&pulse_us)==SERVO_PULSE_CENTER_US)) {
            lab3_progress(WORK_CONTROL);continue;
        }
        lab3_profile_begin(PROFILE_STEER);
        int rc = servo_set_steering(target);
        if (rc) {
            atomic_set(&output_failed, 1);
            /* Best effort neutral; retain a latched fault even if it works. */
            (void)servo_center();
        }
        atomic_set(&pulse_us, servo_get_last_pulse_us());
        lab3_profile_end(PROFILE_STEER);
        lab3_progress(WORK_CONTROL);
    }
}
K_THREAD_DEFINE(steer_thread, 1024, steer_worker, NULL, NULL, NULL,
                5, 0, SYS_FOREVER_MS);

int steer_task_init(void)
{
    int rc = servo_prepare(); /* PWM off until the worker takes ownership. */
    if (rc) { atomic_set(&output_failed, 1); return rc; }
    k_thread_start(steer_thread);
    steer_task_submit(0, false, 0); /* Startup/error output is neutral. */
    return 0;
}

void steer_task_submit(int16_t command, bool normal, uint32_t generation)
{
    k_mutex_lock(&steer_lock, K_FOREVER);
    bool changed = !submitted || normal != last_normal ||
                   (normal && generation != last_generation);
    if (changed) {
        target_percent = lab3_steer_percent(command, normal);
        last_normal = normal;
        last_generation = generation;
        command_at=k_uptime_get_32();
        submitted = true;
    }
    k_mutex_unlock(&steer_lock);
    if (changed) k_sem_give(&steer_event);
}

bool steer_task_failed(void) { return atomic_get(&output_failed) != 0; }
uint32_t steer_task_pulse_us(void) { return (uint32_t)atomic_get(&pulse_us); }
