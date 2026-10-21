#include "blinkers.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define BLINKER_FL_NODE DT_ALIAS(blinker_fl)
#define BLINKER_FR_NODE DT_ALIAS(blinker_fr)
#define BLINKER_RL_NODE DT_ALIAS(blinker_rl)
#define BLINKER_RR_NODE DT_ALIAS(blinker_rr)

#if !DT_NODE_HAS_STATUS(BLINKER_FL_NODE, okay) || \
	!DT_NODE_HAS_STATUS(BLINKER_FR_NODE, okay) || \
	!DT_NODE_HAS_STATUS(BLINKER_RL_NODE, okay) || \
	!DT_NODE_HAS_STATUS(BLINKER_RR_NODE, okay)
#error "Devicetree aliases blinker-fl/fr/rl/rr are required"
#endif

#define TURN_HALF_PERIOD_MS 500
#define HAZARD_HALF_PERIOD_MS 250

static const struct gpio_dt_spec front_left =
	GPIO_DT_SPEC_GET(BLINKER_FL_NODE, gpios);
static const struct gpio_dt_spec front_right =
	GPIO_DT_SPEC_GET(BLINKER_FR_NODE, gpios);
static const struct gpio_dt_spec rear_left =
	GPIO_DT_SPEC_GET(BLINKER_RL_NODE, gpios);
static const struct gpio_dt_spec rear_right =
	GPIO_DT_SPEC_GET(BLINKER_RR_NODE, gpios);

static struct k_spinlock state_lock;
static enum blinker_mode current_mode = BLINKER_MODE_OFF;
static bool phase_on;
static bool turn_crossed;
static bool initialized;
static int16_t last_steering;
static int64_t next_edge_ms;

static int apply_outputs(enum blinker_mode mode, bool on)
{
	const int left_on = on &&
		(mode == BLINKER_MODE_LEFT || mode == BLINKER_MODE_HAZARD);
	const int right_on = on &&
		(mode == BLINKER_MODE_RIGHT || mode == BLINKER_MODE_HAZARD);
	int rc;

	rc = gpio_pin_set_dt(&front_left, left_on);
	if (rc == 0) {
		rc = gpio_pin_set_dt(&rear_left, left_on);
	}
	if (rc == 0) {
		rc = gpio_pin_set_dt(&front_right, right_on);
	}
	if (rc == 0) {
		rc = gpio_pin_set_dt(&rear_right, right_on);
	}

	return rc;
}

static void blink_work_handler(struct k_work *work)
{
	enum blinker_mode mode;
	bool on;
	k_spinlock_key_t key;

	ARG_UNUSED(work);

	key = k_spin_lock(&state_lock);
	if (!initialized || current_mode == BLINKER_MODE_OFF) {
		k_spin_unlock(&state_lock, key);
		return;
	}
	int64_t now = k_uptime_get();
	if (now < next_edge_ms) {
		k_spin_unlock(&state_lock, key);
		return; /* Discard queued work from a previous light mode. */
	}
	uint32_t half_period = current_mode == BLINKER_MODE_HAZARD ?
		HAZARD_HALF_PERIOD_MS : TURN_HALF_PERIOD_MS;
	uint32_t edges = 1U + (uint32_t)((now - next_edge_ms) / half_period);
	next_edge_ms += (int64_t)edges * half_period;
	if (edges & 1U) phase_on = !phase_on;
	mode = current_mode;
	on = phase_on;
	/* These four outputs are local STM32 GPIOs; serialize them with mode changes. */
	(void)apply_outputs(mode, on);
	k_spin_unlock(&state_lock, key);
}

K_WORK_DEFINE(blink_work, blink_work_handler);

static void blink_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	(void)k_work_submit(&blink_work);
}

K_TIMER_DEFINE(blink_timer, blink_timer_handler, NULL);

static int configure_output(const struct gpio_dt_spec *output)
{
	if (!gpio_is_ready_dt(output)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(output, GPIO_OUTPUT_INACTIVE);
}

int blinkers_init(void)
{
	const struct gpio_dt_spec *const outputs[] = {
		&front_left,
		&front_right,
		&rear_left,
		&rear_right,
	};
	k_spinlock_key_t key;
	int rc;

	k_timer_stop(&blink_timer);
	(void)k_work_cancel(&blink_work);

	for (size_t i = 0; i < ARRAY_SIZE(outputs); ++i) {
		rc = configure_output(outputs[i]);
		if (rc != 0) {
			return rc;
		}
	}

	key = k_spin_lock(&state_lock);
	current_mode = BLINKER_MODE_OFF;
	phase_on = false;
	turn_crossed = false;
	last_steering = 0;
	initialized = true;
	k_spin_unlock(&state_lock, key);

	return apply_outputs(BLINKER_MODE_OFF, false);
}

int blinkers_set_mode(enum blinker_mode mode)
{
	k_spinlock_key_t key;
	uint32_t half_period_ms;
	int rc;

	if (mode < BLINKER_MODE_OFF || mode > BLINKER_MODE_HAZARD) {
		return -EINVAL;
	}

	key = k_spin_lock(&state_lock);
	if (!initialized) {
		k_spin_unlock(&state_lock, key);
		return -EACCES;
	}
	k_spin_unlock(&state_lock, key);

	k_timer_stop(&blink_timer);
	(void)k_work_cancel(&blink_work);

	key = k_spin_lock(&state_lock);
	current_mode = mode;
	phase_on = mode != BLINKER_MODE_OFF;
	turn_crossed = false;
	half_period_ms = mode == BLINKER_MODE_HAZARD ?
		HAZARD_HALF_PERIOD_MS : TURN_HALF_PERIOD_MS;
	next_edge_ms = k_uptime_get() + half_period_ms;
	rc = apply_outputs(mode, phase_on);
	k_spin_unlock(&state_lock, key);
	if (rc != 0 || mode == BLINKER_MODE_OFF) {
		return rc;
	}

	half_period_ms = mode == BLINKER_MODE_HAZARD ?
		HAZARD_HALF_PERIOD_MS : TURN_HALF_PERIOD_MS;
	k_timer_start(&blink_timer, K_MSEC(half_period_ms),
		      K_MSEC(half_period_ms));
	return 0;
}

int blinkers_set_error(bool error_active)
{
	return blinkers_set_mode(error_active ? BLINKER_MODE_HAZARD :
					  BLINKER_MODE_OFF);
}

int blinkers_update_steering(int16_t steering_percent)
{
	k_spinlock_key_t key;
	bool should_cancel = false;

	if (steering_percent < -100) {
		steering_percent = -100;
	} else if (steering_percent > 100) {
		steering_percent = 100;
	}

	key = k_spin_lock(&state_lock);
	if (!initialized) {
		k_spin_unlock(&state_lock, key);
		return -EACCES;
	}

	last_steering = steering_percent;
	if (current_mode == BLINKER_MODE_LEFT) {
		if (!turn_crossed &&
		    steering_percent <= -BLINKER_TURN_THRESHOLD_PERCENT) {
			turn_crossed = true;
		} else if (turn_crossed &&
			   steering_percent >= -BLINKER_RETURN_THRESHOLD_PERCENT) {
			should_cancel = true;
		}
	} else if (current_mode == BLINKER_MODE_RIGHT) {
		if (!turn_crossed &&
		    steering_percent >= BLINKER_TURN_THRESHOLD_PERCENT) {
			turn_crossed = true;
		} else if (turn_crossed &&
			   steering_percent <= BLINKER_RETURN_THRESHOLD_PERCENT) {
			should_cancel = true;
		}
	}
	k_spin_unlock(&state_lock, key);

	return should_cancel ? blinkers_set_mode(BLINKER_MODE_OFF) : 0;
}

enum blinker_mode blinkers_get_mode(void)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	enum blinker_mode mode = current_mode;

	k_spin_unlock(&state_lock, key);
	return mode;
}

bool blinkers_turn_crossed(void)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	bool crossed = turn_crossed;

	k_spin_unlock(&state_lock, key);
	return crossed;
}

int16_t blinkers_get_last_steering(void)
{
	k_spinlock_key_t key = k_spin_lock(&state_lock);
	int16_t steering = last_steering;

	k_spin_unlock(&state_lock, key);
	return steering;
}
