/**
 * @file blinkers_back.c
 * @brief Rear Blinker GPIO Control for Node 3 (Drivetrain Zone)
*/

#include "blinkers_back.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define BLINKER_RL_NODE DT_ALIAS(blinker_rl)
#define BLINKER_RR_NODE DT_ALIAS(blinker_rr)

#if !DT_NODE_HAS_STATUS(BLINKER_RL_NODE, okay) || \
		!DT_NODE_HAS_STATUS(BLINKER_RR_NODE, okay)
#error "Devicetree aliases blinker-rl/rr are required"
#endif

static const struct gpio_dt_spec rear_left =
	GPIO_DT_SPEC_GET(BLINKER_RL_NODE, gpios);
static const struct gpio_dt_spec rear_right =
	GPIO_DT_SPEC_GET(BLINKER_RR_NODE, gpios);

static struct k_spinlock state_lock;
static enum blinker_mode current_mode = BLINKER_MODE_OFF;
static bool initialized = false;

/** @brief Helper Functions for Blinkers **/

static int configure_output(const struct gpio_dt_spec *output)
{
	if (!gpio_is_ready_dt(output)) { return -ENODEV; }
	return gpio_pin_configure_dt(output, GPIO_OUTPUT_INACTIVE);
}

/** @brief Blinker Functions **/

int blinkers_init(void)
{
	const struct gpio_dt_spec *const outputs[] = {
		&rear_left,
		&rear_right,
	};
	k_spinlock_key_t key;
	int rc;

	for (size_t i = 0; i < ARRAY_SIZE(outputs); ++i) {
		rc = configure_output(outputs[i]);
		if (rc != 0) { return rc; }
	}

	key = k_spin_lock(&state_lock);
	current_mode = BLINKER_MODE_OFF;
	initialized = true;
	k_spin_unlock(&state_lock, key);

	return blinkers_set_state(BLINKER_MODE_OFF, false); 
}

int blinkers_set_state(enum blinker_mode mode, bool phase)
{
    k_spinlock_key_t key = k_spin_lock(&state_lock);
    if (!initialized) {
        k_spin_unlock(&state_lock, key);
        return -EACCES;
    }

    if (mode < BLINKER_MODE_OFF || mode > BLINKER_MODE_HAZARD) {
        k_spin_unlock(&state_lock, key);
        return -EINVAL;
    }

    current_mode = mode;

    const int left_on = phase && (mode == BLINKER_MODE_LEFT || mode == BLINKER_MODE_HAZARD);
    const int right_on = phase && (mode == BLINKER_MODE_RIGHT || mode == BLINKER_MODE_HAZARD);

    int rc = gpio_pin_set_dt(&rear_left, left_on);
    if (rc == 0) { rc = gpio_pin_set_dt(&rear_right, right_on); }

    k_spin_unlock(&state_lock, key);
    return rc;
}

// enum blinker_mode blinkers_get_mode(void)
// {
//     k_spinlock_key_t key = k_spin_lock(&state_lock);
//     enum blinker_mode mode = current_mode;
//     k_spin_unlock(&state_lock, key);
//     return mode;
// }
