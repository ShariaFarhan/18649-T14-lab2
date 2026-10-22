#include "front_blinkers.h"

#include "can_comms.h"

void front_blinkers_resolve(uint8_t mode, bool phase_on, bool zone_fault,
                            uint32_t now_ms, struct front_blinker_outputs *out)
{
    if (out == NULL) return;
    if (zone_fault) {
        bool on = ((now_ms / 250u) & 1u) == 0;
        *out = (struct front_blinker_outputs){ .left = on, .right = on };
        return;
    }
    out->left = phase_on && (mode == BLINK_MODE_LEFT || mode == BLINK_MODE_HAZARD);
    out->right = phase_on && (mode == BLINK_MODE_RIGHT || mode == BLINK_MODE_HAZARD);
}

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define FRONT_LEFT_NODE DT_ALIAS(blinker_fl)
#define FRONT_RIGHT_NODE DT_ALIAS(blinker_fr)
#if !DT_NODE_EXISTS(FRONT_LEFT_NODE) || !DT_NODE_EXISTS(FRONT_RIGHT_NODE)
#error "N2 overlay must define blinker-fl and blinker-fr aliases"
#endif

static const struct gpio_dt_spec left_lamp = GPIO_DT_SPEC_GET(FRONT_LEFT_NODE, gpios);
static const struct gpio_dt_spec right_lamp = GPIO_DT_SPEC_GET(FRONT_RIGHT_NODE, gpios);

int front_blinkers_init(void)
{
    if (!gpio_is_ready_dt(&left_lamp) || !gpio_is_ready_dt(&right_lamp))
        return -ENODEV;
    int rc = gpio_pin_configure_dt(&left_lamp, GPIO_OUTPUT_INACTIVE);
    if (rc != 0) return rc;
    return gpio_pin_configure_dt(&right_lamp, GPIO_OUTPUT_INACTIVE);
}

int front_blinkers_apply(const struct front_blinker_outputs *out)
{
    if (out == NULL) return -EINVAL;
    int left_rc = gpio_pin_set_dt(&left_lamp, out->left);
    int right_rc = gpio_pin_set_dt(&right_lamp, out->right);
    return left_rc != 0 ? left_rc : right_rc;
}
