#include "current_sense.h"

#include <errno.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define CURRENT_SENSE_NODE DT_PATH(zephyr_user)
#define CURRENT_SENSE_SAMPLES 32U

/*
 * ACS712ELC-05B: 185 mV/A at 5.0 V. Its output is ratiometric, so scale
 * sensitivity for the measured 5.27 V supply. The installed divider is
 * 1 kohm from OUT to ADC and 2 kohm from ADC to ground (ratio 2/3).
 */
#define ACS712_SUPPLY_MV 5270LL
#define ACS712_NOMINAL_SUPPLY_MV 5000LL
#define ACS712_SENSITIVITY_MV_PER_A 185LL
#define ACS712_DIVIDER_NUMERATOR 2LL
#define ACS712_DIVIDER_DENOMINATOR 3LL

#if !DT_NODE_HAS_PROP(CURRENT_SENSE_NODE, io_channels)
#error "zephyr,user io-channels are required for current sensing"
#endif

static const struct adc_dt_spec channels[CURRENT_SENSE_CHANNEL_COUNT] = {
	[CURRENT_SENSE_LEFT_MOTOR] =
		ADC_DT_SPEC_GET_BY_NAME(CURRENT_SENSE_NODE, left_motor_current),
	[CURRENT_SENSE_RIGHT_MOTOR] =
		ADC_DT_SPEC_GET_BY_NAME(CURRENT_SENSE_NODE, right_motor_current),
	[CURRENT_SENSE_SERVO] =
		ADC_DT_SPEC_GET_BY_NAME(CURRENT_SENSE_NODE, servo_current),
};

static const char *const channel_names[CURRENT_SENSE_CHANNEL_COUNT] = {
	[CURRENT_SENSE_LEFT_MOTOR] = "left-motor",
	[CURRENT_SENSE_RIGHT_MOTOR] = "right-motor",
	[CURRENT_SENSE_SERVO] = "servo",
};

static int32_t zero_mv[CURRENT_SENSE_CHANNEL_COUNT];
static bool zero_valid[CURRENT_SENSE_CHANNEL_COUNT];
static bool initialized;
K_MUTEX_DEFINE(adc_lock);

const char *current_sense_channel_name(enum current_sense_channel channel)
{
	if ((unsigned int)channel >= CURRENT_SENSE_CHANNEL_COUNT) {
		return "invalid";
	}

	return channel_names[channel];
}

int current_sense_init(void)
{
	int rc;

	for (size_t i = 0U; i < ARRAY_SIZE(channels); ++i) {
		if (!adc_is_ready_dt(&channels[i])) {
			initialized = false;
			return -ENODEV;
		}

		rc = adc_channel_setup_dt(&channels[i]);
		if (rc != 0) {
			initialized = false;
			return rc;
		}
	}

	initialized = true;
	return 0;
}

static int read_average(enum current_sense_channel channel, uint16_t *raw,
			int32_t *millivolts)
{
	const struct adc_dt_spec *spec;
	struct adc_sequence sequence = {0};
	int64_t total = 0;
	int16_t buffer;
	int32_t converted;
	int rc = 0;

	if (!initialized) {
		return -EACCES;
	}
	if ((unsigned int)channel >= CURRENT_SENSE_CHANNEL_COUNT || raw == NULL ||
	    millivolts == NULL) {
		return -EINVAL;
	}

	spec = &channels[channel];
	sequence.buffer = &buffer;
	sequence.buffer_size = sizeof(buffer);
	rc = adc_sequence_init_dt(spec, &sequence);
	if (rc != 0) {
		return rc;
	}

	k_mutex_lock(&adc_lock, K_FOREVER);
	for (uint32_t i = 0U; i < CURRENT_SENSE_SAMPLES; ++i) {
		buffer = 0;
		rc = adc_read_dt(spec, &sequence);
		if (rc != 0) {
			break;
		}
		total += buffer;
	}
	k_mutex_unlock(&adc_lock);

	if (rc != 0) {
		return rc;
	}

	*raw = (uint16_t)((total + (CURRENT_SENSE_SAMPLES / 2U)) /
			  CURRENT_SENSE_SAMPLES);
	converted = *raw;
	rc = adc_raw_to_millivolts_dt(spec, &converted);
	if (rc != 0) {
		return rc;
	}
	*millivolts = converted;

	return 0;
}

int current_sense_read(enum current_sense_channel channel,
		       struct current_sense_sample *sample)
{
	int rc;

	if (sample == NULL ||
	    (unsigned int)channel >= CURRENT_SENSE_CHANNEL_COUNT) {
		return -EINVAL;
	}

	rc = read_average(channel, &sample->raw, &sample->adc_mv);
	if (rc != 0) {
		return rc;
	}

	sample->zero_valid = zero_valid[channel];
	sample->delta_mv = sample->zero_valid
				   ? sample->adc_mv - zero_mv[channel]
				   : 0;
	if (sample->zero_valid) {
		/* Preserve the sign; IP+ to IP- produces a positive result. */
		int64_t numerator = (int64_t)sample->delta_mv * 1000LL *
			ACS712_NOMINAL_SUPPLY_MV * ACS712_DIVIDER_DENOMINATOR;
		int64_t denominator = ACS712_SENSITIVITY_MV_PER_A *
			ACS712_SUPPLY_MV * ACS712_DIVIDER_NUMERATOR;

		sample->current_ma = (int32_t)(numerator / denominator);
	} else {
		sample->current_ma = 0;
	}
	return 0;
}

int current_sense_zero_all(void)
{
	uint16_t raw;
	int32_t measured_mv[CURRENT_SENSE_CHANNEL_COUNT];
	int rc;

	for (size_t i = 0U; i < CURRENT_SENSE_CHANNEL_COUNT; ++i) {
		rc = read_average((enum current_sense_channel)i, &raw,
				  &measured_mv[i]);
		if (rc != 0) {
			return rc;
		}
		/* An unpowered ACS712 sits near ground, not its VCC/2 offset. */
		if (measured_mv[i] < 1000 || measured_mv[i] > 2500) {
			return -ERANGE;
		}
	}

	for (size_t i = 0U; i < CURRENT_SENSE_CHANNEL_COUNT; ++i) {
		zero_mv[i] = measured_mv[i];
		zero_valid[i] = true;
	}

	return 0;
}
