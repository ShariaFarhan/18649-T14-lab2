#ifndef CURRENT_SENSE_H_
#define CURRENT_SENSE_H_

#include <stdbool.h>
#include <stdint.h>

enum current_sense_channel {
	CURRENT_SENSE_RIGHT_MOTOR = 0,
	CURRENT_SENSE_SERVO,
	CURRENT_SENSE_CHANNEL_COUNT,
};

struct current_sense_sample {
	uint16_t raw;
	int32_t adc_mv;
	int32_t delta_mv;
	int32_t current_ma;
	bool zero_valid;
};

/* Configure both ADC channels. Safe to call again after a failed init. */
int current_sense_init(void);

/* Read and average the selected ADC input. */
int current_sense_read(enum current_sense_channel channel,
		       struct current_sense_sample *sample);

/* Capture the present unloaded readings as the independent channel zeros. */
int current_sense_zero_all(void);

const char *current_sense_channel_name(enum current_sense_channel channel);

#endif /* CURRENT_SENSE_H_ */
