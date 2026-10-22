#ifndef LAB2_BLINKERS_H_
#define LAB2_BLINKERS_H_

#include <stdbool.h>
#include <stdint.h>

enum blinker_mode {
	BLINKER_MODE_OFF = 0,
	BLINKER_MODE_LEFT,
	BLINKER_MODE_RIGHT,
	BLINKER_MODE_HAZARD,
};

/* TODO: May need to adjust later*/
#define BLINKER_TURN_THRESHOLD_PERCENT 35
#define BLINKER_RETURN_THRESHOLD_PERCENT 10

int blinkers_init(void);

/* Selecting left or right replaces any previously selected direction. */
int blinkers_set_mode(enum blinker_mode mode);

/* 2 Hz hazards; clearing it turns the blinkers off. */
int blinkers_set_error(bool error_active);
/* Lab 3: apply the cockpit's phase immediately; no local blink timer. */
int blinkers_apply_phase(enum blinker_mode mode, bool on);

/*
 * Feed the latest normalized steering value to the self-cancel state machine.
 * A turn is armed after crossing the turn threshold and cancels after returning
 * into the center band.
 */
int blinkers_update_steering(int16_t steering_percent);

enum blinker_mode blinkers_get_mode(void);
bool blinkers_turn_crossed(void);
int16_t blinkers_get_last_steering(void);

#endif /* LAB2_BLINKERS_H_ */
