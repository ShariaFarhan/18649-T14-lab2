#ifndef BLINKERS_BACK_H_
#define BLINKERS_BACK_H_

#include <stdbool.h>
#include <stdint.h>

enum blinker_mode {
	BLINKER_MODE_OFF = 0,
	BLINKER_MODE_LEFT,
	BLINKER_MODE_RIGHT,
	BLINKER_MODE_HAZARD,
};

/**
 * @brief Initialize rear left and right blinker GPIO pins.
 */
int blinkers_init(void);

/**
 * @brief Directly apply state from received CAN 0x200 BLINK payload.
 * @param mode Blinker mode (OFF, LEFT, RIGHT, HAZARD)
 * @param phase True = ON state, False = OFF state
 */
int blinkers_set_state(enum blinker_mode mode, bool phase);

// enum blinker_mode blinkers_get_mode(void);

#endif /* BLINKERS_BACK_H_ */
