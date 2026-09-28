

#ifndef PI_LINK_H
#define PI_LINK_H

#include <stdbool.h>
#include "link_protocol.h"

//Starts the UART RX ISR + ring buffer, call once from main()
void pi_link_init(void);


void pi_link_on_link_lost(void);
void pi_link_on_link_restored(void);

void pi_link_get_currents(uint16_t *motor_l, uint16_t *motor_r, uint16_t *servo);

void pi_link_set_zone_state_override(enum pi_link_zone_state state);


//debugging prints
bool pi_link_is_alive(void);
uint32_t pi_link_rx_valid_count(void);
uint32_t pi_link_rx_crc_fail_count(void);
uint32_t pi_link_rx_seq_gap_count(void);

#endif 
