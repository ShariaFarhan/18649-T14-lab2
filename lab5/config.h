#ifndef L5_CONFIG_H
#define L5_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
struct l5_config {
    uint8_t n,default_leader;
    uint16_t heartbeat_ms,missed_heartbeats,election_jitter_ms;
    uint32_t log_size,append_ms,event_ms,network_delay_us,seed;
    bool debug;
    uint32_t path_delay_us[4][4];
    uint32_t startup_grace_ms;
};
/* Generated/checked against config.json by tools/lab5_config.py. */
#include "config_generated.h"
#endif
