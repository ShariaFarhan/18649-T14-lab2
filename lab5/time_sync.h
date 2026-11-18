#ifndef L5_TIME_SYNC_H
#define L5_TIME_SYNC_H
#include <stdbool.h>
#include <stdint.h>
struct l5_clock {
    uint64_t local_anchor,global_anchor,last_return,previous_local,previous_leader,last_ref;
    int64_t phase_error;
    int32_t drift_ppb;
    uint32_t term;
    uint8_t leader,samples;
    bool initialized,synced,established,recovering;
};
void l5_clock_init(struct l5_clock *,uint64_t local,uint64_t global);
uint64_t l5_global_gettime(struct l5_clock *,uint64_t local);
bool l5_clock_reference(struct l5_clock *,uint64_t local,uint64_t leader_time,
                        uint32_t delay_us,unsigned leader,uint32_t term);
void l5_clock_master(struct l5_clock *,uint64_t local,unsigned leader,uint32_t term);
void l5_clock_verified_epoch(struct l5_clock *,uint64_t local,uint64_t global,unsigned leader,uint32_t term);
void l5_clock_holdover(struct l5_clock *,uint64_t local);
int64_t l5_clock_correction(const struct l5_clock *,uint64_t local);
bool l5_blink_phase(uint64_t global_us,uint8_t mode,uint32_t start_ms,unsigned hazard_half_us);
#endif
