#ifndef LAB3_SUPERVISION_H
#define LAB3_SUPERVISION_H
#include <stdbool.h>
#include <stdint.h>
#define LAB3_PROGRESS_LIMIT_MS 200u
static inline bool lab3_progress_fresh(uint32_t now,uint32_t last)
{ return (uint32_t)(now-last)<LAB3_PROGRESS_LIMIT_MS; }
#endif
