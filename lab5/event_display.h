#ifndef L5_EVENT_DISPLAY_H
#define L5_EVENT_DISPLAY_H
#include "raft.h"
#include <limits.h>
struct l5_event_display {char directory[4096];};
int l5_display_open(struct l5_event_display *,const char *log_path);
int l5_display_record(struct l5_event_display *,const struct l5_entry *,bool replayed);
int l5_display_repair(struct l5_event_display *,const struct l5_storage *,uint32_t commit);
#endif
