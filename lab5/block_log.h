#ifndef L5_BLOCK_LOG_H
#define L5_BLOCK_LOG_H
#include "raft.h"
#define L5_BANK_BYTES 131072u
#define L5_BANK_HEADER 8192u
#define L5_BANK_ENTRIES 15000u
/* Two logical log banks plus one copy-on-write scratch bank. No log array in RAM. */
struct l5_block_ops {
    void *context;
    int (*read)(void *,uint32_t offset,void *,unsigned bytes);
    int (*program)(void *,uint32_t offset,const void *,unsigned bytes);
    int (*erase)(void *,uint32_t offset,unsigned bytes);
    /* NULL only for a backend whose program/erase return after durability. */
    int (*sync)(void *);
};
struct l5_block_log {
    struct l5_block_ops io;
    int bank[2];uint32_t generation[2],last,meta_slot;
    struct l5_meta meta;
};
int l5_block_mount(struct l5_block_log *,const struct l5_block_ops *);
struct l5_storage l5_block_storage(struct l5_block_log *);
#endif
