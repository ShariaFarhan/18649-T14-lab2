#ifndef L5_POSIX_STORE_H
#define L5_POSIX_STORE_H
#include "block_log.h"
struct l5_posix_store {int fd;struct l5_block_log block;struct l5_storage inner;};
int l5_posix_open(struct l5_posix_store *,const char *path,struct l5_storage *);
void l5_posix_close(struct l5_posix_store *);
#endif
