#include <pthread.h>
#include <errno.h>
#define K_NO_WAIT 0
#define K_FOREVER (-1)
struct k_mutex { pthread_mutex_t mutex; };
static inline int k_mutex_lock(struct k_mutex *m,int wait)
{ return -(wait==K_NO_WAIT ? pthread_mutex_trylock(&m->mutex) : pthread_mutex_lock(&m->mutex)); }
static inline int k_mutex_unlock(struct k_mutex *m) { return -pthread_mutex_unlock(&m->mutex); }
