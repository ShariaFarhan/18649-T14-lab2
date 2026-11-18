#define _POSIX_C_SOURCE 200809L
#include "event_display.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
int l5_display_open(struct l5_event_display*d,const char*log)
{
    if(snprintf(d->directory,sizeof(d->directory),"%s.events",log)>=(int)sizeof(d->directory))return -ENAMETOOLONG;
    if(mkdir(d->directory,0700)&&errno!=EEXIST)return -errno;
    struct stat st;if(lstat(d->directory,&st)||!S_ISDIR(st.st_mode))return -EINVAL;
    return 0;
}
int l5_display_record(struct l5_event_display*d,const struct l5_entry*e,bool replayed)
{
    if(l5_kind(e->event)!=L5_EVENT)return 0;
    char path[4200],temporary[4200],data[256];
    snprintf(path,sizeof(path),"%s/%08u.json",d->directory,e->index);
    struct stat st;if(!lstat(path,&st))return S_ISREG(st.st_mode)?0:-EINVAL;
    if(errno!=ENOENT)return -errno;
    snprintf(temporary,sizeof(temporary),"%s/%08u.tmp",d->directory,e->index);
    int size=snprintf(data,sizeof(data),"{\"term\":%u,\"index\":%u,\"source\":%u,\"counter\":%u,\"replayed\":%s}\n",
                      e->term,e->index,l5_node(e->event),l5_request(e->event),replayed?"true":"false");
    int fd=open(temporary,O_WRONLY|O_CREAT|O_TRUNC,0600);if(fd<0)return -errno;
    int rc=write(fd,data,size)==size&&fsync(fd)==0?0:-EIO;close(fd);
    if(!rc&&rename(temporary,path))rc=-errno;
    if(!rc){int dir=open(d->directory,O_RDONLY|O_DIRECTORY);if(dir<0)rc=-errno;else{if(fsync(dir))rc=-errno;close(dir);}}
    return rc;
}
int l5_display_repair(struct l5_event_display*d,const struct l5_storage*s,uint32_t commit)
{
    for(uint32_t i=1;i<=commit;i++){
        struct l5_entry e;int rc=s->read(s->context,i,&e);if(rc)return rc;
        if((rc=l5_display_record(d,&e,true)))return rc;
    }
    return 0;
}
