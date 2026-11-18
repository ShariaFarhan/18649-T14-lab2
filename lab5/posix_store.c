#define _POSIX_C_SOURCE 200809L
#include "posix_store.h"
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
static int rd(void*v,uint32_t at,void*d,unsigned n)
{struct l5_posix_store*s=v;return pread(s->fd,d,n,at)==(ssize_t)n?0:-EIO;}
static int wr(void*v,uint32_t at,const void*d,unsigned n)
{struct l5_posix_store*s=v;return pwrite(s->fd,d,n,at)==(ssize_t)n?0:-EIO;}
static int barrier(void*v){return fsync(((struct l5_posix_store*)v)->fd);}
static int sync_parent(const char*path)
{
    char*copy=strdup(path);if(!copy)return -ENOMEM;
    char*slash=strrchr(copy,'/');const char*dir=".";
    if(slash){if(slash==copy)slash[1]=0;else *slash=0;dir=copy;}
    int fd=open(dir,O_RDONLY|O_DIRECTORY),rc=fd<0?-errno:0;
    if(fd>=0){if(fsync(fd))rc=-errno;close(fd);}free(copy);return rc;
}
static int erase(void*v,uint32_t at,unsigned n)
{unsigned char b[1024];memset(b,255,sizeof(b));for(unsigned i=0;i<n;i+=sizeof(b)){int rc=wr(v,at+i,b,sizeof(b));if(rc)return rc;}return 0;}
static int load(void*v,struct l5_meta*m,uint32_t*n){struct l5_posix_store*s=v;return s->inner.load(s->inner.context,m,n);}
static int readlog(void*v,uint32_t i,struct l5_entry*e){struct l5_posix_store*s=v;return s->inner.read(s->inner.context,i,e);}
static int append(void*v,const struct l5_entry*e){struct l5_posix_store*s=v;int rc=s->inner.append(s->inner.context,e);return rc?rc:fsync(s->fd);}
static int truncate_log(void*v,uint32_t i){struct l5_posix_store*s=v;int rc=s->inner.truncate(s->inner.context,i);return rc?rc:fsync(s->fd);}
static int save(void*v,const struct l5_meta*m){struct l5_posix_store*s=v;int rc=s->inner.save(s->inner.context,m);return rc?rc:fsync(s->fd);}
int l5_posix_open(struct l5_posix_store*s,const char*path,struct l5_storage*out)
{
    memset(s,0,sizeof(*s));s->fd=open(path,O_RDWR|O_CREAT,0600);if(s->fd<0)return -errno;
    if(flock(s->fd,LOCK_EX|LOCK_NB)){close(s->fd);s->fd=-1;return -EBUSY;}
    struct stat st;if(fstat(s->fd,&st))goto fail;
    if(st.st_size==0){if(erase(s,0,3*L5_BANK_BYTES)||fsync(s->fd))goto fail;}
    else if(st.st_size!=3*L5_BANK_BYTES)goto fail;
    struct l5_block_ops io={s,rd,wr,erase,barrier};if(l5_block_mount(&s->block,&io)||fsync(s->fd)||sync_parent(path))goto fail;
    s->inner=l5_block_storage(&s->block);*out=(struct l5_storage){s,load,readlog,append,truncate_log,save};return 0;
fail:close(s->fd);s->fd=-1;return -EIO;
}
void l5_posix_close(struct l5_posix_store*s){if(s->fd>=0){fsync(s->fd);close(s->fd);s->fd=-1;}}
