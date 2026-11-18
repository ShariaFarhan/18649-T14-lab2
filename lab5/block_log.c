#include "block_log.h"
#include <errno.h>
#include <string.h>
#define MAGIC 0x354c4132u
#define META_SLOTS ((L5_BANK_HEADER-32u)/16u)
static uint32_t crc(const uint32_t *words,unsigned count)
{
    uint32_t c=0xffffffff;
    for(unsigned i=0;i<count;i++)for(unsigned byte=0;byte<4;byte++){
        c^=(words[i]>>(byte*8))&255;
        for(unsigned b=0;b<8;b++)c=(c>>1)^(0xedb88320u&-(c&1));
    }
    return ~c;
}
static int read_at(struct l5_block_log *s,uint32_t at,void *d,unsigned n)
{return s->io.read(s->io.context,at,d,n);}
static int write_at(struct l5_block_log *s,uint32_t at,const void *d,unsigned n)
{return s->io.program(s->io.context,at,d,n);}
static int durable(struct l5_block_log *s)
{return s->io.sync?s->io.sync(s->io.context):0;}
static uint32_t entry_at(struct l5_block_log *s,uint32_t index)
{unsigned logical=(index-1)/L5_BANK_ENTRIES;return s->bank[logical]*L5_BANK_BYTES+L5_BANK_HEADER+((index-1)%L5_BANK_ENTRIES)*8;}
static bool valid_entry(const uint32_t w[2])
{
    uint32_t checked[2]={w[0]>>16,w[1]};
    return !(w[0]&1u) && checked[0]>0 && checked[0]<65535 && l5_event_valid(w[1]) &&
           ((w[0]>>3)&0x1fffu)==(crc(checked,2)&0x1fffu);
}
static int new_bank(struct l5_block_log *s,unsigned logical,int physical,uint32_t generation,uint32_t keep)
{
    uint32_t base=physical*L5_BANK_BYTES;
    int rc=s->io.erase(s->io.context,base,L5_BANK_BYTES);if(rc)return rc;
    if((rc=durable(s)))return rc; /* Invalidate any old MAGIC before changing its header. */
    uint32_t header[8]={0xffffffff,generation,logical,0,0,0,0,0};header[3]=crc(&header[1],2);
    if((rc=write_at(s,base+4,&header[1],28)))return rc;
    if(s->bank[logical]>=0){
        uint32_t count=keep>logical*L5_BANK_ENTRIES?keep-logical*L5_BANK_ENTRIES:0;
        if(count>L5_BANK_ENTRIES)count=L5_BANK_ENTRIES;
        uint32_t buf[32];
        for(uint32_t i=0;i<count;){
            unsigned n=count-i>16?16:count-i;
            if((rc=read_at(s,s->bank[logical]*L5_BANK_BYTES+L5_BANK_HEADER+i*8,buf,n*8)))return rc;
            if((rc=write_at(s,base+L5_BANK_HEADER+i*8,buf,n*8)))return rc;
            i+=n;
        }
    }
    if(logical==0){
        uint32_t meta[4]={s->meta.term,s->meta.client_seq,s->meta.voted_for,0};meta[3]=crc(meta,3);
        if((rc=write_at(s,base+32,meta,16)))return rc;
    }
    if((rc=durable(s)))return rc; /* Entire copied prefix must precede publication. */
    uint32_t magic=MAGIC;if((rc=write_at(s,base,&magic,4)))return rc;
    if((rc=durable(s)))return rc;
    s->bank[logical]=physical;s->generation[logical]=generation;
    if(logical==0)s->meta_slot=1;
    return 0;
}
static int rewrite(struct l5_block_log *s,unsigned logical,uint32_t keep)
{
    int spare=0;while(spare==s->bank[0]||spare==s->bank[1])spare++;
    if(spare>2 || s->generation[logical]==UINT32_MAX)return -ENOSPC;
    return new_bank(s,logical,spare,s->generation[logical]+1,keep);
}
int l5_block_mount(struct l5_block_log *s,const struct l5_block_ops *ops)
{
    memset(s,0,sizeof(*s));s->io=*ops;s->bank[0]=s->bank[1]=-1;
    for(unsigned p=0;p<3;p++){
        uint32_t h[8];if(read_at(s,p*L5_BANK_BYTES,h,sizeof(h)))return -EIO;
        if(h[0]!=MAGIC||h[2]>1||h[3]!=crc(&h[1],2))continue;
        unsigned l=h[2];if(s->bank[l]<0||h[1]>s->generation[l]){s->bank[l]=p;s->generation[l]=h[1];}
    }
    for(unsigned l=0;l<2;l++)if(s->bank[l]<0){
        int p=0;while(p==s->bank[0]||p==s->bank[1])p++;
        if(new_bank(s,l,p,1,0))return -EIO;
    }
    for(unsigned i=0;i<META_SLOTS;i++){
        uint32_t m[4];if(read_at(s,s->bank[0]*L5_BANK_BYTES+32+i*16,m,16))return -EIO;
        if(m[3]!=crc(m,3)||m[2]>3){s->meta_slot=i;break;}
        s->meta.term=m[0];s->meta.client_seq=m[1];s->meta.voted_for=m[2];s->meta_slot=i+1;
    }
    bool committed_prefix=true,applied_prefix=true;
    for(uint32_t i=1;i<=30000;i++){
        uint32_t w[2];if(read_at(s,entry_at(s,i),w,8))return -EIO;
        if(!valid_entry(w))break;
        s->last=i;
        if(w[0]&2)committed_prefix=false;
        if(w[0]&4)applied_prefix=false;
        if(committed_prefix)s->meta.commit=i;
        if(applied_prefix && committed_prefix)s->meta.applied=i;
    }
    return 0;
}
static int load(void *p,struct l5_meta *m,uint32_t *last)
{struct l5_block_log *s=p;*m=s->meta;*last=s->last;return 0;}
static int read_entry(void *p,uint32_t index,struct l5_entry *e)
{
    struct l5_block_log *s=p;uint32_t w[2];
    if(!index||index>s->last)return -EINVAL;
    int rc=read_at(s,entry_at(s,index),w,8);if(rc)return rc;
    if(!valid_entry(w))return -EIO;
    *e=(struct l5_entry){w[0]>>16,index,w[1]};return 0;
}
static int append_entry(void *p,const struct l5_entry *e)
{
    struct l5_block_log *s=p;
    if(e->index!=s->last+1||e->index>30000||!e->term||e->term>=65535||!l5_event_valid(e->event))return -EINVAL;
    uint32_t at=entry_at(s,e->index),old[2];int rc=read_at(s,at,old,8);if(rc)return rc;
    if(old[0]!=UINT32_MAX||old[1]!=UINT32_MAX){
        if((rc=rewrite(s,(e->index-1)/L5_BANK_ENTRIES,s->last)))return rc;
        at=entry_at(s,e->index);
    }
    uint32_t checked[2]={e->term,e->event};
    uint32_t word=(e->term<<16)|((crc(checked,2)&0x1fffu)<<3)|7u;
    if((rc=write_at(s,at+4,&e->event,4)))return rc;
    if((rc=write_at(s,at,&word,4)))return rc;
    if((rc=durable(s)))return rc;
    word&=~1u; /* Publish validity only after every data/CRC byte is durable. */
    if((rc=write_at(s,at,&word,4)))return rc;
    if((rc=durable(s)))return rc;
    s->last=e->index;return 0;
}
static int truncate_log(void *p,uint32_t keep)
{
    struct l5_block_log *s=p;
    if(keep<s->meta.commit||keep>s->last)return -EINVAL;
    if(keep==s->last)return 0;
    int rc;
    if(keep<L5_BANK_ENTRIES && s->last>L5_BANK_ENTRIES)
        if((rc=rewrite(s,1,keep)))return rc;
    if((rc=rewrite(s,keep<L5_BANK_ENTRIES?0:1,keep)))return rc;
    s->last=keep;return 0;
}
static int save_meta(void *p,const struct l5_meta *m)
{
    struct l5_block_log *s=p;int rc;
    if(m->commit>s->last||m->applied>m->commit||m->commit<s->meta.commit||m->applied<s->meta.applied)return -EINVAL;
    if(m->term!=s->meta.term||m->client_seq!=s->meta.client_seq||m->voted_for!=s->meta.voted_for){
        uint32_t disk[4]={m->term,m->client_seq,m->voted_for,0};disk[3]=crc(disk,3);
        if(s->meta_slot>=META_SLOTS)if((rc=rewrite(s,0,s->last)))return rc;
        uint32_t at=s->bank[0]*L5_BANK_BYTES+32+s->meta_slot*16,old[4];
        if((rc=read_at(s,at,old,16)))return rc;
        if(old[0]!=UINT32_MAX||old[1]!=UINT32_MAX||old[2]!=UINT32_MAX||old[3]!=UINT32_MAX){
            if((rc=rewrite(s,0,s->last)))return rc;
            at=s->bank[0]*L5_BANK_BYTES+32+s->meta_slot*16;
        }
        uint32_t staged[4];memcpy(staged,disk,sizeof(staged));
        staged[2]|=0x80000000u; /* Invalid voted_for until all metadata/CRC is durable. */
        if((rc=write_at(s,at,staged,16)))return rc;
        if((rc=durable(s)))return rc;
        if((rc=write_at(s,at+8,&disk[2],4)))return rc;
        if((rc=durable(s)))return rc;
        s->meta_slot++;
    }
    for(uint32_t i=s->meta.commit+1;i<=m->commit;i++){
        uint32_t w;if((rc=read_at(s,entry_at(s,i),&w,4)))return rc;w&=~2u;
        if((rc=write_at(s,entry_at(s,i),&w,4)))return rc;
    }
    for(uint32_t i=s->meta.applied+1;i<=m->applied;i++){
        uint32_t w;if((rc=read_at(s,entry_at(s,i),&w,4)))return rc;w&=~4u;
        if((rc=write_at(s,entry_at(s,i),&w,4)))return rc;
    }
    if((rc=durable(s)))return rc;
    s->meta=*m;return 0;
}
struct l5_storage l5_block_storage(struct l5_block_log *s)
{return (struct l5_storage){s,load,read_entry,append_entry,truncate_log,save_meta};}
