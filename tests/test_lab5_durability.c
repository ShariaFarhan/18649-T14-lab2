#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "block_log.h"
#define SIZE (3*L5_BANK_BYTES)
#define PAGE 4096u
static unsigned char cache[SIZE],disk[SIZE],baseline[SIZE];
static bool dirty[SIZE/PAGE],reorder,barriers=true;
static unsigned operation,cut;
static jmp_buf power_loss;
static void checkpoint(void){if(cut&&++operation==cut)longjmp(power_loss,1);}
static int read_cache(void*v,uint32_t at,void*d,unsigned n)
{(void)v;assert(at+n<=SIZE);memcpy(d,cache+at,n);return 0;}
static int program_cache(void*v,uint32_t at,const void*d,unsigned n)
{
    (void)v;assert(at+n<=SIZE);checkpoint();memcpy(cache+at,d,n);
    for(unsigned p=at/PAGE;p<=(at+n-1)/PAGE;p++)dirty[p]=true;
    /* Model legal page-cache writeback: publish header page before data pages. */
    if(reorder && at%L5_BANK_BYTES<PAGE){unsigned p=at/PAGE;memcpy(disk+p*PAGE,cache+p*PAGE,PAGE);dirty[p]=false;}
    checkpoint();return 0;
}
static int erase_cache(void*v,uint32_t at,unsigned n)
{(void)v;checkpoint();memset(cache+at,255,n);for(unsigned p=at/PAGE;p<(at+n)/PAGE;p++)dirty[p]=true;checkpoint();return 0;}
static int sync_cache(void*v)
{
    (void)v;checkpoint();
    if(barriers)for(unsigned p=0;p<SIZE/PAGE;p++)if(dirty[p]){memcpy(disk+p*PAGE,cache+p*PAGE,PAGE);dirty[p]=false;}
    checkpoint();return 0;
}
static void recover(void){memcpy(cache,disk,SIZE);memset(dirty,0,sizeof(dirty));operation=0;}
int main(void)
{
    memset(cache,255,SIZE);memset(disk,255,SIZE);
    struct l5_block_ops io={NULL,read_cache,program_cache,erase_cache,sync_cache};
    struct l5_block_log log;assert(!l5_block_mount(&log,&io));struct l5_storage s=l5_block_storage(&log);
    struct l5_meta m={.term=1,.voted_for=1};assert(!s.save(s.context,&m));
    for(unsigned i=1;i<=1000;i++){struct l5_entry e={1,i,l5_event(L5_EVENT,2,0,2,i)};assert(!s.append(s.context,&e));}
    m.commit=m.applied=1000;assert(!s.save(s.context,&m));
    while(log.meta_slot<510){m.term++;assert(!s.save(s.context,&m));}
    memcpy(baseline,disk,SIZE);unsigned failures[2]={0,0};
    /* Negative control must expose the original bug, positive case none. */
    for(unsigned mode=0;mode<2;mode++)for(unsigned boundary=1;boundary<=180;boundary++){
        memcpy(disk,baseline,SIZE);recover();cut=0;reorder=false;barriers=true;
        assert(!l5_block_mount(&log,&io));s=l5_block_storage(&log);
        uint32_t last;assert(!s.load(s.context,&m,&last));assert(last==1000);
        m.term++;operation=0;cut=boundary;reorder=true;barriers=mode!=0;
        if(!setjmp(power_loss))(void)s.save(s.context,&m);
        cut=0;reorder=false;barriers=true;recover();
        int rc=l5_block_mount(&log,&io);s=l5_block_storage(&log);
        if(rc||s.load(s.context,&m,&last)||last!=1000||m.commit!=1000||m.applied!=1000)failures[mode]++;
        else for(unsigned i=1;i<=1000;i++){
            struct l5_entry e;assert(!s.read(s.context,i,&e));assert(e.event==l5_event(L5_EVENT,2,0,2,i));
        }
    }
    assert(failures[0]>0);assert(failures[1]==0);
    printf("PASS durability: %u old-model failures reproduced; zero committed-prefix losses across 180 barrier cut points\n",failures[0]);
}
