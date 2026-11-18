#include "time_sync.h"
#include <string.h>
static int64_t clamp(int64_t x,int64_t lo,int64_t hi){return x<lo?lo:x>hi?hi:x;}
void l5_clock_init(struct l5_clock *c,uint64_t local,uint64_t global)
{memset(c,0,sizeof(*c));c->local_anchor=local;c->global_anchor=global;c->last_return=global;}
int64_t l5_clock_correction(const struct l5_clock *c,uint64_t local)
{
    uint64_t dt=local>=c->local_anchor?local-c->local_anchor:0;
    /* Normal correction <=2500ppm (50us/20ms); nonce-verified recovery uses
     * <=5% while SYNC is low. Neither path steps an established clock. */
    int64_t limit=(int64_t)(dt/(c->recovering?20:400));
    return (int64_t)(dt/1000)*c->drift_ppb/1000000+clamp(c->phase_error,-limit,limit);
}
uint64_t l5_global_gettime(struct l5_clock *c,uint64_t local)
{
    uint64_t dt=local>=c->local_anchor?local-c->local_anchor:0;
    int64_t correction=l5_clock_correction(c,local);
    uint64_t value=c->global_anchor+dt;
    value=correction<0 && (uint64_t)-correction>value ? 0 : (uint64_t)((int64_t)value+correction);
    if(value<c->last_return)value=c->last_return;
    c->last_return=value;return value;
}
bool l5_clock_reference(struct l5_clock *c,uint64_t local,uint64_t remote,
                        uint32_t delay,unsigned leader,uint32_t term)
{
    if(c->last_ref && local<=c->last_ref)return false;
    uint64_t target=remote+delay;
    if(!c->initialized){
        c->local_anchor=local;c->global_anchor=target;c->last_return=target;c->initialized=true;
        c->samples=0;c->previous_local=0;
    }
    uint64_t current=l5_global_gettime(c,local);
    bool same=c->leader==leader && c->term==term;
    if(same && c->previous_local && local>c->previous_local && remote>c->previous_leader){
        uint64_t f=local-c->previous_local,l=remote-c->previous_leader;
        if(f>=1000 && f<=1000000){
            int64_t ppb=((int64_t)l-(int64_t)f)*1000000000/(int64_t)f;
            if(ppb>=-2000000 && ppb<=2000000)c->drift_ppb+=(ppb-c->drift_ppb)/8;
        }
    } else { c->samples=0;c->drift_ppb=0; }
    c->phase_error=(int64_t)target-(int64_t)current;
    c->global_anchor=current;c->local_anchor=local;c->previous_local=local;
    c->previous_leader=remote;c->last_ref=local;c->leader=leader;c->term=term;
    if(c->samples<255)c->samples++;
    c->synced=c->samples>=2 && c->phase_error>-1000 && c->phase_error<1000;
    if(c->synced)c->recovering=false;
    if(c->synced)c->established=true;
    return true;
}
void l5_clock_master(struct l5_clock *c,uint64_t local,unsigned leader,uint32_t term)
{
    uint64_t now=l5_global_gettime(c,local);
    c->global_anchor=now;c->local_anchor=local;c->phase_error=0;c->drift_ppb=0;
    c->leader=leader;c->term=term;c->initialized=true;c->synced=true;c->established=true;c->recovering=false;c->last_ref=local;
}
void l5_clock_verified_epoch(struct l5_clock*c,uint64_t local,uint64_t global,unsigned leader,uint32_t term)
{
    if(!c->established){
        l5_clock_init(c,local,global);c->initialized=true;c->established=true;
    }else{
        uint64_t current=l5_global_gettime(c,local);
        c->global_anchor=current;c->local_anchor=local;c->phase_error=(int64_t)global-(int64_t)current;
    }
    c->leader=leader;c->term=term;c->last_ref=local;c->samples=0;c->synced=false;
    c->recovering=c->phase_error>1000||c->phase_error< -1000;
}
void l5_clock_holdover(struct l5_clock*c,uint64_t local)
{
    if(!c->recovering||local<c->local_anchor)return;
    uint64_t remaining=c->phase_error<0?(uint64_t)-c->phase_error:(uint64_t)c->phase_error;
    if((local-c->local_anchor)/20<remaining)return;
    /* The previously verified finite correction has completed even if its
     * master has since failed. It is now a usable holdover, not SYNC proof. */
    c->global_anchor=l5_global_gettime(c,local);c->local_anchor=local;
    c->phase_error=0;c->recovering=false;c->synced=false;
}
bool l5_blink_phase(uint64_t global,uint8_t mode,uint32_t start_ms,unsigned hazard_half)
{
    if(!mode)return false;
    if(mode==3)return (global/hazard_half)%2==0;
    uint64_t now_ms=global/1000,epoch=(now_ms&~UINT64_C(0xffffffff))|start_ms;
    if(epoch+UINT64_C(0x80000000)<now_ms)epoch+=UINT64_C(0x100000000);
    else if(epoch>now_ms+UINT64_C(0x80000000)&&epoch>=UINT64_C(0x100000000))epoch-=UINT64_C(0x100000000);
    return global>=epoch*1000 && ((global-epoch*1000)/500000)%2==0;
}
