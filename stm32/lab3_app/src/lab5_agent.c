#include <errno.h>
#include <string.h>
#include <time.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/atomic.h>
#include "lab5_agent.h"
#include "block_log.h"
#include "tx_queue.h"
#include "runtime.h"
#include "blinkers.h"
#include "lab3_can_catalog.h"
#include <zephyr/posix/time.h>
static const struct gpio_dt_spec pins[]={
    GPIO_DT_SPEC_GET(DT_ALIAS(lab5_pps),gpios),GPIO_DT_SPEC_GET(DT_ALIAS(lab5_leader),gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(lab5_sync),gpios),GPIO_DT_SPEC_GET(DT_ALIAS(lab5_ref_rx),gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(lab5_hazard),gpios)};
static struct l5_node node;static struct l5_block_log log_store;
static const struct flash_area *flash;
static struct l5_snapshot published;
static struct k_spinlock snapshot_lock,cycles_lock;
static atomic_t ready;
static atomic_t output_fault;
#if NODE_ID == 2
static const struct gpio_dt_spec relay_marker=GPIO_DT_SPEC_GET(DT_ALIAS(relay_tx),gpios);
#endif
static struct k_spinlock lights_lock;
static uint8_t requested_mode;
static bool requested_error;
static uint32_t requested_epoch;
static lab5_board_sender send_frame;
bool lab5_board_fault(void){return atomic_get(&output_fault)!=0;}
static void signal_pin(unsigned index,bool level)
{if(gpio_pin_set_dt(&pins[index],level))atomic_set(&output_fault,1);}
struct incoming {struct can_frame frame;uint64_t at;uint8_t bus;bool legacy,fresh;};
K_MSGQ_DEFINE(lab5_incoming,sizeof(struct incoming),128,8);
static struct l5_tx_queue outgoing[2];
uint64_t lab5_local_us(void)
{
    static uint32_t last;static uint64_t cycles;
    k_spinlock_key_t key=k_spin_lock(&cycles_lock);uint32_t now=k_cycle_get_32();
    cycles+=(uint32_t)(now-last);last=now;uint64_t value=cycles;k_spin_unlock(&cycles_lock,key);
    return k_cyc_to_us_floor64(value);
}
static int flash_read_log(void*v,uint32_t at,void*d,unsigned n){ARG_UNUSED(v);return flash_area_read(flash,at,d,n);}
static int flash_write_log(void*v,uint32_t at,const void*d,unsigned n){ARG_UNUSED(v);return flash_area_write(flash,at,d,n);}
static int flash_erase_log(void*v,uint32_t at,unsigned n)
{
    ARG_UNUSED(v);uint32_t words[32];bool erased=true;
    for(unsigned i=0;i<n&&erased;i+=sizeof(words)){
        if(flash_area_read(flash,at+i,words,sizeof(words)))return -EIO;
        for(unsigned j=0;j<ARRAY_SIZE(words);j++)if(words[j]!=UINT32_MAX){erased=false;break;}
    }
    return erased?0:flash_area_erase(flash,at,n);
}
static void output(void*v,unsigned buses,uint16_t id,const uint8_t*d,bool relay)
{
    ARG_UNUSED(v);
    for(unsigned b=0;b<2;b++)if(buses&(1u<<b))l5_tx_add(&outgoing[b],id,d,node.now_us,relay);
}
static void flush(uint64_t now)
{
    for(unsigned b=0;b<2;b++){
        if(lab3_local_failed()){outgoing[b].count=0;continue;}
        struct l5_tx_item item;unsigned slot;
        if(!l5_tx_pick(&outgoing[b],now,&item,&slot))continue;
        l5_tx_reference_delay(&item,now);
        struct can_frame f={.id=item.id,.dlc=8};memcpy(f.data,item.data,8);
        int rc=send_frame(b,&f);
#if NODE_ID == 2
        if(!rc&&item.relay&&gpio_pin_toggle_dt(&relay_marker))atomic_set(&output_fault,1);
#endif
        if(!rc || (rc!=-EAGAIN&&rc!=-EBUSY))l5_tx_remove(&outgoing[b],slot);
    }
}
static void entry(void*v,const struct l5_entry*e,bool replayed)
{
    ARG_UNUSED(v);
    printk("LAB5 LOG term=%u index=%u kind=%u node=%u bus=%u client=%u request=%u %s\n",
           e->term,e->index,l5_kind(e->event),l5_node(e->event),l5_bus(e->event),l5_client(e->event),l5_request(e->event),replayed?"replayed":"live");
}
static void trace(void*v,const struct l5_message*m,bool tx)
{
    ARG_UNUSED(v);if(m->type>6)return;
    printk("LAB5 RAFT %s type=%u src=%u dst=%u term=%u index=%u commit=%u ok=%u\n",
           tx?"TX":"RX",m->type,m->source,m->destination,m->term,m->index,m->commit,m->ok);
}
static void set_wallclock(void*v,uint64_t global)
{
    ARG_UNUSED(v);struct timespec ts={.tv_sec=global/1000000,.tv_nsec=(global%1000000)*1000};
    if(clock_settime(CLOCK_REALTIME,&ts))atomic_set(&output_fault,1);
}
struct l5_snapshot lab5_board_snapshot(void)
{k_spinlock_key_t key=k_spin_lock(&snapshot_lock);struct l5_snapshot s=published;k_spin_unlock(&snapshot_lock,key);return s;}
void lab5_board_lights(uint8_t mode,bool error,uint32_t epoch)
{k_spinlock_key_t key=k_spin_lock(&lights_lock);requested_mode=mode;requested_error=error;requested_epoch=epoch;k_spin_unlock(&lights_lock,key);}
unsigned lab5_board_route(void)
{if(!atomic_get(&ready))return 3;struct l5_snapshot s=lab5_board_snapshot();return l5_snapshot_route(&s,0);}
bool lab5_board_capture(unsigned bus,const struct can_frame*f)
{
    if(!l5_wire_id(f->id))return false;
    if(!atomic_get(&ready)||f->flags&(CAN_FRAME_IDE|CAN_FRAME_RTR))return true;
    struct incoming in={.frame=*f,.at=lab5_local_us(),.bus=bus};
    (void)k_msgq_put(&lab5_incoming,&in,K_NO_WAIT);return true;
}
void lab5_board_legacy(unsigned bus,const struct can_frame*f,bool fresh)
{
    if(!atomic_get(&ready))return;
    struct incoming in={.frame=*f,.at=lab5_local_us(),.bus=bus,.legacy=true,.fresh=fresh};
    (void)k_msgq_put(&lab5_incoming,&in,K_NO_WAIT);
}
static void agent_worker(void*a,void*b,void*c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);uint64_t status_at=0;
    for(;;){
        struct incoming in;int rc=k_msgq_get(&lab5_incoming,&in,K_USEC(100));
        uint64_t now=lab5_local_us();
        if(!rc){
            if(in.legacy)l5_node_legacy(&node,in.bus,in.frame.id,in.frame.data,in.fresh,in.at);
            else{
                lab3_rx_mark(in.bus);
                if(in.frame.id>=0x45&&in.frame.id<=0x4f&&gpio_pin_toggle_dt(&pins[3]))atomic_set(&output_fault,1);
                l5_node_receive(&node,in.bus,in.frame.id,in.frame.data,in.frame.dlc,in.at,now);
            }
        }
        l5_node_tick(&node,now,lab3_local_failed(),false);flush(lab5_local_us());
        struct l5_snapshot s;l5_node_snapshot(&node,&s);
        k_spinlock_key_t key=k_spin_lock(&snapshot_lock);published=s;k_spin_unlock(&snapshot_lock,key);
        if(now>=status_at){
            printk("LAB5 NETWORK leader=%u term=%u commit=%u hash=%08x nodes=%x links=%u/%u/%u cockpit=%u fatal=%u time_pending=%u time_conflict=%u\n",
                s.leader,s.term,s.commit,s.log_hash,s.network.nodes,s.network.links[1],s.network.links[2],s.network.links[3],s.network.cockpit_ok,s.fatal,s.time_pending,s.time_conflict);
            status_at=now+1000000;
        }
        lab3_progress(WORK_LAB5);
    }
}
static void clock_worker(void*a,void*b,void*c)
{
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    uint64_t last_second=UINT64_MAX,high_until=0,last_global=0;uint8_t oldmode=255;bool oldphase=false;
    for(;;){
        uint64_t local=lab5_local_us();struct l5_snapshot s=lab5_board_snapshot();
        uint64_t global=l5_global_gettime(&s.clock,local);
        if(s.clock.established){if(global<last_global)global=last_global;last_global=global;}
        bool sync=s.synced&&(s.is_leader||local-s.clock.last_ref<s.reference_timeout_us);
        signal_pin(1,s.is_leader&&!lab3_local_failed());
        signal_pin(2,sync);
        if(s.clock.established){
            if(last_second==UINT64_MAX)last_second=global/1000000;
            if(global/1000000!=last_second){
                last_second=global/1000000;high_until=local+1000;signal_pin(0,true);
                printk("LAB5 PPS global_us=%llu local_us=%llu drift_ppb=%d ratio=%d/1000000000 correction_us=%lld SYNC=%d\n",
                       global,local,s.clock.drift_ppb,1000000000+s.clock.drift_ppb,l5_clock_correction(&s.clock,local),sync);
            }
        }
        if(local>=high_until)signal_pin(0,false);
        k_spinlock_key_t key=k_spin_lock(&lights_lock);
        bool error=requested_error;uint8_t mode=requested_mode;uint32_t epoch=requested_epoch;
        k_spin_unlock(&lights_lock,key);
        error=error||lab3_local_failed()||!s.cockpit_ok||s.fatal;
        if(error)mode=BLINK_MODE_HAZARD;
        bool phase=l5_blink_phase(global,mode,epoch,l5_hazard_half_us(&s));
        if(mode!=oldmode||phase!=oldphase){
            if(blinkers_apply_phase((enum blinker_mode)mode,phase))atomic_set(&output_fault,1);
            else {oldmode=mode;oldphase=phase;}
        }
        signal_pin(4,mode==BLINK_MODE_HAZARD&&phase);
        lab3_progress(WORK_CLOCK);k_sleep(K_USEC(100));
    }
}
K_THREAD_DEFINE(lab5_agent_thread,4096,agent_worker,NULL,NULL,NULL,4,0,SYS_FOREVER_MS);
K_THREAD_DEFINE(lab5_clock_thread,1536,clock_worker,NULL,NULL,NULL,2,0,SYS_FOREVER_MS);
int lab5_board_init(lab5_board_sender sender)
{
    send_frame=sender;
#if NODE_ID == 2
    if(!gpio_is_ready_dt(&relay_marker))return -ENODEV;
    int marker_rc=gpio_pin_configure_dt(&relay_marker,GPIO_OUTPUT_INACTIVE);if(marker_rc)return marker_rc;
#endif
    for(unsigned i=0;i<ARRAY_SIZE(pins);i++){
        if(!gpio_is_ready_dt(&pins[i]))return -ENODEV;
        int rc=gpio_pin_configure_dt(&pins[i],GPIO_OUTPUT_INACTIVE);if(rc)return rc;
    }
    int rc=flash_area_open(FIXED_PARTITION_ID(lab5_log),&flash);if(rc)return rc;
    if(flash->fa_size!=3*L5_BANK_BYTES)return -EINVAL;
    struct l5_block_ops io={NULL,flash_read_log,flash_write_log,flash_erase_log,NULL};
    if((rc=l5_block_mount(&log_store,&io)))return rc;
    struct l5_storage store=l5_block_storage(&log_store);
    if((rc=l5_node_init(&node,NODE_ID,&store,NULL,output,entry,set_wallclock,lab5_local_us())))return rc;
    node.trace=trace;
    l5_node_snapshot(&node,&published);requested_error=true;atomic_set(&ready,1);
    k_thread_start(lab5_agent_thread);k_thread_start(lab5_clock_thread);return 0;
}
