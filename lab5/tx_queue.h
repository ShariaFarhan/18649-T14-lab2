#ifndef L5_TX_QUEUE_H
#define L5_TX_QUEUE_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
struct l5_tx_item {uint16_t id;uint8_t data[8];bool relay;uint64_t at,order;};
struct l5_tx_queue {struct l5_tx_item items[128];unsigned count;uint64_t order;uint32_t dropped;};
static inline uint64_t l5_tx_lifetime(uint16_t id)
{
    if(id==0x010 || (id>=0x100&&id<=0x102) || (id>=0x45&&id<=0xff))return 20000;
    if(id==0x020)return 100000;
    return 200000;
}
static inline bool l5_tx_add(struct l5_tx_queue*q,uint16_t id,const uint8_t*d,uint64_t at,bool relay)
{
    unsigned slot=q->count;
    if(id<0x100)for(unsigned i=0;i<q->count;i++)if(q->items[i].id==id){slot=i;break;}
    if(slot==q->count){
        if(q->count==128){
            slot=0;for(unsigned i=1;i<128;i++)if(q->items[i].id>q->items[slot].id)slot=i;
            q->dropped++;if(q->items[slot].id<=id)return false;
        }else q->count++;
    }
    q->items[slot]=(struct l5_tx_item){.id=id,.at=at,.order=q->order++,.relay=relay};memcpy(q->items[slot].data,d,8);return true;
}
static inline bool l5_tx_pick(struct l5_tx_queue*q,uint64_t now,struct l5_tx_item*out,unsigned*slot)
{
    for(unsigned i=0;i<q->count;){
        if(now-q->items[i].at>l5_tx_lifetime(q->items[i].id)){q->items[i]=q->items[--q->count];q->dropped++;}else i++;
    }
    if(!q->count)return false;
    unsigned first=0;
    for(unsigned i=1;i<q->count;i++)if(q->items[i].id<q->items[first].id ||
        (q->items[i].id==q->items[first].id&&q->items[i].order<q->items[first].order))first=i;
    *out=q->items[first];*slot=first;return true;
}
static inline void l5_tx_remove(struct l5_tx_queue*q,unsigned slot)
{if(slot<q->count)q->items[slot]=q->items[--q->count];}
static inline void l5_tx_reference_delay(struct l5_tx_item*o,uint64_t now)
{
    if(o->id>=0x45&&o->id<=0x4f){
        uint64_t stamp=0;for(unsigned j=0;j<6;j++)stamp|=(uint64_t)o->data[j+2]<<(8*j);
        stamp+=now-o->at;for(unsigned j=0;j<6;j++)o->data[j+2]=stamp>>(8*j);
    }
    else if(o->id>=0xc0&&o->id<=0xff){
        uint32_t stamp=0;for(unsigned j=0;j<4;j++)stamp|=(uint32_t)o->data[j+4]<<(8*j);
        stamp+=(uint32_t)(now-o->at);for(unsigned j=0;j<4;j++)o->data[j+4]=stamp>>(8*j);
    }
}
#endif
