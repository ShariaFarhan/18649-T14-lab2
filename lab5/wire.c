#include "wire.h"
#include <string.h>
static void put32(uint8_t *d,uint32_t x){for(unsigned i=0;i<4;i++)d[i]=x>>(8*i);}
static uint32_t get32(const uint8_t*d){uint32_t x=0;for(unsigned i=0;i<4;i++)x|=(uint32_t)d[i]<<(8*i);return x;}
static uint32_t crc(const uint8_t*d,unsigned n){uint32_t c=~0u;for(unsigned i=0;i<n;i++){c^=d[i];for(unsigned b=0;b<8;b++)c=(c>>1)^(0xedb88320u&-(c&1));}return ~c;}
static uint64_t get48(const uint8_t*d){uint64_t x=0;for(unsigned i=0;i<6;i++)x|=(uint64_t)d[i]<<(8*i);return x;}
static void put48(uint8_t*d,uint64_t x){for(unsigned i=0;i<6;i++)d[i]=x>>(8*i);}
bool l5_wire_id(uint16_t id)
{return (id>=0x35&&id<=0x3f)||(id>=0x65&&id<=0x6f)||(id>=0x45&&id<=0x4f)||(id>=0x80&&id<=0xbf)||(id>=L5_SAMPLE_BASE&&id<L5_PROOF_BASE+64)||(id>L5_TIME_REPLY_BASE&&id<=L5_TIME_REPLY_BASE+3)||(id>=L5_SHORT_BASE&&id<0x480)||(id>=0x501&&id<=0x573);}
static uint16_t get16(const uint8_t*d){return d[0]|((uint16_t)d[1]<<8);}
static void put16(uint8_t*d,uint32_t v){d[0]=v;d[1]=v>>8;}
static uint16_t rpc_id(unsigned type,unsigned hop)
{return type==L5_TIME_REPLY?L5_TIME_REPLY_BASE+hop:L5_RPC_BASE+(type-1)*16+hop;}
void l5_wire_init(struct l5_wire*w,unsigned id,uint32_t delay,void*context,
                  void(*send)(void*,unsigned,uint16_t,const uint8_t*),
                  void(*message)(void*,const struct l5_message*,uint64_t),
                  void(*reference)(void*,unsigned,uint8_t,uint8_t,uint64_t,uint64_t,uint32_t))
{memset(w,0,sizeof(*w));w->id=id;w->delay_us=delay;for(unsigned i=0;i<4;i++)w->delay_by_hop[i]=delay;w->context=context;w->send_frame=send;w->message=message;w->reference=reference;}
void l5_wire_send(struct l5_wire*w,const struct l5_message*m,unsigned buses)
{
    /* The persisted term and log capacity fit 16 bits. These messages have
     * at most three nonzero numeric fields. Deliver each in one CAN frame. */
    if((m->type>=L5_PREVOTE&&m->type<=L5_VOTE_REPLY)||m->type==L5_APPEND_REPLY){
        unsigned k=m->type==L5_APPEND_REPLY?4:m->type-1;
        uint8_t d[8]={0};put16(d,m->term);put16(d+2,m->index);
        put16(d+4,m->type==L5_APPEND_REPLY?m->aux:m->previous_term);
        d[6]=m->ok;d[7]=w->transactions[m->type]++;
        w->send_frame(w->context,buses,L5_SHORT_BASE+k*64+m->source*16+m->destination*4+w->id,d);
        return;
    }
    if(m->type==L5_LEADER_PING||m->type==L5_LEADER_PONG){
        unsigned base=m->type==L5_LEADER_PING?L5_PING_BASE:L5_PONG_BASE;
        uint8_t d[8]={m->term&255,m->term>>8};put48(d+2,((uint64_t)m->aux<<32)|m->index);
        w->send_frame(w->context,buses,base+m->source*4+w->id,d);return;
    }
    if(m->type==L5_TIME_QUERY||(m->type==L5_TIME_REPLY&&!(m->ok&1))){
        uint8_t d[8]={m->term&255,(m->term>>8)&255,0,0,0,0,m->ok,0};put32(d+2,m->index);
        unsigned base=m->type==L5_TIME_QUERY?L5_QUERY_BASE:L5_PROOF_BASE;
        w->send_frame(w->context,buses,base+(m->source<<4)+(m->destination<<2)+w->id,d);return;
    }
    if(m->type==L5_REF_ACK){
        uint8_t d[8]={m->term&255,(m->term>>8)&255};put48(d+2,((uint64_t)m->aux<<32)|m->index);
        w->send_frame(w->context,buses,L5_ACK_BASE+(m->source<<4)+(m->destination<<2)+w->id,d);return;
    }
    if(m->type<1||m->type>11)return;
    uint8_t bytes[36]={m->type,m->source,m->destination,m->ok};
    put32(bytes+4,m->term);put32(bytes+8,m->index);put32(bytes+12,m->previous_term);
    put32(bytes+16,m->commit);put32(bytes+20,m->event);put32(bytes+24,m->aux);put32(bytes+28,crc(bytes,28));
    uint8_t transaction=w->transactions[m->type]++;
    for(unsigned fragment=0;fragment<6;fragment++){
        uint8_t d[8]={(m->destination<<6)|(m->source<<4)|fragment,transaction};
        memcpy(d+2,bytes+fragment*6,6);
        w->send_frame(w->context,buses,rpc_id(m->type,w->id),d);
    }
}
void l5_wire_ref(struct l5_wire*w,uint32_t term,uint8_t cycle,uint64_t global)
{uint8_t d[8]={cycle,term&255};put48(d+2,global);w->send_frame(w->context,3,L5_REF_BASE+w->id*4+w->id,d);}
void l5_wire_sample(struct l5_wire*w,unsigned destination,uint32_t nonce,uint64_t global)
{
    uint8_t d[8];put32(d,nonce);put32(d+4,(uint32_t)global);
    w->send_frame(w->context,3,L5_SAMPLE_BASE+(w->id<<4)+(destination<<2)+w->id,d);
}
static bool ref_new(struct l5_reference_seen*s,uint8_t cycle,uint8_t term,unsigned bus,uint64_t now)
{
    if(s->valid&&s->term==term&&s->cycle==cycle){s->seen|=1u<<bus;return false;}
    if(s->valid&&s->term==term&&(uint8_t)(cycle-s->cycle)>=128&&now-s->at<200000)return false;
    *s=(struct l5_reference_seen){.cycle=cycle,.term=term,.seen=1u<<bus,.at=now,.valid=true};return true;
}
bool l5_wire_receive(struct l5_wire*w,unsigned bus,uint16_t id,const uint8_t*d,unsigned dlc,uint64_t at,uint64_t now)
{
    if(!l5_wire_id(id))return false;
    if(bus>1||dlc!=8)return true;
    unsigned src=0;
    if(id>=L5_SHORT_BASE&&id<0x480)src=(id>>4)&3;
    else if(id>=L5_RPC_BASE||id>L5_TIME_REPLY_BASE)src=(d[0]>>4)&3;
    else if((id>=L5_REF_BASE&&id<=0x4f)||(id>=0x35&&id<=0x3f)||(id>=0x65&&id<=0x6f))src=(id>>2)&3;
    else src=(id>>4)&3;
    if(src&&src==(id&3)){w->direct_known[src][bus]=true;w->direct_at[src][bus]=at;}
    if(id>=L5_SHORT_BASE&&id<0x480){
        unsigned k=(id-L5_SHORT_BASE)/64,type=k==4?L5_APPEND_REPLY:k+1,dest=(id>>2)&3;
        if(!src||src==w->id||!(id&3))return true;
        struct l5_wire_slot*s=&w->slots[type-1][src][dest];
        if(s->active&&now-s->touched<200000){
            uint8_t distance=d[7]-s->transaction;
            if(!distance||distance>=128)return true;
        }
        s->active=true;s->transaction=d[7];s->touched=now;
        if(dest!=w->id)w->send_frame(w->context,1u<<(bus^1),(id&~3u)+w->id,d);
        if(!dest||dest==w->id){
            struct l5_message m={.type=type,.source=src,.destination=dest,.term=get16(d),.index=get16(d+2),.ok=d[6]};
            if(type==L5_APPEND_REPLY)m.aux=get16(d+4);else m.previous_term=get16(d+4);
            w->message(w->context,&m,at);
        }
        return true;
    }
    if((id>=0x35&&id<=0x3f)||(id>=0x65&&id<=0x6f)){
        bool pong=id>=0x65;unsigned base=pong?L5_PONG_BASE:L5_PING_BASE;
        unsigned source=(id-base)>>2,hop=id&3;
        if(!source||!hop||source==w->id)return true;
        uint32_t term=d[0]|((uint32_t)d[1]<<8);uint64_t stamp=get48(d+2);
        struct l5_reference_seen*s=&w->pulses[pong][source];
        if(hop==source){w->direct_known[source][bus]=true;w->direct_at[source][bus]=at;}
        if(s->valid&&(term<s->full_term||(term==s->full_term&&stamp<=s->stamp)))return true;
        *s=(struct l5_reference_seen){.valid=true,.full_term=term,.stamp=stamp};
        struct l5_message m={.type=pong?L5_LEADER_PONG:L5_LEADER_PING,.source=source,.term=term,.index=(uint32_t)stamp,.aux=(uint32_t)(stamp>>32)};
        w->message(w->context,&m,at);
        unsigned other=bus^1;
        if(!w->direct_known[source][other]||now-w->direct_at[source][other]>=40000)
            w->send_frame(w->context,1u<<other,base+source*4+w->id,d);
        return true;
    }
    if(id>=L5_QUERY_BASE&&id<L5_PROOF_BASE+64){
        bool query=id<L5_PROOF_BASE;unsigned base=query?L5_QUERY_BASE:L5_PROOF_BASE;
        unsigned source=((id-base)>>4)&3,dest=(id>>2)&3,hop=id&3;
        if(!source||!dest||!hop||source==w->id)return true;
        /* Compact freshness messages use the reserved nonce as the complete
         * transaction ID, with independent caches for requests and proofs. */
        struct l5_sample_seen*s=&w->compact[query?0:1][source][dest];
        uint32_t nonce=get32(d+2);
        if(s->valid&&nonce<=s->nonce)return true;
        s->valid=true;s->nonce=nonce;
        if(dest==w->id){
            struct l5_message m={.type=query?L5_TIME_QUERY:L5_TIME_REPLY,.source=source,.destination=dest,
                .term=(uint32_t)d[0]|((uint32_t)d[1]<<8),.index=nonce,.ok=d[6]};
            w->message(w->context,&m,at);
        }else w->send_frame(w->context,1u<<(bus^1),base+(source<<4)+(dest<<2)+w->id,d);
        return true;
    }
    if(id>=L5_SAMPLE_BASE&&id<L5_SAMPLE_BASE+64){
        unsigned source=((id-L5_SAMPLE_BASE)>>4)&3,dest=(id>>2)&3,hop=id&3;
        if(!source||!dest||!hop||source==w->id)return true;
        uint32_t nonce=get32(d);struct l5_sample_seen*s=&w->samples[source][dest];
        if(s->valid&&nonce<=s->nonce)return true;
        s->valid=true;s->nonce=nonce;
        if(dest==w->id){if(w->sample)w->sample(w->context,source,nonce,get32(d+4),at,w->delay_by_hop[hop]);}
        else{
            uint8_t copy[8];memcpy(copy,d,8);
            put32(copy+4,get32(d+4)+w->delay_by_hop[hop]+(uint32_t)(now>=at?now-at:0));
            w->send_frame(w->context,1u<<(bus^1),L5_SAMPLE_BASE+(source<<4)+(dest<<2)+w->id,copy);
        }
        return true;
    }
    if(id<0x100){
        bool ref=id<0x80;unsigned base=ref?L5_REF_BASE:L5_ACK_BASE;
        unsigned origin=ref?(id-base)>>2:((id-base)>>4)&3,hop=id&3;
        unsigned destination=ref?0:(id>>2)&3;
        if(origin<1||origin>3||hop<1||hop>3||origin==w->id)return true;
        struct l5_reference_seen*s=ref?&w->refs[origin]:&w->acks[origin];
        uint8_t cycle=d[0],term=d[1];
        if(ref){if(!ref_new(s,cycle,term,bus,now))return true;}
        else{
            uint16_t full=(uint16_t)d[0]|((uint16_t)d[1]<<8);uint64_t stamp=get48(d+2);
            if(destination<1||destination>3)return true;
            if(s->valid&&(full<s->full_term||(full==s->full_term&&stamp<=s->stamp)))return true;
            *s=(struct l5_reference_seen){.valid=true,.full_term=full,.stamp=stamp,.at=now,.seen=1u<<bus};
        }
        if(ref)w->reference(w->context,origin,d[1],cycle,get48(d+2),at,w->delay_by_hop[hop]);
        else {
            uint64_t stamp=get48(d+2);
            struct l5_message m={.type=L5_REF_ACK,.source=origin,.destination=destination,
                .term=(uint32_t)d[0]|((uint32_t)d[1]<<8),.aux=(uint32_t)(stamp>>32),.index=(uint32_t)stamp};
            w->message(w->context,&m,at);
        }
        unsigned other=bus^1u;
        /* ACKs also carry peer-liveness evidence for observers on the other
         * bus. A leader acting as the bridge must forward them too. */
        {
            uint8_t copy[8];memcpy(copy,d,8);
            if(ref)put48(copy+2,get48(copy+2)+w->delay_by_hop[hop]+(now>=at?now-at:0));
            uint16_t out_id=ref?base+origin*4+w->id:base+(origin<<4)+(destination<<2)+w->id;
            w->send_frame(w->context,1u<<other,out_id,copy);s->forwarded|=1u<<other;
        }
        return true;
    }
    unsigned type=id<L5_RPC_BASE?L5_TIME_REPLY:(id-L5_RPC_BASE)/16+1,hop=id&15,source=(d[0]>>4)&3,dest=d[0]>>6,fragment=d[0]&15;
    if(type>11||type==L5_REF_ACK||hop<1||hop>3||source<1||source>3||source==w->id||fragment>=6)return true;
    struct l5_wire_slot*s=&w->slots[type-1][source][dest];uint8_t transaction=d[1],bit=1u<<fragment;
    /* Reboot may restart the 8-bit fragment transaction at the same value as
     * an old complete/bad slot. Expire that slot too, not only a changed ID. */
    if(!s->active||s->transaction!=transaction||now-s->touched>=200000){
        if(s->active&&(uint8_t)(transaction-s->transaction)>=128&&now-s->touched<200000)return true;
        memset(s,0,sizeof(*s));s->active=true;s->transaction=transaction;s->touched=now;
    }
    if(s->bad)return true;
    bool seen=(s->seen[0]|s->seen[1])&bit;
    if(seen && memcmp(s->bytes+fragment*6,d+2,6)){s->bad=true;return true;}
    memcpy(s->bytes+fragment*6,d+2,6);s->seen[bus]|=bit;
    unsigned other=bus^1u;
    if(dest!=w->id && !(s->seen[other]&bit) && !(s->forwarded[other]&bit)){
        w->send_frame(w->context,1u<<other,rpc_id(type,w->id),d);s->forwarded[other]|=bit;
    }
    if(!s->complete && (s->seen[0]|s->seen[1])==63){
        s->complete=true;
        if(s->bytes[0]!=type||s->bytes[1]!=source||s->bytes[2]!=dest||get32(s->bytes+28)!=crc(s->bytes,28))return true;
        struct l5_message m={type,source,dest,s->bytes[3],get32(s->bytes+4),get32(s->bytes+8),
            get32(s->bytes+12),get32(s->bytes+16),get32(s->bytes+20),get32(s->bytes+24)};
        w->message(w->context,&m,at);
    }
    return true;
}
