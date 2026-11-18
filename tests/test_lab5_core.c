#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "raft.h"
#include "time_sync.h"
#include "block_log.h"
struct memstore {struct l5_meta meta;struct l5_entry log[2048];unsigned count;};
static struct memstore stores[4];static struct l5_agent agents[4];
static bool online[4],paths[4][4];static uint64_t now;
static bool chaos;static uint32_t entropy=1;
static uint32_t rnd(void){entropy^=entropy<<13;entropy^=entropy>>17;entropy^=entropy<<5;return entropy;}
struct queued {struct l5_message m;};static struct queued queue[65536];static unsigned head,tail;
static int load(void *v,struct l5_meta *m,uint32_t *n){struct memstore*s=v;*m=s->meta;*n=s->count;return 0;}
static int readlog(void*v,uint32_t i,struct l5_entry*e){struct memstore*s=v;if(!i||i>s->count)return -1;*e=s->log[i-1];return 0;}
static int appendlog(void*v,const struct l5_entry*e){struct memstore*s=v;assert(e->index==s->count+1&&s->count<2048);s->log[s->count++]=*e;return 0;}
static int truncate_log(void*v,uint32_t n){struct memstore*s=v;assert(n>=s->meta.commit);s->count=n;return 0;}
static int save(void*v,const struct l5_meta*m){((struct memstore*)v)->meta=*m;return 0;}
static void send(void*v,const struct l5_message*m){(void)v;assert(tail-head<65535);queue[(tail++)%65536].m=*m;}
static void reference(void*v,unsigned leader,uint32_t term,uint8_t cycle)
{(void)v;for(unsigned p=1;p<=3;p++)if(p!=leader&&online[p]&&paths[leader][p])l5_receive_reference(&agents[p],leader,term&255,cycle,now,(uint32_t)(now*1000));}
static void drain(void)
{
    unsigned limit=0;
    while(head!=tail){assert(++limit<10000);
        if(chaos&&tail-head>1){unsigned slot=(head+rnd()%(tail-head))%65536;struct queued t=queue[head%65536];queue[head%65536]=queue[slot];queue[slot]=t;}
        struct l5_message m=queue[(head++)%65536].m;
        for(unsigned p=1;p<=3;p++)if(online[p]&&paths[m.source][p]&&p!=m.source&&(!m.destination||m.destination==p)){
            if(chaos&&rnd()%10==0)continue;
            l5_receive(&agents[p],&m,now);
            if(chaos&&rnd()%13==0)l5_receive(&agents[p],&m,now);
        }
    }
}
static void advance(unsigned ms)
{for(unsigned i=0;i<ms;i++){now++;for(unsigned p=1;p<=3;p++)if(online[p])l5_tick(&agents[p],now);drain();}}
static void boot(unsigned id,struct l5_config cfg)
{
    struct l5_storage s={&stores[id],load,readlog,appendlog,truncate_log,save};
    struct l5_callbacks cb={NULL,send,reference,NULL};
    online[id]=true;assert(!l5_init(&agents[id],id,&cfg,&s,&cb,now));
}
static void equal_logs(unsigned a,unsigned b)
{
    assert(agents[a].log_hash==agents[b].log_hash);
    assert(agents[a].meta.commit==agents[b].meta.commit);
    for(unsigned i=0;i<agents[a].meta.commit;i++)assert(!memcmp(&stores[a].log[i],&stores[b].log[i],sizeof(struct l5_entry)));
    assert(!memcmp(&agents[a].network,&agents[b].network,sizeof(struct l5_network)));
}
static void cluster_test(void)
{
    struct l5_config cfg=L5_DEFAULT_CONFIG;cfg.log_size=2048;
    for(unsigned i=1;i<=3;i++)for(unsigned j=1;j<=3;j++)paths[i][j]=true;
    boot(1,cfg);boot(2,cfg);boot(3,cfg);advance(300);
    assert(agents[1].role==L5_LEADER&&agents[1].meta.term==1);
    for(unsigned p=1;p<=3;p++){
        assert(l5_client_submit(&agents[p],L5_NODE_UP,p,0,now));
    }
    advance(500); /* Links are proposed after addNode has committed. */
    for(unsigned p=1;p<=3;p++){
        assert(l5_client_submit(&agents[p],L5_LINK_UP,p,0,now));
        assert(l5_client_submit(&agents[p],L5_LINK_UP,p,1,now));
    }
    assert(l5_client_submit(&agents[1],L5_COCKPIT_OK,1,0,now));advance(1500);
    equal_logs(1,2);equal_logs(1,3);assert(agents[1].network.nodes==14&&agents[1].network.cockpit_ok);
    uint32_t before=agents[1].last_index;
    struct l5_message stale={.type=L5_CLIENT,.source=2,.destination=1,.term=1,.commit=0,
        .event=l5_event(L5_NODE_DOWN,1,0,2,65000)};
    l5_receive(&agents[1],&stale,now);drain();
    assert(agents[1].last_index==before && (agents[1].network.nodes&2));
    /* State changes do not change Raft cluster membership or cause elections. */
    assert(l5_client_submit(&agents[1],L5_LINK_DOWN,1,1,now));advance(300);
    assert(l5_client_submit(&agents[3],L5_LINK_DOWN,3,0,now));advance(300);
    equal_logs(1,3);assert(agents[1].meta.term==1);
    assert(l5_route(&agents[1].network,1,3)==1); /* A through N2, then B. */
    online[1]=false;advance(1000);
    unsigned leader=agents[2].role==L5_LEADER?2:3;
    assert(agents[leader].role==L5_LEADER&&agents[leader].meta.term>1);
    assert((agents[2].role==L5_LEADER)+(agents[3].role==L5_LEADER)==1);
    assert(l5_client_submit(&agents[2],L5_EVENT,2,0,now));advance(300);
    uint32_t term=agents[leader].meta.term;boot(1,cfg);advance(500);
    equal_logs(1,leader);assert(agents[1].role==L5_FOLLOWER&&agents[leader].meta.term==term);
    /* Isolated nonleader prevotes but cannot raise the majority term. */
    unsigned isolated=leader==2?3:2;
    for(unsigned p=1;p<=3;p++)paths[isolated][p]=paths[p][isolated]=false;
    advance(500);assert(agents[leader].meta.term==term&&agents[isolated].meta.term==term);
    for(unsigned p=1;p<=3;p++)paths[isolated][p]=paths[p][isolated]=true;
    advance(300);equal_logs(1,2);equal_logs(1,3);
    puts("PASS Raft: predefined leader, majority commit, client graph, hop routes, election, rejoin, minority prevote");
}
static unsigned char flash[3*L5_BANK_BYTES];
static int fail_write=-1;
static int fr(void*c,uint32_t a,void*d,unsigned n){(void)c;if(a+n>sizeof(flash))return -1;memcpy(d,flash+a,n);return 0;}
static int fw(void*c,uint32_t a,const void*d,unsigned n){(void)c;const unsigned char*p=d;if(fail_write==0)return -1;if(fail_write>0)fail_write--;if(a+n>sizeof(flash))return -1;for(unsigned i=0;i<n;i++){if((flash[a+i]&p[i])!=p[i])return -1;flash[a+i]&=p[i];}return 0;}
static int fe(void*c,uint32_t a,unsigned n){(void)c;assert(a%L5_BANK_BYTES==0&&n==L5_BANK_BYTES);memset(flash+a,255,n);return 0;}
static void storage_test(void)
{
    memset(flash,255,sizeof(flash));struct l5_block_ops io={NULL,fr,fw,fe,NULL};struct l5_block_log log;
    assert(!l5_block_mount(&log,&io));struct l5_storage s=l5_block_storage(&log);
    struct l5_meta m={.term=1,.voted_for=1};assert(!s.save(s.context,&m));
    for(unsigned i=1;i<=30000;i++){struct l5_entry e={1,i,l5_event(L5_EVENT,2,0,2,i)};assert(!s.append(s.context,&e));}
    m.commit=20000;m.applied=19000;m.client_seq=30080;assert(!s.save(s.context,&m));
    struct l5_block_log restarted;assert(!l5_block_mount(&restarted,&io));s=l5_block_storage(&restarted);
    uint32_t last;assert(!s.load(s.context,&m,&last));assert(last==30000&&m.commit==20000&&m.applied==19000);
    assert(s.truncate(s.context,19999));assert(!s.truncate(s.context,20001));
    struct l5_entry e={2,20002,l5_event(L5_EVENT,3,0,3,1)};assert(!s.append(s.context,&e));
    for(unsigned i=0;i<600;i++){m.term=2+i;m.voted_for=1+i%3;assert(!s.save(s.context,&m));}
    assert(!l5_block_mount(&restarted,&io));s=l5_block_storage(&restarted);assert(!s.load(s.context,&m,&last));
    assert(m.term==601&&last==20002);assert(!s.read(s.context,20002,&e)&&e.term==2);
    e=(struct l5_entry){601,20003,l5_event(L5_EVENT,3,0,3,2)};fail_write=1;
    assert(s.append(s.context,&e));fail_write=-1;
    assert(!l5_block_mount(&restarted,&io));s=l5_block_storage(&restarted);
    assert(!s.load(s.context,&m,&last)&&last==20002);
    assert(!s.append(s.context,&e)); /* Partially written slot is reclaimed safely. */
    e.index=20004;e.event=l5_event(L5_EVENT,3,0,3,3);fail_write=2;
    assert(s.append(s.context,&e));fail_write=-1;
    assert(!l5_block_mount(&restarted,&io));s=l5_block_storage(&restarted);
    assert(!s.load(s.context,&m,&last)&&last==20003);assert(!s.append(s.context,&e));
    puts("PASS storage: 30000 entries, reboot, commit/applied markers, conflict truncate, metadata COW rollover");
}
static void time_test(void)
{
    assert(!l5_blink_phase(550000,1,570,250000));
    assert(l5_blink_phase(570000,1,570,250000)); /* Start20ms later, not next500ms boundary. */
    assert(!l5_blink_phase(1070000,1,570,250000));
    assert(l5_blink_phase((UINT64_C(0x100000000)+10)*1000,1,5,250000));
    struct l5_clock c;l5_clock_init(&c,0,0);
    assert(l5_clock_reference(&c,1000,1000000,350,1,1));
    assert(l5_clock_reference(&c,21002,1020000,350,1,1));
    uint64_t before=l5_global_gettime(&c,22000);l5_clock_master(&c,22000,2,2);
    assert(l5_global_gettime(&c,22000)==before);
    assert(l5_clock_reference(&c,42000,before+19000,350,3,3));
    assert(l5_global_gettime(&c,42000)>=before);
    uint64_t prev=l5_global_gettime(&c,42000);
    for(unsigned i=42001;i<50000;i++){uint64_t t=l5_global_gettime(&c,i);assert(t>=prev);prev=t;}
    puts("PASS time: delay correction, drift estimate, continuous leader handover and monotonic slew");
}
static void random_test(void)
{
    bool pi_won=false,stm_won=false;
    for(unsigned seed=1;seed<=12;seed++){
        memset(stores,0,sizeof(stores));memset(agents,0,sizeof(agents));head=tail=0;now=0;
        struct l5_config cfg=L5_DEFAULT_CONFIG;cfg.default_leader=0;cfg.seed=seed;cfg.log_size=2048;
        for(unsigned p=1;p<=3;p++)boot(p,cfg);
        advance(800);unsigned count=0,leader=0;
        for(unsigned p=1;p<=3;p++)if(agents[p].role==L5_LEADER){count++;leader=p;}
        assert(count==1);pi_won|=leader==1;stm_won|=leader!=1;
    }
    assert(pi_won&&stm_won);puts("PASS random mode: deterministic seeds cover Pi/STM leaders, one elected leader");
}
static void crash_and_loss_test(void)
{
    for(unsigned seed=1;seed<=8;seed++){
        memset(stores,0,sizeof(stores));memset(agents,0,sizeof(agents));head=tail=0;now=0;entropy=seed;chaos=true;
        struct l5_config cfg=L5_DEFAULT_CONFIG;cfg.default_leader=0;cfg.seed=seed;cfg.log_size=2048;
        for(unsigned p=1;p<=3;p++)boot(p,cfg);
        for(unsigned step=0;step<100;step++){
            unsigned p=1+rnd()%3;
            if(step%9==0){online[p]=false;advance(100);boot(p,cfg);}
            assert(l5_client_submit(&agents[p],L5_EVENT,p,0,now));advance(50);
            for(unsigned a=1;a<=3;a++)for(unsigned b=a+1;b<=3;b++){
                assert(!(agents[a].role==L5_LEADER&&agents[b].role==L5_LEADER&&agents[a].meta.term==agents[b].meta.term));
                uint32_t n=agents[a].meta.commit<agents[b].meta.commit?agents[a].meta.commit:agents[b].meta.commit;
                for(unsigned i=0;i<n;i++)assert(!memcmp(&stores[a].log[i],&stores[b].log[i],sizeof(struct l5_entry)));
            }
        }
        chaos=false;advance(3000);equal_logs(1,2);equal_logs(1,3);
    }
    puts("PASS Raft fault schedules: loss, duplicate/reordered RPCs, repeated crash/restart, committed-prefix agreement");
}
int main(void){cluster_test();storage_test();time_test();random_test();crash_and_loss_test();}
