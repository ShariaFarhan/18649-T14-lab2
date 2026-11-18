#ifndef L5_RAFT_H
#define L5_RAFT_H
#include "config.h"
#include "network.h"
enum l5_role { L5_FOLLOWER,L5_PRE_CANDIDATE,L5_CANDIDATE,L5_LEADER };
#define L5_LEADER_PING 12u
#define L5_LEADER_PONG 13u
enum l5_rpc { L5_PREVOTE=1,L5_PREVOTE_REPLY,L5_VOTE,L5_VOTE_REPLY,
              L5_APPEND,L5_APPEND_REPLY,L5_CLIENT,L5_CLIENT_REPLY,L5_REF_ACK,L5_TIME_QUERY,L5_TIME_REPLY };
struct l5_entry { uint32_t term,index,event; };
struct l5_meta { uint32_t term,commit,applied,client_seq;uint8_t voted_for; };
struct l5_message {
    uint8_t type,source,destination,ok;
    uint32_t term,index,previous_term,commit,event,aux;
};
struct l5_storage {
    void *context;
    int (*load)(void *,struct l5_meta *,uint32_t *last_index);
    int (*read)(void *,uint32_t index,struct l5_entry *);
    int (*append)(void *,const struct l5_entry *);
    int (*truncate)(void *,uint32_t keep);
    int (*save)(void *,const struct l5_meta *);
};
struct l5_callbacks {
    void *context;
    void (*send)(void *,const struct l5_message *);
    void (*reference)(void *,unsigned leader,uint32_t term,uint8_t cycle);
    void (*applied)(void *,const struct l5_entry *,bool replayed);
};
struct l5_agent {
    struct l5_config config;
    struct l5_storage storage;
    struct l5_callbacks callbacks;
    struct l5_meta meta;
    struct l5_network network,projected;
    uint32_t last_index,next[4],matched[4],last_request[4],request_index[4];
    uint32_t pending[16],pending_base[16],pending_count,replay_until,request_sequence;
    uint32_t graph_version[10];
    uint64_t election_due,last_contact,last_ack[4],next_ref,next_append,next_client,startup_until;
    uint32_t random;
    uint8_t id,leader,votes,cycle;
    enum l5_role role;
    bool leader_seen,ever_quorum,paused,fatal,joining;
    bool ack_seen[4];
    bool pristine_start,time_pending;
    uint32_t log_hash,hash_index;
};
int l5_init(struct l5_agent *,unsigned id,const struct l5_config *,
            const struct l5_storage *,const struct l5_callbacks *,uint64_t now_ms);
void l5_tick(struct l5_agent *,uint64_t now_ms);
void l5_receive(struct l5_agent *,const struct l5_message *,uint64_t now_ms);
bool l5_receive_reference(struct l5_agent *,unsigned leader,uint8_t term_low,uint8_t cycle,uint64_t now_ms,uint64_t reference_time);
bool l5_client_submit(struct l5_agent *,unsigned kind,unsigned node,unsigned bus,uint64_t now_ms);
bool l5_reserve_id(struct l5_agent *,uint32_t *id);
void l5_pause(struct l5_agent *,bool paused,uint64_t now_ms);
void l5_confirm_contact(struct l5_agent *,unsigned peer,bool peer_is_leader,uint64_t sent_ms);
const struct l5_network *l5_get_network(const struct l5_agent *);
#endif
