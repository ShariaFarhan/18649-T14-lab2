#ifndef LAB5_PI_AGENT_H
#define LAB5_PI_AGENT_H
#include "../lab5/node.h"
enum l5_pi_pin { L5_PI_PPS,L5_PI_LEADER,L5_PI_SYNC,L5_PI_REF_RX,L5_PI_HAZARD,L5_PI_RELAY,L5_PI_CAN_TX_A,L5_PI_CAN_TX_B };
int lab5_pi_start(const char *log,const char *can_a,const char *can_b,void *context,
                  void (*pin)(void *,enum l5_pi_pin,bool));
void lab5_pi_stop(void);
void lab5_pi_input(bool valid,bool self_test,bool error);
struct l5_snapshot lab5_pi_snapshot(void);
#endif
