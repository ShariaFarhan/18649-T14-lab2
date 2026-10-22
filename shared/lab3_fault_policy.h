#ifndef LAB3_FAULT_POLICY_H
#define LAB3_FAULT_POLICY_H
#include <stdbool.h>
#include "lab3_can_catalog.h"
static inline bool lab3_follow_master(bool fresh,bool master_live,bool normal,uint8_t mode)
{ return fresh && master_live && (normal || mode==BLINK_MODE_HAZARD); }
static inline bool lab3_tx_allowed(bool failed,unsigned own,uint16_t id,const uint8_t *data)
{
    return !failed || (id==CAN_ID_FAULT && data[FAULT_ZONE]==own &&
                      data[FAULT_REASON]==FAULT_R_SELF_TEST);
}
#endif
