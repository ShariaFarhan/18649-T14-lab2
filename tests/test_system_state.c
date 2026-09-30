#include <assert.h>
#include "system_state.h"

int main(void)
{
    struct system_state s;
    system_state_init(&s, true); assert(system_state_reason(&s)==SYS_STARTUP);
    system_state_sync_button(&s, 0);
    system_state_link_lost(&s); assert(system_state_reason(&s)==SYS_STARTUP);
    system_state_valid(&s); assert(system_state_reason(&s)==SYS_OK);
    assert(!system_state_button(&s,1,100)); assert(system_state_reason(&s)==SYS_SELF_TEST);
    system_state_valid(&s); assert(system_state_reason(&s)==SYS_SELF_TEST);
    assert(!system_state_button(&s,1,150)); /* Repeated packets are not clicks. */
    assert(!system_state_button(&s,2,149)); /* Bounce ignored at 49 ms. */
    assert(system_state_reason(&s)==SYS_SELF_TEST);
    assert(system_state_button(&s,3,600)); /* Exactly 500 ms is accepted. */
    assert(system_state_reason(&s)==SYS_OK);
    assert(!system_state_button(&s,4,1200));
    assert(!system_state_button(&s,5,1701)); /* 501 ms is not a double click. */
    assert(system_state_reason(&s)==SYS_SELF_TEST);
    assert(system_state_button(&s,6,1751)); /* Exactly 50 ms is accepted. */
    system_state_link_lost(&s); assert(system_state_reason(&s)==SYS_LINK_LOST);
    system_state_valid(&s); assert(system_state_reason(&s)==SYS_OK);
    system_state_bad_input(&s); assert(system_state_reason(&s)==SYS_BAD_INPUT);
    system_state_valid(&s); assert(system_state_reason(&s)==SYS_OK);
    assert(!system_state_button(&s,7,2400));
    system_state_link_lost(&s);
    system_state_valid(&s);
    assert(!system_state_button(&s,8,2500)); /* Cannot recover across disconnect. */
    assert(system_state_button(&s,9,2600));
    system_state_output_failed(&s);
    system_state_valid(&s); assert(system_state_reason(&s)==SYS_OUTPUT_FAULT);
    assert(!system_state_button(&s,10,3300));
    assert(system_state_button(&s,11,3400));
    assert(system_state_reason(&s)==SYS_OUTPUT_FAULT); /* Gesture alone cannot clear it. */
    system_state_reinitialized(&s,false); assert(system_state_reason(&s)==SYS_OUTPUT_FAULT);
    system_state_reinitialized(&s,true); assert(system_state_reason(&s)==SYS_OK);
    system_state_sync_button(&s,31);
    assert(!system_state_button(&s,0,4000)); /* Counter wraps. */
    system_state_sync_button(&s,0); assert(system_state_reason(&s)==SYS_SELF_TEST);
    assert(!system_state_button(&s,1,4200));
    assert(system_state_button(&s,2,4300));
    assert(!system_state_button(&s,5,5000)); /* Lost events conservatively fail. */
    assert(system_state_reason(&s)==SYS_SELF_TEST);
    system_state_init(&s,true); system_state_valid(&s);
    assert(!system_state_button(&s,17,100)); /* Old count after MCU reboot is a baseline. */
    assert(system_state_reason(&s)==SYS_OK);
    assert(!system_state_button(&s,18,200));
    assert(system_state_reason(&s)==SYS_SELF_TEST);
    system_state_init(&s,false); system_state_valid(&s);
    assert(system_state_reason(&s)==SYS_OUTPUT_FAULT);
    return 0;
}
