#include <assert.h>
#include <stdio.h>
#include "link_watchdog.h"
int main(void)
{
    struct link_watchdog w;
    link_watchdog_init(&w,75);
    assert(!link_watchdog_poll(&w,10000)); /* Startup belongs to the system state. */
    link_watchdog_feed(&w,10000);
    for(unsigned age=0;age<75;age++)assert(!link_watchdog_poll(&w,10000+age));
    assert(link_watchdog_poll(&w,10075));
    assert(!link_watchdog_poll(&w,10100)); /* Report only once, not once per poll. */
    link_watchdog_feed(&w,10101);assert(!link_watchdog_poll(&w,10102));
    for(uint64_t t=10200;t<70200;t+=50){
        link_watchdog_feed(&w,t);
        for(unsigned age=0;age<50;age++)assert(!link_watchdog_poll(&w,t+age));
    }
    puts("PASS: healthy polling has no expiry, delayed-but-valid updates stay normal, real deadline reports once");
}
