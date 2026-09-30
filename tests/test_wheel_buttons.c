#include <assert.h>
#include "wheel_buttons.h"
int main(void)
{
    struct wheel_buttons s;
    wheel_buttons_reset(&s);
    assert(wheel_buttons_pressed(&s,WHEEL_LSB_MASK,0)==0); /* Held at startup. */
    assert(wheel_buttons_pressed(&s,0,10)==0);
    assert(wheel_buttons_pressed(&s,WHEEL_LSB_MASK,20)==WHEEL_LSB_MASK);
    assert(wheel_buttons_pressed(&s,WHEEL_LSB_MASK,40)==0); /* Held, no retrigger. */
    assert(wheel_buttons_pressed(&s,0,50)==0);
    assert(wheel_buttons_pressed(&s,WHEEL_LSB_MASK,69)==0); /* 49 ms bounce. */
    assert(wheel_buttons_pressed(&s,0,70)==0);
    assert(wheel_buttons_pressed(&s,WHEEL_LSB_MASK,71)==WHEEL_LSB_MASK);
    assert(wheel_buttons_pressed(&s,WHEEL_RSB_MASK,80)==WHEEL_RSB_MASK);
    assert(wheel_buttons_pressed(&s,WHEEL_LSB_MASK|WHEEL_RSB_MASK,140)==0);
    assert(wheel_buttons_pressed(&s,WHEEL_LSB_MASK,150)==0); /* Releasing is not pressing. */
    wheel_buttons_reset(&s); /* Link recovery cannot replay a held press. */
    assert(wheel_buttons_pressed(&s,WHEEL_RSB_MASK,200)==0);
    assert(wheel_buttons_pressed(&s,0,210)==0);
    assert(wheel_buttons_pressed(&s,WHEEL_RSB_MASK,220)==WHEEL_RSB_MASK);
    return 0;
}
