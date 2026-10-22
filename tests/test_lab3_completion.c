#include <assert.h>
#include <stdio.h>
#include "lab3_supervision.h"
#include "system_state.h"
#include "../stm32/lab3_app/src/drive_controller.h"
int main(void)
{
    struct system_state s;
    system_state_init(&s,true);system_state_sync_button(&s,0);
    system_state_valid(&s);
    system_state_button(&s,1,100);assert(s.self_test);
    system_state_button(&s,2,110);assert(s.self_test); /* Bounce is not recovery. */
    assert(system_state_button(&s,3,300));assert(!s.self_test);
    system_state_button(&s,4,900);assert(s.self_test);
    system_state_button(&s,5,1600);assert(s.self_test);
    assert(system_state_button(&s,6,1800));
    assert(lab3_progress_fresh(1000,801));assert(!lab3_progress_fresh(1000,800));
    assert(lab3_progress_fresh(10,UINT32_MAX-10));
    struct lab3_drive_controller d={0};
    assert(!lab3_controller_command(&d,20,0,true,0,0));
    assert(d.output.duty>=40 && d.running);
    for (unsigned t=10;t<=5000;t+=10) {
        lab3_model_step(&d,10);
        assert(!lab3_controller_command(&d,20,0,true,t,10000));
        assert(d.output.duty<=100 && d.model_mrpm>=0);
    }
    assert(d.model_mrpm>0 && d.output.duty<40); /* Boost expires; PID regulates model. */
    assert(!lab3_controller_command(&d,100,1,true,5010,0));
    assert(d.output.mode==LAB3_DRIVE_BRAKE && d.output.duty==100 && !d.running);
    assert(!lab3_controller_command(&d,100,0,false,5020,0));
    assert(d.output.mode==LAB3_DRIVE_BRAKE);
    assert(!lab3_controller_command(&d,0,0,true,5030,0));
    assert(d.output.mode==LAB3_DRIVE_COAST && !d.output.duty);
    puts("PASS: button fail/recover/bounce, watchdog progress/wrap, PID model, boost expiry and brake priority");
}
