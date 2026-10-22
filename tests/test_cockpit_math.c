#define main cockpit_program_main
#include "../pi/cockpit.c"
#undef main
#include <assert.h>
#include <math.h>
int main(void)
{
    assert(fabs(advance_deadline(1.0,1.0,.020)-1.020)<1e-9);
    assert(fabs(advance_deadline(1.0,1.051,.020)-1.060)<1e-9);
    assert(map_axis(0,-32767,32767,-1000,1000)==0);
    assert(map_axis(32767,0,32767,0,100)==100);
    assert(map_axis(-10,0,1000,0,100)==0);
    assert(map_axis(-30,0,1000,0,100)<0);
    state_t input={0};
    assert(!wheel_fresh(&input,1.0)); /* Startup still requires real input. */
    input.have_rx=true;input.last_rx=1.0;
    assert(wheel_fresh(&input,1.120)); /* Observed Wi-Fi delay must not fail. */
    assert(wheel_fresh(&input,1.250));
    assert(!wheel_fresh(&input,1.251)); /* Actual disconnect still expires. */
    state_t s={0};struct can_frame frame;
    assert(control_command(&s,"bad-input on") && s.bad_input);
    assert(control_command(&s,"bad-input off") && !s.bad_input);
    assert(!control_command(&s,"invalid") && !s.bad_input);
    char path[100];snprintf(path,sizeof(path),"/tmp/lab3-test-control-%ld.sock",(long)getpid());
    int control=control_open(path);assert(control>=0);
    struct stat metadata;assert(!stat(path,&metadata));assert((metadata.st_mode&0777)==0600);
    assert(control_open(path)<0);assert(!stat(path,&metadata)); /* Existing endpoint preserved. */
    int client=socket(AF_UNIX,SOCK_DGRAM,0);assert(client>=0);
    struct sockaddr_un address={.sun_family=AF_UNIX};strcpy(address.sun_path,path);
    assert(sendto(client,"bad-input on",12,0,(struct sockaddr *)&address,sizeof(address))==12);
    char command[32]={0};assert(recv(control,command,sizeof(command)-1,0)==12);
    assert(control_command(&s,command) && s.bad_input);
    assert(control_command(&s,"bad-input off"));
    close(client);close(control);assert(!unlink(path));
    s.steering=-1000;s.throttle=100;s.serial_seq=15;
    build_command_frame(&frame,&s,false);
    assert(frame.can_id==CAN_ID_CTRL && frame.can_dlc==8);
    assert(can_seq_of(frame.data)==15 && lab3_payload_valid(CAN_ID_CTRL,frame.data));
    s.bad_input=true;build_command_frame(&frame,&s,false);
    assert(!lab3_payload_valid(CAN_ID_CTRL,frame.data));
    state_t lights={0};
    lights.btn_synced=true;lights.btn_last[0]=lights.btn_last[1]=-1;
    lights.buttons=CTRL_BTN_LSB;blinker_buttons(&lights,1.0);
    assert(lights.blink_sel==BLINK_MODE_LEFT);
    blinker_buttons(&lights,1.1);assert(lights.blink_sel==BLINK_MODE_LEFT);
    lights.buttons=CTRL_BTN_RSB;blinker_buttons(&lights,1.2);
    assert(lights.blink_sel==BLINK_MODE_RIGHT);
    lights.steering=350;blinker_self_cancel(&lights);assert(lights.turn_crossed);
    lights.steering=100;blinker_self_cancel(&lights);assert(lights.blink_sel==BLINK_MODE_OFF);
    lights.blink_last_tx=-1;
    blinker_set_output(&lights,BLINK_MODE_HAZARD,2.0);
    assert(lights.blink_on && lights.blink_next_edge==2.25);
    blinker_sent(&lights,2.0);
    assert(blinker_due(&lights,2.25) && !lights.blink_on);
    build_blink_frame(&frame,&lights);
    assert(frame.can_id==CAN_ID_BLINK && frame.data[BLINK_MODE]==BLINK_MODE_HAZARD && frame.data[BLINK_PHASE]==0);
    puts("PASS: cockpit deadlines, calibration, CTRL validation, left/right switching, self-cancel and hazard phase");
}
