#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/shell/shell.h>
#include "speed_control.h"
static int result(const struct shell *sh,int rc){
    if(rc)shell_error(sh,"Drive command failed: %d; check drive status",rc);
    return rc;
}
static int arm(const struct shell *sh,size_t argc,char **argv){
    ARG_UNUSED(argc);ARG_UNUSED(argv);
    return result(sh,speed_control_arm());
}
static int disarm(const struct shell *sh,size_t argc,char **argv){
    ARG_UNUSED(argc);ARG_UNUSED(argv);
    return result(sh,speed_control_disarm());
}
static int throttle(const struct shell *sh,size_t argc,char **argv){
    ARG_UNUSED(argc);
    char *end;errno=0;long p=strtol(argv[1],&end,10);
    if(errno || !*argv[1] || *end || p<0 || p>100)return -EINVAL;
    struct speed_status s;int rc=speed_control_get(&s);if(rc)return result(sh,rc);
    return result(sh,speed_control_set_input((uint16_t)p*10,s.brake));
}
static int brake(const struct shell *sh,size_t argc,char **argv){
    ARG_UNUSED(argc);
    if(strcmp(argv[1],"on") && strcmp(argv[1],"off"))return -EINVAL;
    return result(sh,speed_control_set_brake(!strcmp(argv[1],"on")));
}
static int status(const struct shell *sh,size_t argc,char **argv){
    ARG_UNUSED(argc);ARG_UNUSED(argv);
    struct speed_status s;int rc=speed_control_get(&s);if(rc)return result(sh,rc);
    shell_print(sh,"armed=%d brake=%d fault=%d throttle_permille=%u target_mrpm=%d feedback_mrpm=%d drive_pwm=%u%%",
        s.armed,s.brake,s.fault,s.throttle_permille,s.target_rpm_milli,s.feedback_rpm_milli,s.duty_percent);
    return 0;
}
SHELL_STATIC_SUBCMD_SET_CREATE(drive_commands,
    SHELL_CMD_ARG(arm,NULL,"Enable closed loop at zero throttle; verify wiring/signs first",arm,1,0),
    SHELL_CMD_ARG(disarm,NULL,"Brake and release closed-loop ownership",disarm,1,0),
    SHELL_CMD_ARG(throttle,NULL,"Set throttle percent 0..100",throttle,2,0),
    SHELL_CMD_ARG(brake,NULL,"Brake on/off; overrides throttle",brake,2,0),
    SHELL_CMD_ARG(status,NULL,"Show target/feedback in milli-RPM and drive PWM",status,1,0),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(drive,&drive_commands,"Mean-wheel-speed PI control",NULL);
