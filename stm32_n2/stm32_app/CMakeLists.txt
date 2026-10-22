cmake_minimum_required(VERSION 3.28.0)

if(NOT DEFINED DTC_OVERLAY_FILE)
    set(DTC_OVERLAY_FILE boards/nucleo_f401re_front.overlay)
endif()
if(NOT DEFINED CONF_FILE)
    set(CONF_FILE front_zone.conf)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab3_front_zone)

target_sources(app PRIVATE
    src/front_main.c
    ../logic/comms/can_comms.c
    ../logic/system/front_zone.c
    ../logic/blinkers/front_blinkers.c
    ../logic/steering/servo.c
)
target_include_directories(app PRIVATE
    ../logic/comms ../logic/system ../logic/blinkers ../logic/steering
)
