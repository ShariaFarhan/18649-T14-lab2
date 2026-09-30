cmake_minimum_required(VERSION 3.28.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab2)

target_sources(app PRIVATE
    src/main.c
    src/encoder_shell.c
    ../logic/steering/servo.c
    ../logic/blinkers/blinkers.c
    ../logic/drive/motor_control.c
    ../logic/drive/brake.c
    ../logic/drive/encoder.c
    ../logic/drive/encoder_math.c
)

target_include_directories(app PRIVATE
    ../logic/steering
    ../logic/blinkers
    ../logic/drive
)
