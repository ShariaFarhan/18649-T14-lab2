cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab2)

target_sources(app PRIVATE
    src/main.c
    ../logic/steering/servo.c
    ../logic/blinkers/blinkers.c
)

target_include_directories(app PRIVATE
    ../logic/steering
    ../logic/blinkers
)
