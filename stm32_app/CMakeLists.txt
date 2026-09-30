cmake_minimum_required(VERSION 3.28.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab2)

option(LAB2_UART_PWM_BENCH "Remote fixed-PWM voltage test" OFF)
if(LAB2_UART_PWM_BENCH)
    target_sources(app PRIVATE
        src/uart_pwm_bench.c
        ../logic/system/system_state.c
        src/encoder_shell.c
        ../logic/steering/servo.c
        ../logic/blinkers/blinkers.c
        ../logic/drive/encoder.c
        ../logic/drive/encoder_math.c
        ../logic/drive/motor_control.c
        ../logic/drive/brake.c
        ../logic/comms/pi_link.c
        ../logic/comms/link_parser.c
    )
    target_include_directories(app PRIVATE ../logic/drive ../logic/comms ../logic/steering ../logic/blinkers ../logic/system)
    target_compile_definitions(app PRIVATE LINK_TIMEOUT_MS=60)
else()
target_sources(app PRIVATE
    src/main.c
    src/encoder_shell.c
    src/speed_shell.c
    ../logic/steering/servo.c
    ../logic/blinkers/blinkers.c
    ../logic/drive/motor_control.c
    ../logic/drive/brake.c
    ../logic/drive/encoder.c
    ../logic/drive/encoder_math.c
    ../logic/drive/speed_pi.c
    ../logic/drive/speed_control.c
)

target_include_directories(app PRIVATE
    ../logic/steering
    ../logic/blinkers
    ../logic/drive
)

endif()

# Optional transport-only bench test; motor/servo commands from Pi are not applied.
option(LAB2_UART_DIAGNOSTIC "Enable Pi UART bench diagnostics" OFF)
if(LAB2_UART_DIAGNOSTIC AND LAB2_UART_PWM_BENCH)
    message(FATAL_ERROR "Select only one UART bench profile")
endif()
if(LAB2_UART_DIAGNOSTIC)
    target_sources(app PRIVATE
        src/link_diagnostic.c
        ../logic/comms/pi_link.c
        ../logic/comms/link_parser.c
    )
    target_include_directories(app PRIVATE ../logic/comms)
    target_compile_definitions(app PRIVATE LAB2_UART_DIAGNOSTIC=1)
endif()
