cmake_minimum_required(VERSION 3.28.0)

option(LAB2_WHEEL_STEERING "Real wheel to steering, motors disabled" OFF)
option(LAB2_WHEEL_RIGHT_DRIVE "Add right-wheel encoder speed control to wheel steering" OFF)
option(LAB2_WHEEL_DUAL_DRIVE "Two-encoder mean-speed control, steering and lights" OFF)
option(LAB2_ENCODER_ONLY "Standalone encoder readout without Pi or actuator PWM" OFF)
if(LAB2_ENCODER_ONLY)
    list(APPEND EXTRA_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/encoder_only.overlay")
endif()
if(LAB2_WHEEL_STEERING)
    if(NOT LAB2_WHEEL_RIGHT_DRIVE AND NOT LAB2_WHEEL_DUAL_DRIVE)
        list(APPEND EXTRA_DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/wheel_steering.overlay")
    endif()
endif()
if(LAB2_WHEEL_RIGHT_DRIVE AND NOT LAB2_WHEEL_STEERING)
    message(FATAL_ERROR "Right drive requires LAB2_WHEEL_STEERING")
endif()
if(LAB2_WHEEL_DUAL_DRIVE AND (LAB2_WHEEL_RIGHT_DRIVE OR NOT LAB2_WHEEL_STEERING))
    message(FATAL_ERROR "Select only DUAL drive with LAB2_WHEEL_STEERING")
endif()
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab2)

target_sources(app PRIVATE ../logic/instrumentation/test_points.c)
target_include_directories(app PRIVATE ../logic/instrumentation)

option(LAB2_UART_PWM_BENCH "Remote fixed-PWM voltage test" OFF)
if(LAB2_WHEEL_STEERING AND (LAB2_ENCODER_ONLY OR LAB2_UART_PWM_BENCH OR LAB2_UART_DIAGNOSTIC))
    message(FATAL_ERROR "Wheel steering is a separate profile")
endif()
if(LAB2_ENCODER_ONLY AND LAB2_UART_PWM_BENCH)
    message(FATAL_ERROR "Select either encoder-only or actuator bench")
endif()
if(LAB2_WHEEL_STEERING)
    if(LAB2_WHEEL_DUAL_DRIVE)
        target_sources(app PRIVATE src/wheel_dual_main.c)
    else()
        target_sources(app PRIVATE src/wheel_steering_main.c)
    endif()
    target_sources(app PRIVATE
        ../logic/steering/servo.c ../logic/steering/wheel_mapping.c
        ../logic/steering/wheel_buttons.c ../logic/blinkers/blinkers.c
        ../logic/comms/pi_link.c)
    target_include_directories(app PRIVATE ../logic/steering ../logic/comms ../logic/blinkers)
    target_compile_definitions(app PRIVATE LAB2_RAW_WHEEL_LINK=1
        SERVO_PULSE_LEFT_US=1000U SERVO_PULSE_RIGHT_US=2000U)
    if(LAB2_WHEEL_DUAL_DRIVE)
        # Polled every 20 ms: nominal disconnect response <=95 ms plus output execution.
        target_compile_definitions(app PRIVATE LINK_TIMEOUT_MS=75)
    else()
        target_compile_definitions(app PRIVATE LINK_TIMEOUT_MS=60)
    endif()
    if(LAB2_WHEEL_RIGHT_DRIVE)
        target_sources(app PRIVATE ../logic/drive/right_drive.c ../logic/drive/right_speed_model.c
            ../logic/drive/motor_control.c ../logic/drive/encoder.c ../logic/drive/encoder_math.c
            src/encoder_shell.c)
        target_include_directories(app PRIVATE ../logic/drive)
        target_compile_definitions(app PRIVATE LAB2_WHEEL_RIGHT_DRIVE=1 MOTOR_TIMING_WHEEL=RIGHT_MOTOR)
    endif()
    if(LAB2_WHEEL_DUAL_DRIVE)
        target_sources(app PRIVATE ../logic/drive/dual_drive.c ../logic/drive/dual_speed_pid.c ../logic/drive/right_speed_model.c
            ../logic/drive/motor_control.c ../logic/drive/encoder.c ../logic/drive/encoder_math.c
            ../logic/sensing/current_sense.c
            ../logic/system/system_state.c
            src/encoder_shell.c)
        target_include_directories(app PRIVATE ../logic/drive ../logic/sensing ../logic/system)
        # User observed negative right RPM while physically driving forward: invert feedback only.
        target_compile_definitions(app PRIVATE LAB2_WHEEL_DUAL_DRIVE=1 MOTOR_TIMING_WHEEL=LEFT_MOTOR
            ENCODER_RIGHT_DEFAULT_SIGN=-1)
    endif()
elseif(LAB2_ENCODER_ONLY)
    target_sources(app PRIVATE src/encoder_only_main.c src/encoder_shell.c
        ../logic/drive/encoder.c ../logic/drive/encoder_math.c)
    target_include_directories(app PRIVATE ../logic/drive)
elseif(LAB2_UART_PWM_BENCH)
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
    ../logic/sensing/current_sense.c
)

target_include_directories(app PRIVATE
    ../logic/steering
    ../logic/blinkers
    ../logic/drive
    ../logic/sensing
)

endif()

# Optional transport-only bench test; motor/servo commands from Pi are not applied.
option(LAB2_UART_DIAGNOSTIC "Enable Pi UART bench diagnostics" OFF)
if(LAB2_UART_DIAGNOSTIC AND (LAB2_UART_PWM_BENCH OR LAB2_ENCODER_ONLY OR LAB2_WHEEL_STEERING))
    message(FATAL_ERROR "Select only one UART bench profile")
endif()
if(LAB2_UART_DIAGNOSTIC)
    target_sources(app PRIVATE
        src/link_diagnostic.c
        ../logic/comms/pi_link.c
    )
    target_include_directories(app PRIVATE ../logic/comms)
    target_compile_definitions(app PRIVATE LAB2_UART_DIAGNOSTIC=1)
endif()
