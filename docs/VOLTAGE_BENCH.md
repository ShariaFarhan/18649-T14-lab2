# Integrated STM32 actuator bench

Open http://127.0.0.1:8765 on the Windows computer. The PC sends test commands to Pi over UDP/8001; Pi forwards them over GPIO UART at 115200 8N1. One NUCLEO-F401RE controls both motors, steering and all four lights.

## What is integrated

- Motor forward PWM, coast and brake priority. Both motors use the same duty.
- Steering PWM on PA8, initially disabled; explicit enable from the page.
- Front/rear left and right blinkers, hazards, steering-return auto-cancel.
- Dual encoder sampling and COM9 `encoder status / zero / sign` commands (3960 CPR remains an assumption).
- UART validation, feedback and timeout handling. On link loss: motor brake, hazards, and center an already-enabled servo.

Motor control is still fixed PWM for voltage measurements. Encoder PI remains in the normal project sources but is not enabled in this bench profile. Current sensing and real wheel input calibration are not implemented by this bench profile. The unified startup/fault/recovery state machine is implemented and the page emulates the future wheel self-test button. No unconnected current sensor values are presented as measurements.

## Test controls

- Throttle 0–100 maps directly to PWM. Brake 0 releases braking; any positive brake value commands full brake.
- Steering -100 / 0 / +100 maps to 1000 / 1500 / 2000 us, with a 20000 us period. Mechanical endpoints must be calibrated before driving a connected linkage.
- Left/right lights blink at 1 Hz; hazards at 2 Hz, nominal 50% duty.
- Auto-cancel arms past the selected side's 35% steering threshold and cancels on return to the 10% center boundary.
- Every light-button press has an event number. Repeated UART state packets cannot restart blink phase or re-enable an auto-cancelled turn signal.
- Stop all tests: brakes motors, disables servo PWM, and turns lights off while the link remains healthy.

## Where to measure

Black probe to GND. The readings below are approximate DC averages; a voltmeter cannot verify frequency, pulse width, response deadlines or actual steering angle.

| Function | STM32 pin | Expected reading |
|---|---|---|
| Left motor PWM | PA6, CN10-13 | 50% throttle / no brake: about 1.65 V |
| Right motor PWM | PA7, CN10-15 | Same as left |
| Motor brake | PA6/PA7, PB12–PB15 | All about 3.3 V |
| Motor coast | Same six pins | All about 0 V |
| Steering | PA8, CN10-23 | Left 50% about 0.21 V; center 0.25 V; right 50% 0.29 V |
| Front left | PC6, CN10-4 | Alternates 0 / 3.3 V with rear left |
| Front right | PC7, CN10-19 | Alternates 0 / 3.3 V with rear right |
| Rear left | PC8, CN10-2 | Paired with front left |
| Rear right | PC9, CN10-1 | Paired with front right |

## Build and services

- Build STM32 with `-DLAB2_UART_PWM_BENCH=ON -DEXTRA_CONF_FILE=uart_diagnostic.conf`.
- Pi: `python3 voltage_bench.py pi --config control.json`.
- PC: `python voltage_bench.py pc --config control.json`.
- Keep `actuator_bench.html` beside `voltage_bench.py` on both hosts.
- Local private config: ignored `build-uart-bench/control.json`; Pi: `/home/henry/lab2-voltage-bench/control.json`. Do not commit its key.
- Background helpers are running for this session, not installed as boot services.
- Pi GPIO UART is `/dev/ttyAMA0`. Repeat `sudo dtoverlay uart0-pi5` after Pi reboot until boot configuration is made persistent.
- COM9 / 115200: `bench status` reports PWM registers, pulse width and light pins; `encoder status` reports counts and RPM.

## Test protocol

This is separate from the Logitech UDP/8000 and production serial protocol. PC packets are authenticated JSON on UDP/8001, nominally every 20 ms; the Pi sends each accepted update immediately and repeats at 20 ms. A running helper alone is not a valid input source: explicit application of a command or a self-test action enables the test source.

The 9-byte UART command retains its layout: int16 steering, int16 throttle, int16 brake, uint16 buttons, uint8 flags (little-endian). The first three fields are percentages for this test. Buttons bits 0–1 select off/left/right/hazard, bit 2 enables the servo, and bits 8–15 are a light-button event counter. Buttons bits 3–7 carry a modulo-32 self-test press counter. Flags bit 7 identifies the bench format, bit 0 marks stale input, and bit 1 synchronizes the press counter without clearing a latched self-test fault. Flags bits 2–6 are rejected.

The 8-byte heartbeat uses byte 0 for state and byte 1 marker **0xB3**. Bytes 2–3 contain motor PWM percent in bits 0–7 and acknowledged self-test counter in bits 8–12. Bytes 4–5 contain brake bit 0, light mode bits 1–2, servo-active bit 3, turn-crossed bit 4, self-test-latched bit 5, actual PC6–PC9 outputs in bits 8–11, and fault reason in bits 12–15. Bytes 6–7 contain servo pulse width in microseconds. These words are not current readings.

Watchdogs: PC page heartbeat remains 600 ms; Pi PC-packet age remains 120 ms and STM32-heartbeat age remains 150 ms. This STM32 profile uses a 60 ms valid-command timeout, checked by the 20 ms heartbeat task, leaving nominal margin toward the stricter 100 ms checkoff requirement. These are configured budgets, not a scope-verified worst-case bound. The separate transport-only diagnostic profile keeps its original 150 ms setting.

## Unified state and recovery

| Reason | Enter on | Leave on |
|---|---|---|
| STARTUP | Boot, before valid control source | All required driver initializations succeed and a fresh, valid command arrives |
| NORMAL | No remaining fault | Any fault condition |
| LINK_LOST | UART timeout or upstream stale flag | Fresh, valid command; other latched faults must also clear |
| BAD_INPUT | CRC/shape rejection or out-of-range command | Fresh, valid command; other latched faults must also clear |
| SELF_TEST | First debounced self-test press | Second press 50–500 ms later, with healthy input/link |
| OUTPUT_FAULT | Driver initialization/output failure | Double-press requests reinitialization; it must succeed |

Every non-normal state brakes the motors, selects hazards, and centers an already-active servo. Startup checks the servo PWM device with output disabled. Driver readiness does not prove sensors or actuators are physically wired.

Self-test failure survives ordinary commands, input-source reconnects and PC counter synchronization. A double-press cannot span a communication loss. MCU reboot treats the first button counter as a baseline, not a new press. A power cycle does not persist the previous simulated fault in nonvolatile memory: it returns to startup validation.

Command/timeout callbacks are serialized to prevent an old timeout from overriding a newly accepted command. Duplicate/old UART frames do not refresh the live-link watchdog. Heartbeat state and output fields are captured under one lock. Console printing uses a snapshot and releases the output lock before logging.

Web controls: self-test button, out-of-range injection (throttle 101), and send-valid-input. `bench output_fault` simulates an output fault to exercise the retry path; it does not claim a physical driver has failed. `bench status` includes fault cause and the last UART timeout age.

Host state tests: `gcc -Ilogic/system tests/test_system_state.c logic/system/system_state.c -o state_test` then run it. Live tests are deliberately gated by `python tests/test_state_live.py --bare-board` and must only run with actuators disconnected.

## Verification on 2026-09-30

No motors or servo connected; board powered by USB, Pi connected through UART.

- Left 50%: TIM1 period 20000, compare 1250. Center: 1500. Right 50%: 1750.
- Motor PWM remains 2100/4200 on both channels at 50% while steering and hazards run.
- Left blinkers: only PC6/PC8 switch; sampled full periods 0.984, 0.985 and 1.000 s.
- Right blinkers: only PC7/PC9 switch.
- Hazards: all four switch; sampled full periods 0.515, 0.532 and 0.515 s.
- These period samples include network/polling uncertainty; use a scope for timing acceptance.
- Left turn, steering -50, then center: lights cancel and remain off under repeated command updates.
- Pausing Pi transmission: motor brake, 1500 us servo center and hazard mode observed through COM9. Resuming the relay restores valid command handling.
- Host tests exercise actual blinker C code: modes, nominal periods, paired outputs, auto-cancel and stale queued-work handling.
- Encoder interface works; disconnected encoder zeros are not physical sensor validation.

### Unified-state verification on connected board

- Cold start with Pi relay paused: STARTUP, motor brake, hazards, servo off.
- Fresh applied command: NORMAL.
- Single self-test press: SELF_TEST, brake, hazards, active servo centered. Continued 75–80% throttle did not unlock it.
- A new pair of presses 150 ms apart restored NORMAL; immediate repeated HTTP clicks did not bypass debounce.
- Throttle 101 injection: BAD_INPUT and safe outputs. A fresh valid command restored NORMAL.
- `bench output_fault`: OUTPUT_FAULT survived valid commands. Double-press reinitialized drivers successfully before recovery. This was a software fault injection, not a real hardware failure.
- Pausing UART transmission: LINK_LOST, brake/center/hazards. One observed watchdog age was 69 ms. This records software detection only; it does not prove the GPIO response deadline under all loads.
- Resuming transmission restored the requested 30% PWM and 1250 us steering.
- Old button counter after MCU reboot was baselined without triggering a new self-test press.
- Native state tests cover 49/50 ms debounce, 500/501 ms pair timing, modulo-32 counter wrap, missing events, failed reinitialization, and faults surviving reconnect/sync.
