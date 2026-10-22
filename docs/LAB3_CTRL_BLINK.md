# Lab 3 CTRL, steering, drive test outputs and split blinkers

This file records earlier deployed stages. The unflashed completion candidate
is described in **LAB3_COMPLETION.md**, which supersedes the pending-feature
lists and the former open-loop DRIVE behavior below.

Based on remote `7edeab6`. Reuses `pi/cockpit.c`, `pi/lab3_can_catalog.h`,
`stm32/logic/blinkers` and `stm32/logic/system/system_state`.
Local changes are not pushed.
The new Front steering image was flashed to N2 (D:/, ST-LINK COM10) on 2026-10-22.
Image: `build-checks/firmware/front-steering.bin` (ignored build artifact).
Both Front and Rear builds pass. The steering mapping regression covers full
left/center/right, intermediate positions, symmetric rounding, invalid range
and error-state centering. Post-flash serial telemetry reports NORMAL,
`steer=-30`, `SRV pulse_us=1485`, `output_fault=0`, and zero CAN TX errors.
These are software observations; physical SRV pulse/latency scope measurements
remain pending.

## LED wiring

| Board | Left turn lamp | Right turn lamp | CAN A fault | CAN B fault |
|---|---|---|---|---|
| Front / N2 / COM10 | FL: D14 (PB9) | FR: D15 (PB8) | D3 | D5 |
| Rear / N3 / COM9 | RL: D14 (PB9) | RR: D15 (PB8) | D3 | D5 |

Each output -> its own 330 ohm resistor -> LED anode; cathode -> GND.
I2C1 is disabled so D14/D15 are available. Existing CAN wiring is unchanged.

Front SRV: **D7 (PA8 / TIM1_CH1)**. Scope D7 against board GND for Lab3.
An optional real servo uses D7 for signal, its rated independent supply for
power, and common ground. D7 is a signal output, not the servo power supply.
Rear uses PA8 for a separate 20 kHz drive PWM; it is not a servo output.
Do not copy the Lab2 motor/encoder overlay:
its PA6/PA7 PWM and PB6 encoder assignments conflict with the Lab3 CAN SPI/CS.

## Implemented in this stage

- CTRL 0x010: 8-byte payload, CRC-8/range/flag validation, raw-to-normalized
  mapping reused from the cockpit, application delivery once across A/B.
- STM32 records steering, throttle, brake, buttons and flags. A fresh valid CTRL
  and healthy peer heartbeats allow NORMAL.
- Front reuses the remote `stm32/logic/steering/servo.c` and `servo.h` unchanged.
  Its event-driven STEER thread (priority 5, 1024-byte stack) is notified after
  accepted CTRL delivery or a normal/error transition. Latest target state is
  mutex-protected; a binary semaphore coalesces pending updates. PWM writes run
  outside the mutex. The priority-6 dispatcher retains ownership of system state;
  an atomic output-fault latch reports worker failures back to it.
- The CAN steering range -1000..1000 is rounded to the driver's -100..100 range.
  D7 emits 50 Hz: -1000 -> 1000 us, 0 -> 1500 us, +1000 -> 2000 us.
  Startup and every error state command 1500 us; valid input restores tracking
  once system health returns to NORMAL. A PWM driver failure is latched as
  OUTPUT_FAULT and reported through FAULT, with a best-effort center command.
  Repeated A/B copies do not produce a second steering command.
- BLINK 0x200: cockpit remains master. Both STM32s apply the received mode and
  phase directly using the existing GPIO driver; there is no independent normal
  turn-signal timer on each zone.
- Left/right selection, same-button cancellation and 35%/10% steering
  self-cancel remain in the existing cockpit implementation.
- Invalid CTRL produces BAD_INPUT and FAULT; valid input can clear BAD_INPUT.
  Link loss or missing input yields an error state and hazard lamps. Fresh
  master hazard phases are used when available; otherwise local 2 Hz fallback.
- Per-bus heartbeat progress is independent of global application delivery.
  A bus that lags by one sequence stays alive without reapplying old commands.
  Fixed replay cannot become NEW merely because 100 ms passed. After silence,
  backward-sequence resync needs two advancing observations. Four bits still
  cannot remove ambiguity from arbitrarily delayed/replayed traffic.

Not yet implemented/validated: the 10 ms PID task/feedback integration,
additional per-bus timing markers, physical relay validation, FORCE
producer, STM32 hardware self-test, full template task/RTA compliance, and
scope proof of steering timing or front/rear <=1 ms synchronization. STEER's
local 1 ms and end-to-end 50 ms deadlines are targets, not measured results.

N2's independent RELAY thread and Front D6 marker are implemented and the
relay image has been flashed. See `LAB3_RELAY.md` for routing, loop prevention,
software verification, and the complete crossed-isolation bench sequence.

## Rear DRIVE scope stage

The unchanged remote `motor_control.c/.h` and `instrumentation/test_points.c/.h`
are compiled into N3. PA6/PA7 remain SPI, so the PWM pins move to D6/D7.
Rear and Front builds pass; the new Rear image is saved as
`build-checks/firmware/rear-drive.bin` and was flashed to N3 (G:/, COM9) on
2026-10-22. Post-flash logs show NORMAL, zero CAN errors, duty 0 with both
pedals released, no output fault, and CTRL_RX/PWM_SET counters advancing
together (602/602 in the captured sample). Physical PWM/latency measurements
remain pending. No real motor or encoder is needed for
this scope stage, and this stage is not closed-loop RPM control.

| Rear signal | Board pin | Function |
|---|---|---|
| Left PWM / PWM_OUT | D6 (PB10 / TIM2_CH3) | 20 kHz, 0-100% |
| Left IN1 / DIR_A | A0 (PA0) | Original driver's first direction input |
| Left IN2 / DIR_B | A1 (PA1) | Original driver's second direction input |
| Right PWM | D7 (PA8 / TIM1_CH1) | 20 kHz, same commanded duty |
| Right IN1 / IN2 | A2 (PA4) / A3 (PB0) | Original driver's right direction inputs |
| CTRL_RX | A4 (PC1) | Toggles once per validated, deduplicated CTRL |
| PWM_SET | D8 (PA9) | Toggles after final successful left PWM write |

These pins do not overlap CAN, the four lamps, the degraded LEDs, or the
USART2 debug console. All scope ground clips connect to board GND.
CTRL_RX is a logical delivery marker, not a substitute for separate
CAN_RX_A/CAN_RX_B markers before duplicate suppression.

DRIVE runs on a semaphore at priority 4 with a 1024-byte stack. The dispatcher
publishes the latest command through a mutex; hardware writes happen outside
that mutex. New accepted CTRL or normal/error transitions wake DRIVE; duplicate
bus copies do not. Scope outputs follow validated normal-state CTRL even though
the staged Pi does not set the Lab2 physical motor-enable bit. The HB armed bit
therefore stays clear; HB duty/brake/fault describe the output task.

| Condition | IN1 / IN2 (both channels) | EN / PWM |
|---|---|---|
| NORMAL, throttle > 0, brake = 0 | 0 / 1 | Throttle percentage |
| NORMAL, both pedals released | 0 / 0 | Low (coast) |
| Any positive brake, regardless of throttle | 1 / 1 | High (dynamic brake) |
| Startup, stale input, system error | 1 / 1 | High (dynamic brake) |

The remote high-input brake convention is preserved. Brake at EN=100% is not
a full-speed command because IN1 and IN2 are equal. The original driver briefly
disables EN around every direction/PWM update; scope measurements must include
that behavior. Output-driver failures latch OUTPUT_FAULT and attempt to disable
both EN channels. The 0.5 ms local DRIVE / 2 ms end-to-end budgets remain
unmeasured; do not claim a timing pass from compilation or functional tests.

The exhaustive host policy test checks every throttle percentage against every
positive brake percentage, coast, startup/error braking and invalid input.
The original encoder-based `dual_drive.c` and PID code remain available but are
not linked into this scope stage; their old encoder pins conflict with CAN CS.

## Build

From the configured Zephyr workspace:

```sh
west build -b nucleo_f401re /path/to/repo/stm32/lab3_app -d build-lab3-front -- -DNODE_ID=2
west build -b nucleo_f401re /path/to/repo/stm32/lab3_app -d build-lab3-rear -- -DNODE_ID=3
```

On Pi, from the repository root:

```sh
gcc -std=c11 -Wall -Wextra -Werror -O2 -Ipi pi/cockpit.c -o cockpit-control
./cockpit-control --ctrl-blink-only --pc-ip 172.26.32.245
```

The staged mode never sets the motor-enable bit and sends no force-feedback UDP.
Use the existing 276-byte laptop proxy with a 20 ms update interval. Its target
is the Pi's current IP, UDP port 8000. Input calibration/button CLI overrides
are still supported; defaults are the existing signed Mac-proxy convention.

Current deployed directory: `/home/sfarhan/lab3-heartbeat-s1`.
`sudo python3 ~/lab3-restart.py` brings interfaces up and invokes the updated
startup helper, which now selects `--ctrl-blink-only`.
Do not start a second cockpit while one owns UDP/8000 and the GPIOs.
Without valid Proxy input, four turn lamps may show startup/error hazards;
CAN degraded LEDs independently describe the physical communication paths.

### Windows G920 sender

`tools/windows_wheel_udp.py` reuses the Lab2 WinMM axes without enabling force
feedback. It packages steering X, throttle Z, brake R and the button bitmask
into the existing 276-byte UDP format. Use matching raw calibration on the Pi:

```sh
./cockpit-control --ctrl-blink-only --pc-ip WINDOWS_IP --steer 0,33151,65535 --throttle 65535,0 --brake 65535,0
```

On Windows, with the wheel initialized and G HUB available:

```powershell
python tools/windows_wheel_udp.py --pi PI_IP --status wheel-status.json
```

LSB is bit 9, RSB bit 8, X bit 2. The status file includes raw inputs and maximum
send/read intervals. Its writes run on a separate thread so disk delays do not
block acquisition. Verify actual input changes: device enumeration and arriving
UDP packets alone do not establish that the wheel is producing live values.

On the current Wi-Fi setup, a diagnostic run observed no sequence loss but
112-117 ms receive gaps while the sender's maximum gap was 47 ms. These gaps
exceeded the former 100 ms input timeout and produced transient hazards.
Disabling Pi Wi-Fi power saving did not eliminate the delay: a 60-second run
still measured a 110.645 ms maximum gap and two UDP_LOST transitions.
At the user's request, UDP input timeout is now 250 ms. This deliberately
allows brief Wi-Fi stalls while retaining the last received input. A longer
input outage still marks CTRL stale and enters error; allow the subsequent
CAN/receiver processing time in an end-to-end disconnect measurement.
The 60 ms CAN heartbeat timeout, STM32 CTRL timeout and X self-test behavior
are unchanged. This is not a measured bound on network delay or a claim of
Lab2's 150 ms disconnect compliance. For a temporary Wi-Fi setting (no reboot):

```sh
sudo /usr/sbin/iw dev wlan0 set power_save off
```

After deploying the 250 ms timeout, a 60-second passive run observed 12,181
CAN frames on A, all three nodes remaining NORMAL, BLINK mode remaining OFF,
and no UDP_LOST/FAULT transitions. The host regression checks startup without
input, a 120 ms tolerated gap, and expiry after 250 ms. This observation does
not guarantee that future network stalls cannot exceed the configured timeout.

## Validation performed 2026-10-22

Host tests: CAN sequence wrap, cross-bus dedup, same-bus repeat, conflicting
payloads, advancing reboot resync, 60 ms health, delayed-bus freshness, fixed
replay rejection, cockpit mapping, CRC/range validation, left/right selection,
self-cancel and hazard phase generation.

Both STM32 images compiled and were installed. A loopback-UDP test using the
actual cockpit binary and both physical CAN zones passed nine checks:
NORMAL on both nodes; left on/off phase; right on/off phase; auto-cancel;
BAD_INPUT FAULT from both nodes; valid-input recovery; UDP-loss hazards;
UDP-resume recovery; and no motor-enable request.

These are CAN/application-level checks, not measurements of physical lamp
brightness or GPIO edge timing. Integration results are retained in the ignored
`build-checks/ctrl-blink-test.json` and on the Pi as `ctrl-blink-test.json`.
