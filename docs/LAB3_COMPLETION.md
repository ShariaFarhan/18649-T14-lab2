# Lab3 implementation — software review, not checkoff signoff

Based on remote main `7edeab6`, preserving the original servo, motor driver,
PID algorithm and state-machine modules. The complete version, including the
watchdog startup correction, was deployed on the team bench on 2026-10-22.
For setup and testing on another machine, start with LAB3_TESTING_TUTORIAL.md.
See the review record for the independent review outcome and subsequent fix.
Independent xhigh re-review: **software PASS**, documented in LAB3_REVIEW.md.
Hardware verification remains pending.

## What is implemented

- Standard 11-bit, DLC8 catalog, per-ID four-bit sequence, hot/parallel dual
  transmit, payload validation, first-copy dispatch and independent per-bus
  progress. Source-specific FAULT sequences avoid aliasing different nodes.
- N2 relay thread, original ID/bytes/sequence, cross-bus duplicate suppression,
  bounded pending software copies and no relay on N1/N3. Real hop timing still
  needs measurement; see LAB3_RELAY.md for the topology and queue limitations.
- Front 50 Hz SRV and split synchronized lamps; Rear original motor driver
  with independent DRIVE and 10 ms PID tasks. Brake overrides throttle.
- Board USER button on each Nucleo: leading-edge debounce 50 ms; first press
  fails locally, second within 500 ms restores. The button thread runs at
  priority 3. Local failure asserts ERR, commands neutral/brake, suppresses
  ordinary CAN TX/HB/FORCE and relay, and allows the final SELF_TEST FAULT.
  Entry also aborts each controller's pre-failure hardware TX under a per-bus
  lifecycle gate. Send rechecks the failure state inside that gate; stop/start
  and send cannot interleave. A busy/recovering bus returns EAGAIN to senders
  without blocking their attempt on the other bus.
  On recovery the main state machine requires a new valid CTRL.
- Cached master hazard phases are followed only while HB_N1 is fresh on at
  least one bus. Losing the master starts local hazards instead of holding a
  stale dark/on phase for the BLINK freshness interval.
- Independent STM32 IWDG, nominal 1000 ms timeout. Main feeds only when all
  required workers progressed within 200 ms. Idle workers wake at most every
  20 ms; network loss or an intentional button failure does not itself stop
  progress or cause a reset. A truly stalled critical worker prevents feeding.
  No watchdog pause in sleep/debug is requested. Hardware timeout/reset test
  has not yet been run.
  `CONFIG_WDT_DISABLE_AT_BOOT=y` prevents the Zephyr driver from preinstalling
  its default 100 ms timeout; the application explicitly starts the IWDG later.
  A build assertion guards that requirement. Without it, the first deployment
  repeatedly reset and reported OUTPUT_FAULT; the corrected version boots.
- N2 FORCE dummy signed counter -100..100 every 50 ms, identical copies on
  A/B, sequence advanced once. Pi decodes it and --lab3-demo returns one signed
  byte to the PC's UDP/8001. The Windows receiver records it only; it does not
  invoke force effects or change steering-wheel resistance. Stale/error FORCE
  feedback is zeroed.
- RX_A/B markers before dedup, TX_A/B markers for successful queuing, ERR on
  each node, existing SRV/PWM_SET/PWM_OUT/DIR/lamps/RELAY_TX, and an optional
  per-task execution marker. No SPI, printing or allocation occurs in an ISR.

Controller recovery keeps a stalled indication until an actual successful
TX, with staggered 64-69 ms observation/retry windows. Physical restore timing
must still be measured. The lifecycle gate is included in blocking analysis;
health snapshots and self-test aborts may wait for an in-progress transaction.
- Live bad-input injection through a permission-restricted local Unix socket;
  control remains connected while the next CTRL packets carry out-of-range
  steering. Restore input explicitly or through the tool's finally handler.

## PID feedback is simulated, explicitly

Sensors/actuators are optional per handout p18, but the p8 PID task is present.
The software executes the team's `dual_speed_pid.c` every 10 ms against an
explicit bench plant: 320 RPM at full duty, first-order 200 ms response,
50 ms braking response. These constants are **assumptions**, not measured
encoder readings. Telemetry must not call this actual wheel speed.

Throttle maps to the original 280 RPM maximum target with the original 2%
deadband. A stopped model gets at least 40% initial duty for up to 150 ms;
the original feedforward/PID then updates duty. Any positive brake and system
error cancel boost/PID and command dynamic brake. Coasting resets the PID.
DRIVE handles commands immediately; it does not wait for the next PID tick.
The PID/DRIVE mutex serializes the shared model/controller and hardware writes.
This critical section is a blocking term to measure in RTA. Both use the same
signed model speed; physical encoders are not connected by this candidate.

This changes the earlier scope-only throttle-to-duty mapping: do not expect
50% throttle to give exactly 50% duty. The PWM carrier remains **20 kHz**.
Real motors must remain unpowered during this candidate's scope demonstration.

## Pin table

| Signal | Pi (BCM / physical) | Front N2 | Rear N3 |
|---|---|---|---|
| UDP_RX | 23 / 16 | — | — |
| CMD_TX | 17 / 11 | — | — |
| CAN_RX_A | — | CN7 pin 1 (PC10) | CN7 pin 1 (PC10) |
| CAN_RX_B | — | CN7 pin 2 (PC11) | CN7 pin 2 (PC11) |
| CAN_TX_A | 27 / 13 | CN7 pin 3 (PC12) | CN7 pin 3 (PC12) |
| CAN_TX_B | 22 / 15 | CN7 pin 4 (PD2) | CN7 pin 4 (PD2) |
| DEG_A | 5 / 29 | D3 | D3 |
| DEG_B | 6 / 31 | D5 | D5 |
| ERR | 13 / 33 | A5 | A5 |
| SRV | — | D7 | — |
| Left/right lamps | — | D14 / D15 | D14 / D15 |
| RELAY_TX | — | D6 | — |
| Left/right drive PWM | — | — | D6 / D7 |
| Left IN1/IN2 | — | — | A0 / A1 |
| Right IN1/IN2 | — | — | A2 / A3 |
| PWM_SET (left) | — | — | D8 |
| Additional CTRL_RX after dedup | — | — | A4 |
| Optional task execution marker | — | CN10 pin 4 (PC6) | CN10 pin 4 (PC6) |
| Self-test | Wheel X | Blue USER button | Blue USER button |

CN7 pins 1-4 are Morpho connector pins, not Arduino D1-D4. Bring two GND
rows and the required signals to each breadboard in the handout p17 order.
Scope ground is board GND. CAN wiring and termination remain unchanged.
ERR goes high on startup and while the zone reports error. DEG refers to
physical bus health; local simulated failure alone must not light DEG.

## Actual scheduling (use this in the report)

| Work | Where / priority | Trigger |
|---|---|---|
| CAN ISR / MCP2515 worker | Zephyr driver | Module INT |
| TEST_BTN | Both STM32 / 3 | GPIO ISR queues timestamp; 50 ms leading-edge debounce |
| RELAY | N2 / 4 | New frame, bounded retries when pending |
| STEER | N2 / 5 | CTRL or state change; idle progress check 20 ms |
| FORCE_TX | N2 / 6 | Absolute 50 ms schedule, nonblocking per-bus retry |
| DRIVE | N3 / 4 | CTRL or state change; idle progress check 20 ms |
| PID | N3 / 5 | Absolute 10 ms schedule |
| Dispatch / BLINK / HB / health | Main / 6 | Receive queue, HB 20 ms, loop waits at most 1 ms |
| Pi work | SocketCAN/UDP poll loop | UDP, CAN, 20 ms HB, 50 ms FORCE return |

CTRL is sent immediately on every accepted UDP update (nominal proxy period
20 ms). During delayed input, a 40 ms fallback resends the last command with
a new sequence until the approved 250 ms UDP timeout. A slightly late UDP
packet can follow a fallback closely; network bursts can also produce closely
spaced CTRL frames. Capture and RTA must include these releases/jitter rather
than assert a strict 20 ms minimum interarrival for the actual Wi-Fi run.

HB_MON and BLINK are paths in the main dispatcher, not fabricated independent
threads. Its health check runs more frequently than the template's 20 ms; RTA
must use the actual interference/release behavior. Linux CPU affinity or FIFO
priority is not silently enabled; decide it from measured Pi latency.

`-DLAB3_PROFILE_TASK=N` selects one execution marker at build time:
1 dispatch, 2 STEER, 3 DRIVE, 4 PID, 5 RELAY send, 6 FORCE send,
7 HB send, 8 health/state. Build and measure one at a time. The pulse includes
preemption and lock waiting; it is a response-duration observation, **not pure
WCET** unless interference/blocking has been accounted for separately. ISR and
driver portions require module INT/driver instrumentation in the analysis.

## Build and run

```sh
sh tools/test_lab3.sh
west build -b nucleo_f401re stm32/lab3_app -d build-front -- -DNODE_ID=2
west build -b nucleo_f401re stm32/lab3_app -d build-rear -- -DNODE_ID=3
gcc -std=c11 -Wall -Wextra -Werror -O2 -Ipi pi/cockpit.c -o cockpit
./cockpit --lab3-demo --pc-ip WINDOWS_IP --steer 0,33151,65535 --throttle 65535,0 --brake 65535,0 --control-socket /tmp/lab3-cockpit.sock
```

Do not run a second cockpit while one owns CAN GPIO and UDP/8000. The Unix
socket is created mode 0600 and removed on clean exit. After a crash, remove
the stale socket only after confirming the old process has exited.

Windows:

```powershell
python tools/windows_wheel_udp.py --pi PI_IP --status wheel-status.json
```

This waits past the known all-midpoint WinMM startup sample before sending.
The status file contains actual raw input, sender intervals, and received
dummy force count/value/timestamp. UDP/8001 must be reachable through the PC
firewall. No Logitech force-effect SDK is initialized.

On Pi during the demo:

```sh
python3 tools/lab3_bad_input.py --socket /tmp/lab3-cockpit.sock --seconds 1
python3 tools/lab3_capture.py --seconds 60 --output catalog.json
```

The capture reports CAN arrival intervals, not GPIO end-to-end latency.

## Remaining physical verification / report work

Run the complete handout checkoff: cold start, catalog rates, steering and
blinkers, brake timing, single-bus removal, whole-bus removal, crossed hop,
full node isolation/restoration, local USER buttons and X, bad input.
Independently verify IWDG reset with a deliberately stalled critical worker
in a test image/debug session, with actuator power disconnected. Record the
reset cause/time; normal self-test must not cause watchdog resets.

Use `LAB3_MEASUREMENTS.csv` to record evidence. Blank measurements are
intentional. The report still needs measured/assumed labels, task and message
RTA for single/dual/hop cases, utilization including relays, four chain
verdicts, fault matrix, values table and Lab1 revision entry. Report <=6 pages
excluding schematic/scope captures. No test here substitutes for scope proof
of 2 ms brake, 50 ms steering, 1 ms lamp synchronization or recovery limits.

The user-approved laptop UDP timeout is 250 ms. CAN peer timeout remains
60 ms and CTRL timeout on STM32 remains 100 ms. Do not report the 250 ms
setting as Lab2's 150 ms laptop-disconnect compliance.
