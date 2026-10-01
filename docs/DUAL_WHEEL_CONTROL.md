# Dual-wheel mean-speed control

This profile extends the steering/light test to drive both motors with one PID
controller. It is separate from the old right-only profile and manual firmware.

`feedback RPM = (signed left RPM + signed right RPM) / 2`

Throttle sets 0..280 RPM with a 2% deadband. Both bridges receive the same PWM.
Feedforward uses the measured right-wheel PWM/RPM curve as a provisional starting
estimate; it is not a two-wheel loaded calibration. PID then adjusts PWM from
the mean-speed error every 10 ms. Initial gains are Kp=0.1%/RPM,
Ki=0.2%/(RPM*s), Kd=0.001%/(RPM/s). D uses measured speed, a 50 ms
low-pass filter and a 15%-duty contribution limit; it has no setpoint kick. Average regulation does not independently equalize
the wheels' speeds.

Both encoders use 1320 counts/rev. Both samples must be valid. Startup assistance
is at least 40% PWM for up to 150 ms when starting from rest, ending early if
mean speed reaches target. Brake cancels boost and PID immediately and brakes
both bridges; healthy zero throttle coasts both. Startup now enters ERROR:
both bridges braked, four hazards flashing at 2 Hz, servo initially OFF.

Motor directions follow the latest shared driver: IN1=0/IN2=1 forward,
IN1=1/IN2=0 reverse on both bridges. Brake uses IN1=IN2=0 with EN HIGH.
This dual profile now uses left +1, right -1 after the user observed that
physical right-wheel forward had negative reported RPM. Motor direction is unchanged. Verify positive RPM in the intended forward
direction before closed-loop use. Never take absolute values to hide reversed
feedback. Shell `encoder sign left/right 1/-1` can calibrate signs while stopped.

## Load-compensation observation

Keep throttle/target fixed. With a controlled light load, RPM should initially
fall, PWM should rise, and mean RPM should recover if output headroom remains.
The page shows left, right, mean and target RPM plus PWM. PWM is commanded
electrical output, not a measured torque value. Do not grab a rotating shaft
or fully block a wheel. Real hardware tuning/load recovery is not yet verified.

At the user's request, reverse-reading and no-motion fault latches are removed
from this test profile. Sustained zero speed can therefore drive PID to its 100%
limit; this is not stall/overcurrent protection and wheels must not be held blocked.
Brake priority, 0..100% PWM bounds, centralized command freshness, valid encoder data,
250 ms control-loop freshness and output-error handling remain. A bad/missing
command pauses drive rather than permanently latching a fault. Valid disabled
commands followed by released pedals can re-arm without resetting the MCU.
Hardware output failures are recorded separately from communication state.

## Required system error state

The application owns a unified state through `logic/system/system_state.c`.
Error outputs always brake BOTH motors and select 2 Hz hazards; an active
servo centers (an unstarted servo stays OFF). Motor PID is disarmed, ordinary
turn requests are ignored, and repeated packets cannot restart the hazard phase.

| Trigger | State | Recovery |
|---|---|---|
| Power-up, no valid stream | STARTUP | Valid command stream |
| X single press | SELF_TEST | Two debounced presses 50..500 ms apart |
| Missing/stale Pi commands | LINK_LOST | Valid stream resumes |
| Malformed/out-of-range command | BAD_INPUT | A valid command replaces it |
| Module/output failure | OUTPUT_FAULT | Check hardware and reset/reinitialize |

X was observed as WinMM bit 2 (mask 4). PC detects edges every approximately
5 ms, with 50 ms debounce, and sends button changes immediately in addition
to the normal 20 ms stream. STM32 performs the fail/recover decision using a
modulo-32 press counter. First connection/reboot baselines the counter; repeats
are ignored; skipped events conservatively fail; sync never clears self-test.
Double-press recovery cannot span a connection loss. Valid packets alone never
clear a latched self-test. After recovery, release pedals before restarting drive.

The PC has explicit bad-input/restore-input controls for checkoff. The injected
steering value is 70000, with otherwise valid framing/CRC. Restoring valid input
does not clear SELF_TEST. Read-only STATUS UDP probes retrieve telemetry while
the stream is stopped without refreshing command freshness or moving actuators.

UART freshness has one owner, `pi_link`: a 75 ms deadline polled every 20 ms.
Healthy polling does not change the PID or outputs; expiry emits one state
transition. The independent motor-thread timeout was removed. This targets
<=95 ms plus output execution for actual Pi-to-STM32 disconnection. Pi UDP
input and return-heartbeat staleness each use 150 ms, with separately recorded
ages. This serial-link target is not a claim about PC-network loss timing.
PC and Pi periodic sends now advance absolute deadlines to avoid accumulated
schedule drift. Telemetry retains the latest system error and link-age snapshot.
Physical button latency, GPIO response,
heartbeat jitter and waveform timing still need scope verification. Host tests
are not hardware timing evidence. Three ACS712 channels are now sampled on
PC2/PC3/PC4 and returned as absolute milliamps; overcurrent trip thresholds are
not yet enforced.

## Build and services

Build with `LAB2_WHEEL_STEERING=ON`, `LAB2_WHEEL_DUAL_DRIVE=ON`, and
`EXTRA_CONF_FILE=uart_diagnostic.conf`. Keep left-only/right-only/other profiles OFF.
Use a separate build directory. The updated binary is `lab2-dual-error-watchdog.bin`.

On Pi (deploy `wheel_dual.py`, `wheel_dual_protocol.py`, `wheel_protocol.py`,
`self_test_button.py`, `link_health.py`, and `test_points.py` together):

```sh
python3 wheel_dual.py pi --uart /dev/serial0 --bind 100.83.135.90 --pc-ip 100.71.124.108
```

On the PC, stop the old port-18866 service before starting:

```powershell
python pi/wheel_dual.py pc --pi 100.83.135.90 --web-port 18866
```

Click Start following and release both pedals. The controller automatically
becomes ready; pressing throttle then commands both motors. No separate enable
button is needed. After a disconnect, release both pedals before restarting;
the host sends disabled frames for at least 100 ms before attempting re-arm.
The old right-only page cannot enable the dual profile: its enable bit and
packet trailer are different. Service startup itself stays stopped.

## Test transport

UDP: first 276 bytes retain the wheel layout, then `DD02`, enable (0/1),
test counter (0..31), and sync (0/1): 283 bytes. This is a documented custom
extension, not an unmodified course proxy packet. Older trailers are rejected.
UART: 17-byte raw command: the prior 15-byte command plus test counter and
test flags (bit 0 = sync). Main flags retain 0x01 stale and 0x08 drive enable.
Heartbeat BA is 31 bytes: absolute left/right/servo milliamps, steering/light
telemetry, drive telemetry, uint8 test-counter ack and uint8 system reason
(NORMAL=0, STARTUP=1, LINK_LOST=2, BAD_INPUT=3,
SELF_TEST=4, OUTPUT_FAULT=5). Startup is zone SYS_ERROR, not NORMAL.
Current value 65535 means startup calibration was unavailable.

Software checks exercise mean calculation, increased PWM on reduced feedback,
both outputs, brake priority, startup timing, invalid-feedback pauses, absence
of reverse/stall latches, output failures, release-to-arm and incompatible packets.
Application tests additionally exercise startup outputs, single/double press,
repeated counters, sender sync/restart, bad input and link recovery, with the
actual application callbacks and state model. Blinker tests cover 2 Hz hazards.
