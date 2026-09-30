# Right-wheel closed-loop bring-up

This optional profile combines wheel steering, LSB/RSB lights, and right-wheel
velocity control. The physical LEFT motor and its encoder remain disconnected.
All left motor control outputs remain inactive; left encoder readings are ignored.
The shared encoder driver initializes both timers, but only the right sample
and its validity enter this controller.

## Measured calibration

Source: `calibration/right-pwm-rpm-20260930.csv` and its JSON summary.
Right wheel unloaded, 1320 counts/rev, sign -1, 5 seconds per duty;
the last 3 seconds (18 samples) supply each mean. All ten steps completed
with no dropped encoder samples reported. Count/time recomputation agrees.

| PWM | Mean RPM |
|---:|---:|
| 10% | 40.174 |
| 20% | 63.360 |
| 30% | 96.418 |
| 40% | 139.118 |
| 50% | 183.195 |
| 60% | 226.583 |
| 70% | 264.692 |
| 80% | 295.224 |
| 90% | 314.508 |
| 100% | 320.018 |

These are unloaded speeds, not guaranteed loaded performance. Full throttle
targets 280 RPM, leaving headroom relative to the measured 320 RPM maximum.
The measured curve is inverted with linear interpolation for feedforward PWM.
Below 40 RPM the curve is extrapolated toward zero; minimum stable speed is
not established. Initial PI gains (Kp 0.1%/RPM, Ki 0.2%/(RPM*s)) still require
real load-step tuning. Synthetic tests are not hardware validation.

## Behavior

- Pedals: released raw 65535, fully pressed 0; 2% deadband.
- Healthy zero throttle: coast. Brake beyond the deadband: immediate dynamic
  braking, overriding throttle, PI output, and startup assistance.
- Startup assistance: at rest, at least 40% PWM for up to 150 ms, ending early
  if target RPM is reached. Then measured-curve feedforward plus PI runs every
  10 ms. Braking or zero throttle cancels assistance immediately.
- No right encoder movement for 1 s, invalid feedback, reverse RPM, output
  errors, or 60 ms without fresh commands stops drive and disarms it.
- The MCU enforces the command timeout itself, including during startup boost.
- Initial motor EN is LOW. After an enabled session, stop/link loss brakes
  the right motor. The left bridge remains disabled.
- Enable requires an explicit request after a valid disabled command and
  both pedals released. Restart/reconnection with an old enable bit cannot
  automatically restart the motor. Faults require stopping and checking;
  reset the board if the UI cannot re-enable after the fault is cleared.

## Wiring and build

Right motor: ENA PA6 / CN10-13, IN1 PB12 / CN10-16, IN2 PB13 / CN10-30.
Right encoder: A PB6 / CN10-17, B PB7 / CN7-21. Servo and lights retain their
existing pins. Do not infer power wiring from wire colors.

Build in a separate directory with `LAB2_WHEEL_STEERING=ON`,
`LAB2_WHEEL_RIGHT_DRIVE=ON`, and `EXTRA_CONF_FILE=uart_diagnostic.conf`.
Leave other profiles OFF. Existing dual-wheel/manual code and its old 85 RPM
placeholder are not used by this right-wheel profile.

For timing in this profile, PWM_SET marks RIGHT updates; tap PA6 for PWM_OUT
and PB12 for the forward-to-LOW/LOW-brake edge. Other profiles retain their
LEFT timing marker. Actual timing compliance still requires scope measurements.

## Transport and UI

Run the existing `wheel_steering.py` Pi/PC services, then open port 18866.
Start following enables steering/lights. Release both pedals and click
Enable right motor separately. The page shows target RPM, measured RPM,
PWM, brake, startup assistance, and fault status. Motors default disabled.

The test sender preserves the first 276 bytes of wheel state and appends a
5-byte `RD01` + enable(0/1) trailer. This 281-byte test envelope is not the
unmodified course proxy packet. The relay also accepts ordinary 276-byte
packets with drive disabled. UART stays 15-byte raw command payload; flag
bit 0 means stale and bit 1 enables right drive. Stale always suppresses enable.
The right profile's B6 heartbeat is 19 bytes: the 8-byte B5 steering/light
layout followed by little-endian int32 target milli-RPM, int32 actual
milli-RPM, uint16 duty percent, and uint8 flags (armed/brake/fault/boost).
It is test telemetry, not the final current-sensor status format.

The last known-good steering/light-only and manual motor binaries are retained.
Only the dedicated right-wheel profile uses this measured calibration.
