# Mean-wheel-speed closed loop

Feedback is `(left RPM + right RPM) / 2`, using the 3960-count x4 encoder module.
The controller produces a common drive duty for both motors. It is a **PI**
controller (PID with D=0), with feedforward and anti-windup. It does not implement
an electronic differential or individually force the two wheel speeds equal.

## Speed is not PWM

The selected supplier table gives 110 RPM no-load and 85 RPM rated speed at 12 V.
100% PWM only keeps the driver enabled; it does not guarantee either speed on
the actual car. Supply voltage, L298N voltage drop and load affect the result.
The Lab2 handout lets the team choose and document full-throttle target speed.

`SPEED_FULL_THROTTLE_RPM = 85` is a **placeholder**, not a handout requirement or
measured safe limit. Replace it with a sustainable loaded speed that leaves
control headroom. Confirm actual motor voltage rating before any power changes.

Throttle is 0..1000 at the C API (0..100% in the shell). A 2% deadband is applied
once here: target = max_RPM × max(0, (throttle−20)/980). The motor driver receives
the resulting PWM; it must not apply another throttle deadband.

## Behavior

| Condition | Action |
|---|---|
| Startup | Motor initialization, then brake; closed loop remains disarmed |
| Armed, throttle above deadband | Sample mean speed and update both motor duties |
| Healthy, throttle zero | Coast (EN low), reset integral |
| Brake on | Apply both brakes immediately in the API call; periodic loop cannot overwrite it |
| Brake off | Resume retained throttle target if armed and fault-free |
| Invalid encoder / direction / output / input | Latch fault and attempt both brakes |
| Disarm | Brake, clear target, release closed-loop ownership |

The loop sleeps approximately 10ms and uses measured elapsed time, not a fixed
assumed interval. New input applies synchronously instead of waiting for the
next loop; the actual 2ms requirement still needs an oscilloscope measurement.
PWM_SET is PC11 / CN7-2, toggled after the two motor drive updates. While braking,
the reported `drive_pwm=0` means zero propulsion; EN is held HIGH for dynamic brake.

Initial tuning in `speed_pi.c`: Kp=0.4 duty-percent/RPM, Ki=0.8
duty-percent/(RPM·second), feedforward based on 110 RPM at 100% duty. These are
tentative tuning values. No hardware stability or speed-accuracy result is claimed.

Bring-up guards: either wheel reporting below −5 RPM during a forward command
latches a direction error; either wheel below 1 RPM for 1s while duty is at least
20% latches a no-motion error. These encoder-based guards are bench safeguards,
not current-sensor protection or replacements for correct wiring/calibration.
Fault codes: 0 none, 1 encoder, 2 direction, 3 no motion, 4 output, 5 external/input.

## Bench use

First verify encoder counts and signs, actual motor voltage, driver wiring and
forward direction. Use a controlled, current-limited setup with wheels clear.
Nothing is flashed or actuated by the host tests.

```text
encoder status
drive arm
drive throttle 10
drive status
drive brake on
drive brake off
drive throttle 0
drive disarm
```

`drive status` prints target/feedback in milli-RPM: 25000 means 25 RPM.
Start low and tune on the real kit; do not assume the placeholder full-throttle
target is achievable. Correct fault cause before `drive disarm` then `drive arm`.
Zeroing/changing encoder sign while running invalidates feedback and can fault;
disarm before doing either.

`motor set` / `motor init` are rejected while closed-loop control is armed.
`motor brake` and `motor set ... stop ...` apply a persistent brake override while
armed. Manual commands and the loop share one lock so they cannot fight over PWM.
Direct low-level motor/debug APIs must not be called concurrently with the loop.

## Next integration step

This branch still uses the standalone motor/servo/blinker shell main. It is **not
yet merged with the Pi-link integration branch**. That integration should call
`speed_control_set_input(throttle_permille, brake)` for validated pedal updates
and `speed_control_set_fault()` for link loss/self-test/invalid commands. The
supervisor must manage arming/recovery; the shell is not a substitute for it.

## Validation

- Synthetic two-wheel plant test: average speed reaches target and recovers
  after adding load to one wheel. This is a software model, not a motor measurement.
- Actual control wrapper with mocked I/O: brake priority, coast, manual ownership,
  mean feedback, fault latching, no-motion protection and malformed input.
- Existing encoder math and single/dual brake tests pass.
- NUCLEO-F401RE build: Zephyr v4.1.0, GNU Arm 10.3.1, minimal libc. No hardware run.

Run host checks using `cc` with `tests/test_speed_pi.c` + `logic/drive/speed_pi.c`
and `-I. -lm`, then `python3 tests/test_speed_control_host.py`.
