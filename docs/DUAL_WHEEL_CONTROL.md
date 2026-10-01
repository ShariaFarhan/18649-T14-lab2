# Dual-wheel mean-speed control

This profile extends the steering/light test to drive both motors with one PI
controller. It is separate from the old right-only profile and manual firmware.

`feedback RPM = (signed left RPM + signed right RPM) / 2`

Throttle sets 0..280 RPM with a 2% deadband. Both bridges receive the same PWM.
Feedforward uses the measured right-wheel PWM/RPM curve as a provisional starting
estimate; it is not a two-wheel loaded calibration. PI then adjusts PWM from
the mean-speed error every 10 ms. Kp/Ki remain the previous initial values;
there is no derivative term. Average regulation does not independently equalize
the wheels' speeds.

Both encoders use 1320 counts/rev. Both samples must be valid. Startup assistance
is at least 40% PWM for up to 150 ms when starting from rest, ending early if
mean speed reaches target. Brake cancels boost and PI immediately and brakes
both bridges; healthy zero throttle coasts both. Startup leaves drive disabled.

Motor directions follow the latest shared driver: IN1=0/IN2=1 forward,
IN1=1/IN2=0 reverse on both bridges. Brake uses IN1=IN2=0 with EN HIGH.
This dual profile now uses left +1, right +1 to match the reversed right motor
polarity; other profiles retain their defaults. Verify positive RPM in the intended forward
direction before closed-loop use. Never take absolute values to hide reversed
feedback. Shell `encoder sign left/right 1/-1` can calibrate signs while stopped.

## Load-compensation observation

Keep throttle/target fixed. With a controlled light load, RPM should initially
fall, PWM should rise, and mean RPM should recover if output headroom remains.
The page shows left, right, mean and target RPM plus PWM. PWM is commanded
electrical output, not a measured torque value. Do not grab a rotating shaft
or fully block a wheel. Real hardware tuning/load recovery is not yet verified.

At the user's request, reverse-reading and no-motion fault latches are removed
from this test profile. Sustained zero speed can therefore drive PI to its 100%
limit; this is not stall/overcurrent protection and wheels must not be held blocked.
Brake priority, 0..100% PWM bounds, 60 ms command freshness, valid encoder data,
250 ms control-loop freshness and output-error handling remain. A bad/missing
command pauses drive rather than permanently latching a fault. Valid disabled
commands followed by released pedals can re-arm without resetting the MCU.
Hardware output failures remain visible until a successful stop update.

## Build and services

Build with `LAB2_WHEEL_STEERING=ON`, `LAB2_WHEEL_DUAL_DRIVE=ON`, and
`EXTRA_CONF_FILE=uart_diagnostic.conf`. Keep left-only/right-only/other profiles OFF.
Use a separate build directory. The updated binary is `lab2-dual-pedal-control.bin`.

On Pi (deploy `wheel_dual.py`, `wheel_dual_protocol.py`, `wheel_protocol.py`, and
`test_points.py` together):

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

UDP: original 276-byte wheel state plus `DD01` and an enable byte (0/1).
Ordinary 276-byte packets mean drive disabled; right/left-only trailers are rejected.
UART: 15-byte raw command, flag 0x01 stale, flag 0x08 dual enable; stale wins.
Heartbeat: marker B8, 23 bytes: original 8-byte steering/light data followed by
int32 target milli-RPM, int32 left milli-RPM, int32 right milli-RPM, uint16 PWM%,
uint8 flags. Low flags are armed/brake/fault/boost; high nibble is reason.
This diagnostic layout does not represent current-sensor readings.

Software checks exercise mean calculation, increased PWM on reduced feedback,
both outputs, brake priority, startup timing, invalid-feedback pauses, absence
of reverse/stall latches, output failures, release-to-arm and incompatible packets.
