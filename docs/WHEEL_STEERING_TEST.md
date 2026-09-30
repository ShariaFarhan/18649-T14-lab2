# Real-wheel steering test

This separate firmware profile tests G920 -> Windows -> Pi -> STM32 -> PA8.
It holds both motor enable outputs LOW. Keep motor power disconnected.

- Pi GPIO14/TX (physical 8) -> STM32 PA10/RX (CN10-33).
- Pi GPIO15/RX (physical 10) <- STM32 PA9/TX (CN10-21).
- Connect common GND. Servo uses its own supply and PA8 (CN10-23).
- On Pi 4, enable serial hardware and disable serial login. Use `/dev/serial0`.

Build `stm32_app` with `-DLAB2_WHEEL_STEERING=ON` and
`-DEXTRA_CONF_FILE=uart_diagnostic.conf`. Other test profiles must be OFF.

Run on the Pi (replace addresses for a different network):

```sh
python3 wheel_steering.py pi --uart /dev/serial0 --bind 100.83.135.90 --pc-ip 100.71.124.108
```

Run on Windows from the repository:

```powershell
python pi/wheel_steering.py pc --pi 100.83.135.90 --web-port 18866
```

Open http://127.0.0.1:18866/. The sender starts disabled. Click Start only
when ready, then turn gently and compare the physical servo with the reported
pulse. Stop or loss of incoming data centers a previously active servo.
Before the first valid command, servo PWM stays OFF.

The Windows reader uses WinMM, with no Logitech SDK or force effects.
G920 X is steering, Z is throttle, R is brake; each is 0..65535.
The previously observed steering center is 33151; confirm it on this setup.
STM32 maps steering to -100..100 and then 1000..2000 us, center 1500 us.
These pulse limits are not calibrated physical wheel angles.
Pedal values are transported but do not drive motors in this profile.

UDP/8000 uses the handout's 276-byte envelope. This profile's UART command
payload is 15 bytes (`int32` steering/throttle/brake, `uint16` buttons,
`uint8` flags), unlike the older 9-byte bench profile. Match both ends.
Heartbeats marked `0xB4` contain steering-test telemetry, not measured currents.

The Pi repeats commands every 20 ms and marks data stale after 120 ms without
UDP or 150 ms without a steering heartbeat. STM32's UART watchdog is 60 ms.
These are diagnostic settings, not evidence of meeting the final end-to-end deadline.
Closing the browser stops PC sending after at most the 600 ms page watchdog.
Use `steering_status` on COM10/115200 to inspect PWM and motor EN levels.
