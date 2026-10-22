# 18-449/649 Team 14 Lab 3: Dual CAN

**Start here: [Lab 3 wiring, running, and testing tutorial](docs/LAB3_TESTING_TUTORIAL.md).**

Branch `codex/lab3-complete-test-guide` contains the integrated dual-CAN demo:
relay, USER self-test, watchdog, FORCE telemetry, steering and PID bench outputs.
Implementation details are in [LAB3_COMPLETION.md](docs/LAB3_COMPLETION.md);
review and the subsequent watchdog-startup correction are recorded in
[LAB3_REVIEW.md](docs/LAB3_REVIEW.md). PID feedback is simulated; hardware
timing/checkoff evidence is still required. Earlier stage records remain in
[LAB3_CTRL_BLINK.md](docs/LAB3_CTRL_BLINK.md) and [LAB3_RELAY.md](docs/LAB3_RELAY.md).

## Legacy Lab 2 UART application

The instructions below describe the older Lab 2 application, not the Lab 3
CAN program or its current pin assignments.

This repository contains the integrated vehicle-control path:

```text
Logitech G920 → Mac/Windows computer → UDP → Raspberry Pi → UART
    → NUCLEO-F401RE → motors, steering servo, lights and current sensors
```

The computer hosts a local browser interface at
<http://127.0.0.1:18866>. English is the default; use the **中文** button to
switch languages.

> **Safety:** Perform initial tests with both driven wheels raised. Keep hands,
> wires and clothing clear of the steering and wheels. Never hold a wheel
> stalled. The current readings are telemetry only; the current firmware does
> not implement overcurrent shutdown.

## 1. Prerequisites

### All computers

- Git and Python 3
- SSH/SCP access to the Raspberry Pi
- A working Zephyr workspace and Zephyr SDK with `west` and OpenOCD
- USB data cable for the Nucleo ST-LINK
- Computer and Pi connected to the same network

Follow Zephyr's official Getting Started Guide if the Zephyr workspace is not
installed yet: <https://docs.zephyrproject.org/latest/develop/getting_started/index.html>.

### macOS

- Terminal
- `pygame-ce` for reading the G920; installation is included below

macOS does not need Logitech G HUB for this application.

### Windows

- PowerShell and Python 3 available on `PATH`
- Logitech G HUB or Logitech Gaming Software so Windows exposes the G920
- OpenSSH Client enabled in Windows Optional Features if `ssh` and `scp` are
  not already available

The integrated Windows client uses WinMM directly and does not require PyQt.

### Raspberry Pi

- Python 3
- GPIO UART enabled on pins 8 and 10
- libgpiod v2 Python bindings (`import gpiod` must work)
- GPIO23 and GPIO24 available for the required test-point markers

Check the GPIO package on the Pi:

```sh
python3 -c "import gpiod; print(gpiod.__version__)"
```

The relay has been tested with gpiod 2.2.0. Do not run another program that
owns the UART or GPIO23/GPIO24 at the same time.

## 2. Clone or update the repository

For a new checkout:

```sh
git clone https://github.com/ShariaFarhan/18649-T14-lab2.git
cd 18649-T14-lab2
```

For an existing clean checkout:

```sh
git pull --ff-only
```

The Zephyr application is in `stm32_app`; the repository root itself is not a
Zephyr CMake application.

## 3. Wire and power the system

The complete connector table and electrical notes are in
[`docs/Lab2_NUCLEO_F401RE_Pinout.md`](docs/Lab2_NUCLEO_F401RE_Pinout.md).
The essential computer/Pi/STM32 connections are:

| Function | Connection |
|---|---|
| Pi UART TX | Pi GPIO14, physical pin 8 → Nucleo PA10, CN10-33 / D2 |
| Pi UART RX | Pi GPIO15, physical pin 10 ← Nucleo PA9, CN10-21 / D8 |
| Signal reference | Pi GND, Nucleo GND, driver GND, sensor GND and external-supply GND connected together |
| Steering servo signal | Nucleo PA8, CN10-23 / D7 |
| Left/right motor PWM | PA7 / D11 and PA6 / D12 |
| Left/right encoders | PA0+PA1 and PB6+PB7 |
| Left/right/servo current ADC | PC2, PC3 and PC4 through the installed voltage dividers |
| Indicator LEDs | PC6, PC7, PC8 and PC9; one series resistor per ordinary LED |

Important power rules:

- Power motors and the servo from the correctly adjusted external supply, not
  from a Nucleo GPIO or its 3.3 V rail.
- Never feed the ACS712's possible 5 V output directly into the STM32 ADC. The
  installed 1 kΩ/2 kΩ divider limits the ADC node; verify it before connecting.
- Do not join the Pi and Nucleo 5 V outputs.
- The three ACS712 sensors must already be powered, with the motors and servo
  stationary, when the STM32 boots. Startup records each sensor's zero-current
  voltage. If the interface later displays `—` for every current, power the
  sensors first and reset or reconnect the Nucleo USB.

## 4. Build and flash the integrated STM32 firmware

Use a separate `build-dual` directory. Replace the paths below with the local
Zephyr workspace and repository paths.

### macOS shell

```sh
export LAB2_REPO=/absolute/path/to/18649-T14-lab2
cd /absolute/path/to/zephyrproject
source .venv/bin/activate

west build -p always -b nucleo_f401re \
  "$LAB2_REPO/stm32_app" \
  -d "$LAB2_REPO/build-dual" -- \
  -DLAB2_WHEEL_STEERING=ON \
  -DLAB2_WHEEL_DUAL_DRIVE=ON \
  -DEXTRA_CONF_FILE=uart_diagnostic.conf

west flash -r openocd -d "$LAB2_REPO/build-dual"
```

### Windows PowerShell

Run these commands from a Zephyr command environment in which `west` works:

```powershell
$Repo = "C:\absolute\path\to\18649-T14-lab2"
Set-Location C:\absolute\path\to\zephyrproject
& .\.venv\Scripts\Activate.ps1

west build -p always -b nucleo_f401re `
  "$Repo\stm32_app" `
  -d "$Repo\build-dual" -- `
  -DLAB2_WHEEL_STEERING=ON `
  -DLAB2_WHEEL_DUAL_DRIVE=ON `
  -DEXTRA_CONF_FILE=uart_diagnostic.conf

west flash -r openocd -d "$Repo\build-dual"
```

A successful boot prints text similar to the following on the Nucleo ST-LINK
serial console:

```text
DUAL PID + ERROR STATE: boot braked/hazards; valid stream restores normal.
Current sensing PC2/PC3/PC4 startup calibration: 0
```

The system intentionally remains in `STARTUP`, with motors braked, until it
receives a valid command stream.

## 5. Find the network addresses and UART device

On the Pi:

```sh
hostname -I
```

Use the first address on the shared LAN as `<pi-ip>`. Campus DHCP addresses can
change, so repeat this check after a network problem.

Find the computer's address on the same network:

- macOS: `route -n get <pi-ip>` shows the interface, then run
  `ipconfig getifaddr <interface>`.
- Windows: run `ipconfig` and use the IPv4 address of the adapter on the same
  network as the Pi.

This address is `<computer-ip>` below.

The Pi 4 commonly exposes the GPIO UART as `/dev/serial0`. Some Pi 5 setups use
`/dev/ttyAMA0`. Confirm the device rather than guessing:

```sh
readlink -f /dev/serial0
ls -l /dev/ttyAMA0 /dev/serial0 2>/dev/null
```

Use the device wired to GPIO14/GPIO15 as `<pi-uart>`.

## 6. Deploy and start the Raspberry Pi relay

Only one program may own `<pi-uart>`. Leave the relay terminal open while using
the vehicle.

### From macOS

The helper copies all matching protocol files and starts the relay. Values
shown here override its team-specific defaults:

```sh
cd /absolute/path/to/18649-T14-lab2
PI_USER=<pi-user> \
PI_HOST=<pi-ip> \
MAC_IP=<computer-ip> \
PI_UART=<pi-uart> \
./tools/deploy_start_pi_dual.sh
```

Enter the Pi password when SSH prompts.
Keep this terminal running and use a second terminal for the Mac wheel client.

### From Windows PowerShell

Copy the matching relay and protocol files:

```powershell
ssh <pi-user>@<pi-ip> "mkdir -p ~/lab2-dual"
scp .\pi\wheel_dual.py `
    .\pi\wheel_dual_protocol.py `
    .\pi\wheel_protocol.py `
    .\pi\self_test_button.py `
    .\pi\link_health.py `
    .\pi\test_points.py `
    <pi-user>@<pi-ip>:~/lab2-dual/
```

Then open an SSH session:

```powershell
ssh <pi-user>@<pi-ip>
```

In that Pi session, ensure no previous relay owns the UART, then start the new
one:

```sh
fuser -v <pi-uart>
cd ~/lab2-dual
python3 -u wheel_dual.py pi \
  --uart <pi-uart> \
  --bind <pi-ip> \
  --pc-ip <computer-ip>
```

If `fuser` lists an old `wheel_dual.py pi` process, stop that old terminal or
terminate only that identified process. Do not kill an unknown process merely
to free the port.

The relay is ready when it prints:

```text
Wheel relay: UDP/8000 ...
STM32 heartbeat: {...}
```

Keep that SSH session running and use a second PowerShell window for the
Windows wheel client.

## 7. Start the computer wheel client

Connect the G920 power brick first and then USB. Keep hands clear while the
wheel performs its calibration sweep.

### macOS

One-time environment setup from the repository root:

```sh
python3 -m venv .venv-wheel
./.venv-wheel/bin/python -m pip install --upgrade pip
./.venv-wheel/bin/python -m pip install pygame-ce
```

Probe the axes if this Mac or USB configuration has not been tested:

```sh
./tools/mac_wheel_probe.sh
```

The observed G920 mapping is steering `a0`, throttle `a1`, and brake `a2`, with
both pedals near `+1` when released. Start the integrated client:

```sh
./tools/start_mac_dual.sh --pi <pi-ip>
```

If the probe finds different axes, append `--steer-axis N`,
`--throttle-axis N`, or `--brake-axis N`. Add `--invert-throttle` or
`--invert-brake` only when the corresponding pedal reads the opposite direction.

### Windows

One-time setup from the repository root:

```powershell
py -3 -m venv .venv-wheel
```

Start the integrated WinMM client directly; it has no third-party Python
dependency:

```powershell
& .\.venv-wheel\Scripts\python.exe .\pi\wheel_dual.py pc `
  --pi <pi-ip> `
  --web-port 18866
```

The default device is joystick 0. Add `--device N` if Windows assigned the G920
another index. If the client reports no usable wheel, verify G HUB/LGS and the
wheel in Windows Game Controllers before changing firmware.

Both clients should print a wheel name followed by:

```text
Wheel read-only until Start: http://127.0.0.1:18866
```

## 8. Use the browser interface

Open <http://127.0.0.1:18866> on the same computer running the wheel client.
Do not open the Pi address; this page is intentionally local to the computer.

Before pressing **Start control**, confirm:

- both wheels are raised for the first test;
- throttle and brake show released values near 65535;
- Pi and STM32 telemetry is present;
- both encoder RPM readings are zero;
- the page does not report a drive fault.

Press **Start control** while both pedals are released. The controller sends a
disabled recovery interval and then arms automatically. Apply throttle gently.
The page displays independent wheel RPMs, mean target/feedback, commanded PWM,
steering/lighting state, and three current readings.

Press **Stop and enter safe state** before closing terminals or changing
wiring. Stopping the computer sender or losing the link also brakes both motors
and selects the error-state hazard lights.

The orange **Inject invalid input (forces FAIL)** button is exclusively for
fault-injection checkoff. Do not use it during ordinary driving. A single X
button press triggers the self-test fault; a correctly timed double press
recovers it.

## 9. Quick verification and troubleshooting

| Symptom | Check |
|---|---|
| `doesn't contain a CMakeLists.txt` | Build `<repo>/stm32_app`, not the repository root. |
| Browser does not open | Confirm the computer client is still running and uses `--web-port 18866`. |
| `pi_online: false` / no heartbeat | Check `ping <pi-ip>`, both IP addresses, the Pi relay terminal, and UART wiring. |
| `BlockingIOError: Resource temporarily unavailable` | Another process owns the Pi UART. Identify it with `fuser -v <pi-uart>` and stop only the old relay. |
| `STARTUP` | Start a valid stream. This is the expected safe boot state. |
| `LINK_LOST` | Restart the PC stream or repair the UDP/UART link, then release both pedals. |
| `SELF_TEST` | X was pressed once. Use the documented double press to recover. |
| `BAD_INPUT` | Select **Restore valid input**, then release both pedals. |
| All currents display `—` | Power all ACS712 sensors, leave loads stationary, then reset the Nucleo. |
| Motor RPM sign is wrong | Stop the system and follow the encoder-sign procedure in the detailed control documentation. |

Protocol tests that do not move hardware can be run from the repository root:

```sh
python3 tests/test_dual_protocol.py
python3 tests/test_wheel_protocol.py
python3 tests/test_link_health.py
```

## Detailed documentation

- [Dual-wheel controller and safety states](docs/DUAL_WHEEL_CONTROL.md)
- [Complete Nucleo pinout and Pi UART wiring](docs/Lab2_NUCLEO_F401RE_Pinout.md)
- [macOS wheel setup details](docs/MAC_WHEEL_SETUP.md)
- [Current-sensor setup and tests](docs/CURRENT_SENSOR_TESTING.md)
- [Encoder setup and tests](docs/ENCODER_TESTING.md)
- [Servo wiring and tests](docs/SERVO_TESTING.md)
- [Blinker wiring and tests](docs/BLINKER_TESTING.md)
