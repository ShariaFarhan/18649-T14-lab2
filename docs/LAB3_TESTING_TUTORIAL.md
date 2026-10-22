# Lab 3: wiring, running, and testing the Team 14 demo

Use branch **`codex/lab3-complete-test-guide`**. This guide describes the
dual-CAN application in `stm32/lab3_app`, not the older Lab 2 UART application.

## 1. What this version does

The Windows PC reads a Logitech G920 and sends UDP/8000 to a Raspberry Pi 4
(N1). The Pi sends identical CAN messages on buses A and B. Front/N2 controls
the steering PWM and front lamps; Rear/N3 controls the motor test outputs and
rear lamps. N2 also forwards messages between the buses when needed.

Implemented: first-copy deduplication, per-bus fault LEDs, self-test buttons,
error outputs, independent STM32 watchdogs, cross-bus relay, and dummy FORCE
telemetry returned to Windows on UDP/8001.

**Start with LEDs and an oscilloscope; leave real motor/servo power disconnected.**
The 10 ms PID task executes the team's PID algorithm against a **software speed
model**, not encoder measurements. FORCE is a signed counter, not measured
torque, and the Windows sender never applies force-feedback effects.

Functional operation has been checked on the bench. Software review passed,
including a subsequent watchdog-startup fix. Scope deadlines and a deliberate
watchdog-reset test still need recorded measurements; do not claim they passed
just because the system runs.

## 2. Hardware and node labels

- One Raspberry Pi 4; two **NUCLEO-F401RE** boards.
- Six XIAO CAN expansion modules: two per node, **16.000 MHz** oscillators.
- Four turn LEDs plus six CAN-degraded LEDs; one 330 ohm resistor per LED.
- Four 120 ohm termination resistors: two per bus.
- Common ground, breadboards/jumpers, and an oscilloscope.

Label the boards **Front/N2** and **Rear/N3** before flashing. Each board gets a
different image. USB drive letters and COM numbers are not permanent identities.
The original bench used D:/COM10 for Front and G:/COM9 for Rear; verify them on
your computer rather than assuming those assignments.

### CAN-module pin labels

The module's side-header mapping is **D10 = MOSI/SI, D9 = MISO/SO**. The lab
handout's side-header MOSI/MISO labels were reversed. Use the mapping below.
MOSI connects to MOSI/SI, and MISO connects to MISO/SO; these are not UART wires.

### Pi to its two CAN modules

BCM numbers and physical 40-pin-header numbers are different numbering systems.

| Module connection | Pi BCM GPIO | Physical pin | Module side-header pin |
|---|---:|---:|---|
| Both modules: 3.3 V | — | 1 or 17 | 3V3 |
| Both modules: ground | — | 6, or another GND | GND |
| Both modules: SCK | 11 | 23 | D8 |
| Both modules: MOSI | 10 | 19 | D10 |
| Both modules: MISO | 9 | 21 | D9 |
| CAN A: CS | 8 / CE0 | 24 | D7 |
| CAN A: INT | 25 | 22 | D6 |
| CAN B: CS | 7 / CE1 | 26 | D7 |
| CAN B: INT | 24 | 18 | D6 |

### Each STM32 to its two CAN modules

Use this same table on **both** Nucleos. D-numbers refer to the Nucleo Arduino
headers in the middle column, and to the CAN module in the last column.

| Module connection | Nucleo pin | Module pin |
|---|---|---|
| Both modules: supply | 3V3 | 3V3 |
| Both modules: ground | GND | GND |
| Both modules: SCK | D13 | D8 |
| Both modules: MOSI | D11 | D10 |
| Both modules: MISO | D12 | D9 |
| CAN A: CS | D10 | D7 |
| CAN A: INT | D2 | D6 |
| CAN B: CS | D9 | D7 |
| CAN B: INT | D4 | D6 |

SPI clock/data are shared by the two modules **on the same node**. Do not join
the Pi's SPI wires to either Nucleo's SPI wires. Use 3.3 V for these CAN modules;
leave their unused 5 V pin disconnected.

### The two CAN buses

Make four separate conductors/rails: **A_H, A_L, B_H, B_L**.

- All three CAN A modules connect H to A_H and L to A_L.
- All three CAN B modules connect H to B_H and L to B_L.
- A and B are electrically separate. Connect all node/module grounds together.
- At each physical end of bus A, connect 120 ohms **between A_H and A_L**.
  Do the same at the two ends of bus B. Do not connect terminators to ground.
- With power off, each complete bus should measure about **60 ohms H-to-L**.
  If using four external resistors, leave module termination solder pads open.

For fault tests, remove a module's **H/L branch wires**, not its power/SPI/GND,
and leave the bus termination resistors in place.

## 3. Lamps and oscilloscope connections

For each LED: output pin -> 330 ohm resistor -> LED anode; cathode -> GND.
Use board GND for scope ground clips. Logic outputs are approximately 3.3 V.

| Signal | Pi: BCM / physical pin | Front N2 | Rear N3 |
|---|---|---|---|
| CAN A degraded LED | 5 / 29 | D3 | D3 |
| CAN B degraded LED | 6 / 31 | D5 | D5 |
| Left turn lamp | — | D14: FL | D14: RL |
| Right turn lamp | — | D15: FR | D15: RR |
| ERR: high while in error | 13 / 33 | A5 | A5 |
| Steering SRV | — | D7 | — |
| Relay queue marker | — | D6 | — |
| Left/right motor PWM | — | — | D6 / D7 |
| Left direction IN1 / IN2 | — | — | A0 / A1 |
| Right direction IN1 / IN2 | — | — | A2 / A3 |
| PWM_SET: final left timer write | — | — | D8 |
| Extra CTRL_RX: after validation/dedup | — | — | A4 |
| UDP_RX | 23 / 16 | — | — |
| CMD_TX | 17 / 11 | — | — |
| CAN_RX_A | — | CN7 pin 1 | CN7 pin 1 |
| CAN_RX_B | — | CN7 pin 2 | CN7 pin 2 |
| CAN_TX_A | 27 / 13 | CN7 pin 3 | CN7 pin 3 |
| CAN_TX_B | 22 / 15 | CN7 pin 4 | CN7 pin 4 |

**CN7 pins 1-4 are Morpho pins, not Arduino D1-D4.** They correspond to
PC10, PC11, PC12 and PD2. A0-A5 above are Arduino analog-header labels used
as digital GPIO. D7 on Front is 50 Hz servo PWM; D7 on Rear is 20 kHz motor PWM.
Do not interchange them.

Bring test points to labeled breadboard rows in handout page 17 order, including
two ground rows. The optional task-execution marker is CN10 pin 4 on each
Nucleo; its build-time selection is explained in [LAB3_COMPLETION.md](LAB3_COMPLETION.md).

## 4. Get the code and build the STM32 images

Clone the same branch on the build computer and Pi:

```sh
git clone --branch codex/lab3-complete-test-guide https://github.com/ShariaFarhan/18649-T14-lab2.git
cd 18649-T14-lab2
```

The tested firmware uses **Zephyr 4.1.0**. Use Linux/WSL with a configured
Zephyr workspace. If you already have the course environment, activate it and
skip the installation block. A fresh Ubuntu/WSL setup using the GNU Arm toolchain:

```sh
sudo apt update
sudo apt install git build-essential cmake ninja-build gperf ccache \
  device-tree-compiler python3-dev python3-venv python3-pip \
  gcc-arm-none-eabi libnewlib-arm-none-eabi xz-utils file
python3 -m venv ~/.venvs/lab3-zephyr
. ~/.venvs/lab3-zephyr/bin/activate
python -m pip install west
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.1.0 ~/zephyrproject
cd ~/zephyrproject
west update
west zephyr-export
python -m pip install -r zephyr/scripts/requirements.txt
```

Build from that workspace. Replace the repository path; on WSL it may start
with `/mnt/c/Users/...`. Do not build `stm32/stm32_app`, which is Lab 2.

```sh
export LAB3_REPO=/absolute/path/to/18649-T14-lab2
export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
export CROSS_COMPILE=/usr/bin/arm-none-eabi-
cd ~/zephyrproject
west build -p always -b nucleo_f401re "$LAB3_REPO/stm32/lab3_app" -d build-lab3-front -- -DNODE_ID=2
west build -p always -b nucleo_f401re "$LAB3_REPO/stm32/lab3_app" -d build-lab3-rear -- -DNODE_ID=3
mkdir -p "$LAB3_REPO/build-export"
cp build-lab3-front/zephyr/zephyr.bin "$LAB3_REPO/build-export/front.bin"
cp build-lab3-rear/zephyr/zephyr.bin "$LAB3_REPO/build-export/rear.bin"
```

Keep `CONFIG_WDT_DISABLE_AT_BOOT=y`. This does **not** remove the watchdog:
it prevents Zephyr from preinstalling a 100 ms boot timeout. The application
then installs and starts its supervised 1000 ms IWDG. Removing that setting
caused the observed rapid reboot/OUTPUT_FAULT loop and is now a build error.

### Flash the correct board

Connect both Nucleos through their ST-LINK USB ports. Close serial terminals.
Copy `front.bin` to the Front board's USB mass-storage drive and `rear.bin`
to the Rear board's drive. Example PowerShell, **after checking drive letters**:

```powershell
$FrontDrive = 'D:'  # Replace with the verified Front drive
$RearDrive = 'G:'   # Replace with the verified Rear drive
Copy-Item .\build-export\front.bin "$FrontDrive\lab3-front.bin"
Copy-Item .\build-export\rear.bin "$RearDrive\lab3-rear.bin"
```

Wait for both copies to finish. Check for `FAIL.TXT` on the drives. If a disk
or debug serial port becomes unresponsive, reconnect that board's USB cable
and retry; do not repeatedly copy files to a stalled drive.

Optional serial console: install `pyserial`, list ports, then open one at a time.
Boot text identifies `N2` or `N3`. Exit miniterm with **Ctrl+]** before another
program opens the same port or before flashing.

```powershell
python -m pip install pyserial
python -m serial.tools.list_ports
python -m serial.tools.miniterm COM10 115200
```

## 5. Prepare the Pi's CAN interfaces

On Raspberry Pi OS, install tools:

```sh
sudo apt update
sudo apt install gcc python3 can-utils iproute2
```

For this Pi 4 wiring, the active `/boot/firmware/config.txt` configuration is:

```ini
[all]
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25,spimaxfrequency=1000000
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=24,spimaxfrequency=1000000
```

On older images the file may be `/boot/config.txt`. Edit the actual boot file;
do not add duplicate overlays. Reboot after changing it. These values require
the module's 16 MHz oscillator, not an 8 MHz module.

**Verify names before bringing up the demo:**

```sh
readlink -f /sys/class/net/can0/device
readlink -f /sys/class/net/can1/device
```

The required assignment is **CAN A / spi0.0 = can0**, **CAN B / spi0.1 = can1**.
If they are reversed, stop the cockpit first, check that `can_tmp0` is unused,
and swap them while down:

```sh
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 name can_tmp0
sudo ip link set can1 name can0
sudo ip link set can_tmp0 name can1
```

If already correct, skip the swap. Recheck after every boot unless persistent
naming has already been installed. The original team Pi has a naming service;
a fresh Pi does not inherit it merely by cloning this repository.

```sh
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 up type can bitrate 500000 restart-ms 20
sudo ip link set can1 up type can bitrate 500000 restart-ms 20
ip -details -statistics link show can0
ip -details -statistics link show can1
candump -c any
```

Both should be UP and normally ERROR-ACTIVE. Ctrl-C exits candump without
stopping CAN. Pi Wi-Fi power saving can be disabled temporarily with
`sudo /usr/sbin/iw dev wlan0 set power_save off`; this alone did not eliminate
all network delays on our bench.

## 6. Run the Pi cockpit and Windows sender

### Pi terminal

From the cloned repository, compile and run **one** cockpit process:

```sh
mkdir -p build build-logs
gcc -std=c11 -Wall -Wextra -Werror -O2 -Ipi pi/cockpit.c -o build/cockpit
PC_IP='YOUR_WINDOWS_IPV4'  # Replace; use ipconfig on Windows
./build/cockpit --lab3-demo --pc-ip "$PC_IP" \
  --steer 0,33151,65535 --throttle 65535,0 --brake 65535,0 \
  --control-socket "$PWD/build-logs/control.sock"
```

Leave the process running. These are the measured Windows/WinMM conventions,
not the default Mac proxy calibration. The wheel is nearly centered at 33151;
released pedals are 65535. LSB/RSB/X are button bits 9/8/2.

GPIO access normally uses the Raspberry Pi OS `gpio` group. If your user lacks
access to `/dev/gpiochip*`, add it with `sudo usermod -aG gpio "$USER"`, then
log out/in. `--no-test-points` is for software diagnostics only, not checkoff.

Use `pgrep -af cockpit` to check for an existing instance. Do not launch a second
one or blindly kill unrelated Python processes. Stop the intended cockpit with
Ctrl-C or `kill <its-PID>` before replacing it. A socket left after a crash may
be removed only after confirming its owner process has exited.

**Existing team bench:** its launcher runs from `~/lab3-heartbeat-s1` and uses
`~/lab3-heartbeat-s1/control.sock`. If using that already-running service, do
not start another cockpit; use that socket path for the injection command below.

### Windows terminal

Install Python3 and the Logitech G HUB driver. Connect wheel USB and pedals.
For initialization, connect the wheel's original power supply and allow its
automatic calibration to finish without restraining the wheel. This is separate
from the car's motor/servo power, which stays disconnected for scope tests.

From the same repository branch in PowerShell:

```powershell
New-Item -ItemType Directory -Force build-logs | Out-Null
python tools/windows_wheel_udp.py --pi YOUR_PI_IPV4 --status build-logs/wheel-status.json
```

Keep this terminal open. The script uses only Python's standard library; it
does not need pygame or the Logitech force-effect SDK. Allow its UDP traffic
to/from the Pi, including inbound UDP/8001 for dummy FORCE telemetry.

After starting, gently move the wheel and press/release a button once. A newly
opened WinMM device can initially return fixed midpoint values; the sender
waits for a non-default sample instead of transmitting fake neutral commands.

In a second PowerShell terminal:

```powershell
Get-Content .\build-logs\wheel-status.json -Raw | ConvertFrom-Json | Format-List
```

Check `connected=true`, `ready=true`, increasing `sent`, and changing raw
inputs. `force.count` should increase and its value should cycle through
-100..100. These values are dummy telemetry, not wheel torque measurements.
Pi should show NORMAL with no DEG flags when all links and inputs are healthy.

## 7. Test in this order

Return to NORMAL between tests. Do not confuse the **blue USER** button with
the **black RESET** button.

| Test | Action | Expected result |
|---|---|---|
| Cold start | Stop Windows sender, restart boards/cockpit, then start sender | Startup/error hazards; ERR high. With valid input and peer HBs, NORMAL and ERR low. Healthy CAN wiring should not show DEG merely because no wheel input exists. |
| Left/right | Press LSB or RSB once; press same button again | Corresponding front/rear pair blinks at 1 Hz; second press cancels. |
| Auto-cancel | Select a side, steer beyond 35% toward that side, return within 10% of center | Selected turn signal cancels. |
| Steering | Scope Front D7; turn left/center/right | 50 Hz, approximately 1000/1500/2000 us positive pulse width. |
| Throttle/PID | Scope Rear D6 or D7; press throttle beyond 2% deadband | 20 kHz carrier, startup assist then PID-adjusted duty. Throttle percentage is a speed target, not identical PWM duty. |
| Coast | Release both pedals | EN/PWM low, IN1/IN2 low. |
| Brake priority | Hold throttle, then press brake | Brake wins immediately: both direction inputs high, EN high. Equal inputs mean brake, not full-speed drive. |
| Front self-test | Single-click Front USER; then double-click it | Error/hazards, Front neutral, Rear brake; ordinary Front CAN sends/relay stop. Double-click restores after healthy input returns. |
| Rear self-test | Repeat with Rear USER | Same system-error behavior and recovery. |
| Cockpit self-test | Repeat with wheel X | Pi enters self-failed state; double-click restores. |

For a double-click, release between clicks and use roughly **100-500 ms**
between presses. In ordinary system error, healthy output drivers command
neutral/brake. A hardware output-driver failure instead attempts to disable
motor EN and latches OUTPUT_FAULT.

### CAN fault tests

Remove H/L branch wires only; keep power, ground, SPI and terminators.

1. Pull one node's A branch. Its DEG_A should light; controls remain functional
   on B. Restore it, then repeat for B. Test each node.
2. Disconnect every node from A. All DEG_A should light; B carries the system.
   Restore A.
3. **Crossed hop:** disconnect Pi from B and Rear from A; leave Front on both.
   Only Pi DEG_B and Rear DEG_A should remain on. Steering, lamps and Rear
   outputs still respond. Front D6 (`RELAY_TX`) toggles. Restore both branches.
4. Repeat the mirror case: Pi off A and Rear off B.
5. Disconnect both branches of one STM32. That node fails safe and the others
   enter system error. Restore both; recovery should require no RESET.
6. Regression: disconnect a node's bus, press its USER button, wait, then
   restore the bus while still self-failed. It must remain failed until the
   double-click; old queued heartbeats must not briefly revive it.

### Bad-input injection

With NORMAL control running, use a second Pi terminal in the repository:

```sh
python3 tools/lab3_bad_input.py --socket "$PWD/build-logs/control.sock" --seconds 1
```

For the preconfigured team service, the equivalent path is:

```sh
python3 ~/lab3-heartbeat-s1/tools/lab3_bad_input.py \
  --socket ~/lab3-heartbeat-s1/control.sock --seconds 1
```

Expect FAULT, ERR and fail-safe outputs, followed by automatic recovery after
valid input returns. This keeps UDP connected and tests range rejection, not
disconnection. A SIGKILL cannot run the tool's cleanup. If necessary, rerun
the same command with `--seconds 0.1` and let it finish so it sends the restore
command, or restart the intended cockpit cleanly.

## 8. Record timing; do not infer it from LEDs

| Measurement | Probe / trigger | Requirement to verify |
|---|---|---|
| Direct throttle/brake | Pi UDP_RX -> Rear DIR_A/PWM_SET | <=2 ms |
| Relayed brake | Pi UDP_RX -> Front RELAY_TX -> Rear DIR_A/PWM_SET | Record maximum; compare with 2 ms and report the analysis verdict |
| Steering | Pi UDP_RX -> Front SRV | <=50 ms |
| Blinker start | Pi UDP_RX -> requested lamp edge | <=100 ms |
| Front/rear lamp sync | FL versus RL, at least 10 edges | <=1 ms edge difference |
| Local / system self-test | USER edge -> local ERR / remote brake+ERR | <=10 ms / <=100 ms |
| DEG / restoration | Branch removal/restoration -> DEG | On<=60 ms; off<=100 ms |
| Full-isolation recovery | Restore branches -> NORMAL | <=500 ms, no manual reset |

Start servo viewing at 5 ms/div; start motor PWM viewing at 10 us/div. Use
positive pulse width/duty measurements, not just average voltage. RELAY_TX
marks all forwarded IDs; use CAN decoding to associate the relevant CTRL frame.
PWM_SET also toggles for the 10 ms PID task, so do not pair arbitrary edges
with CTRL_RX and call that command latency.

Capture catalog traffic without changing it:

```sh
python3 tools/lab3_capture.py --seconds 60 --output build-logs/catalog.json
```

This measures receive intervals, **not end-to-end GPIO latency**. Measure Pi
UDP_RX-to-CMD_TX for at least a minute, with and without load, and report max
and p99 as required. Record results in [LAB3_MEASUREMENTS.csv](LAB3_MEASUREMENTS.csv).
Keep normal, fault and recovery evidence separate.

The independent watchdog requires a deliberate critical-worker stall in a
controlled test image/debug session. Pressing RESET does not test it. That
forced-reset test remains pending; normal operation should not reset merely
because CAN is disconnected or USER self-test is active.

## 9. Troubleshooting and stopping

- **Hazards, UDP_LOST:** verify sender is running, `ready=true`, target Pi IP,
  and Pi `--pc-ip` matches this computer. Wi-Fi can change addresses.
- **Fixed 32767 axes/buttons0:** check G HUB/calibration and move the wheel or
  press a button. USB enumeration alone does not prove live input.
- **Rapid reboot/OUTPUT_FAULT:** verify the branch/config includes
  `CONFIG_WDT_DISABLE_AT_BOOT=y` and flash both correct images. Do not disable
  runtime supervision to hide a fault.
- **No FORCE in Windows:** check Pi uses `--lab3-demo`, N2 sends 0x300, UDP/8001
  is reachable, and another program is not occupying that port.
- **CAN not recovering:** check A/B naming, H/L wiring, grounds, two terminators
  per bus and 16 MHz configuration before changing bitrate or software limits.
- **ERR versus DEG:** ERR is system/local failure; DEG identifies the affected
  physical CAN interface. They are different signals.
- **250 ms input tolerance:** laptop UDP timeout was deliberately relaxed to
  250 ms. CAN heartbeat loss remains 60 ms. Do not describe this as the earlier
  Lab2 150 ms laptop-disconnect guarantee.

Stop the Windows sender with Ctrl-C to exercise input loss. Stop the Pi cockpit
with Ctrl-C for a clean shutdown. Keep real actuator power off while rewiring,
flashing, or testing fault injection. See [LAB3_REVIEW.md](LAB3_REVIEW.md) for
the review scope and [LAB3_COMPLETION.md](LAB3_COMPLETION.md) for implementation details.
