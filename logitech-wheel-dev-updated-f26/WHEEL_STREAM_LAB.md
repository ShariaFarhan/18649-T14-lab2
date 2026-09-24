# Lab: Streaming Wheel Data to the Raspberry Pi

## Overview

The Logitech G920 plugs into your laptop. A **proxy** program on the laptop
reads the wheel and sends its state to the Pi as UDP packets. On the Pi,
`wheel_monitor` decodes each packet and prints steering, throttle, brake, and
buttons continuously, like a serial monitor.

```
G920 ──USB──> laptop (proxy) ──UDP port 8000──> Raspberry Pi 4 (wheel_monitor)
```

There are two proxies. They send byte-identical packets, so the Pi side is the
same either way:

| Laptop | Proxy | Part |
|---|---|---|
| macOS (or Linux) | `mac_proxy.py` | Part 2A |
| Windows | `proxy_gui.py` | Part 2B |

Each packet is 276 bytes: a 4-byte packet counter followed by a 272-byte
`DIJOYSTATE2` struct (defined in `proxy_receiver/state.h`).

Throughout this handout, text in `<angle-brackets>` is a placeholder. Replace
the whole thing, brackets included, with your own value.

## What goes where

| Machine | Files | Software |
|---|---|---|
| Raspberry Pi 4 | `proxy_receiver/wheel_monitor.c`, `proxy_receiver/state.h` | `gcc` (preinstalled on Raspberry Pi OS) |
| Mac | `mac_proxy.py` | Python 3, `pygame-ce` |
| Windows | `proxy_gui.py`, `logitech_steering_wheel/` folder | Python 3, `PyQt5`, Logitech G HUB or Logitech Gaming Software |

Both the laptop and the Pi must be on the same network: both on CMU-SECURE,
both on the same home Wi-Fi, or joined directly by an Ethernet cable.

---

## Part 1: Raspberry Pi

**1.1** In a terminal on the Pi desktop, find the Pi's IP address:

```
hostname -I
```

Write down the first address shown. This handout calls it `<pi-ip>`. Campus
IPs can change between sessions, so check again whenever things stop working.

**1.2** Copy the two files to the Pi. From the laptop, in the repo folder
(this works from macOS Terminal and from Windows PowerShell):

```
ssh <pi-user>@<pi-ip> "mkdir -p ~/proxy_receiver"
scp proxy_receiver/wheel_monitor.c proxy_receiver/state.h <pi-user>@<pi-ip>:~/proxy_receiver/
```

A USB stick works too. The two files just need to end up in the same folder.

**1.3** Build the monitor on the Pi:

```
cd ~/proxy_receiver
gcc -O2 -Wall -o wheel_monitor wheel_monitor.c
```

> **Checkpoint:** no output means it built.

**1.4** Start it:

```
./wheel_monitor -r
```

> **Checkpoint:** it prints `Listening on 0.0.0.0:8000 ...` followed by
> `-- no packets for Ns` once a second. That's expected, because nothing is
> sending yet. Leave it running and continue with Part 2A or 2B.

Options:

| Flag | Effect |
|---|---|
| `-r` | also print the raw index of every pressed button |
| `-n <N>` | print every Nth packet only (drops and restarts are still reported) |
| `-p <port>` | listen on a different port (default 8000) |

`wheel_monitor` and the upstream `receiver.c` both use port 8000, so only one
can run at a time.

---

## Part 2A: Proxy on macOS

**2A.1** Set up Python (one time only). In Terminal, in the repo folder:

```
python3 -m venv .venv
source .venv/bin/activate
pip install pygame-ce
```

Install **`pygame-ce`**, not `pygame`. It's a maintained fork that is still
imported as `pygame`, and plain `pygame` has no build for recent Python
versions. In every new Terminal window, run `source .venv/bin/activate` before
continuing.

**2A.2** Plug in the wheel: its **power brick first**, then USB. The wheel
does a calibration sweep, turning full left and full right, so keep your hands
clear. macOS needs no driver, and G HUB won't list the wheel on macOS; both
are expected.

**2A.3** Find out which axis number is which control:

```
python3 mac_proxy.py --probe
```

The script takes a few seconds to start.

> **Checkpoint:** you see `Wheel: ...G920...`, then a live row of
> `a0 ... a1 ... a2 ...` values.

Move **one control at a time** and record:

| Control | Axis number (`aN`) | Value when released |
|---|---|---|
| Steering wheel | `<steer-axis>` | (about 0 when centered) |
| Gas pedal | `<throttle-axis>` | +1 or −1 |
| Brake pedal | `<brake-axis>` | +1 or −1 |

Press Ctrl-C when done.

**2A.4** Start sending, using the numbers from 2A.3. For example, if the probe
showed steering on `a0`, gas on `a1` and brake on `a2`, use `--steer 0
--throttle 1 --brake 2`.

```
python3 mac_proxy.py --remote <pi-ip> --steer <steer-axis> --throttle <throttle-axis> --brake <brake-axis> --interval-ms 50
```

- If a pedal read **−1** when released, add `--invert-throttle` and/or
  `--invert-brake`.
- If macOS asks whether Python may accept incoming network connections, click
  **Allow**. That's for the return channel.

> **Checkpoint:** the Mac shows `steer ... thr ... brk ... sent N` with N
> counting up, and `force --`. The `--` is expected, because `wheel_monitor`
> never sends anything back.

Go to Part 3.

---

## Part 2B: Proxy on Windows

**2B.1** Install the Logitech software. Upstream was tested with **Logitech
Gaming Software 5.10**. The current **G HUB** is also expected to work with
the SDK. If the proxy can't connect with one, try the other.

If Windows installed its own driver for the wheel before the Logitech
software, the SDK won't see the wheel. Fix it this way:

1. Plug in and power the wheel.
2. In Device Manager, uninstall the driver Windows installed, and leave the
   wheel plugged in and powered.
3. Reinstall the Logitech software.

**2B.2** Install Python 3 from python.org, and tick "Add python.exe to PATH"
in the installer. Upstream was tested on Python 3.8; any current 3.x should
work. Then, in PowerShell, in the repo folder:

```
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install PyQt5
```

If PowerShell refuses to run `Activate.ps1`, run
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once, then try again.

Always run the proxy from the repo folder. `proxy_gui.py` imports the
`logitech_steering_wheel` folder that sits next to it, and that folder
contains the Logitech DLLs.

**2B.3** Find the laptop's own IP address:

```
ipconfig
```

Use the IPv4 address of the adapter that is on the same network as the Pi.
This handout calls it `<laptop-ip>`.

**2B.4** Configure the proxy. Open `proxy_gui.py` and edit the block marked
`## Configure this ##` near the top:

```python
REMOTE_HOST = "<pi-ip>"      # where wheel data is sent
S_PORT = 8000                # leave as is

LOCAL_HOST = "<laptop-ip>"   # must be this laptop's own IP
R_PORT = 8001                # leave as is
```

`LOCAL_HOST` must be an address this laptop actually has. If it isn't, the
proxy crashes on startup with an `OSError` about the requested address. Don't
commit these edits; the addresses are specific to your setup.

**2B.5** Plug in the wheel (power brick first, then USB), then start the proxy:

```
python proxy_gui.py
```

If Windows Defender Firewall asks about Python, allow it on the network type
you're using.

**2B.6** In the window that opens, click **connect**.

> **Checkpoint:** the console prints `initialized successfully` and
> `connected to a steering wheel at index 0`, and the wheel gives a short
> bump. The three number boxes in the window show steering, throttle, and
> brake, and they change as you move the controls.

The console will also print `No data` many times a second. That's expected:
the proxy looks for a return (force) packet on every tick, and `wheel_monitor`
never sends one.

Keep the proxy window open. **stop** disconnects from the wheel.

---

## Part 3: Verify on the Pi

Once the proxy is sending, `wheel_monitor` prints
`== receiving from <laptop-ip>`, followed by one line per packet:

```
  12.35s #241      steer   -8123  thr  +31002  brk      +0    - - - - --- --- -- --   raw:
```

| Column | Meaning |
|---|---|
| `12.35s` | time since the monitor started |
| `#241` | packet counter from the proxy |
| `steer` / `thr` / `brk` | raw `lX` / `lY` / `lRz` values from the struct |
| `A B X Y LSB RSB PL PR` | button name when pressed, dashes when not (`PL`/`PR` are the left and right paddles) |
| `raw:` | index of every pressed button in `rgbButtons[]` (only with `-r`) |

Work through this checklist and **record what you actually see**:

| Action | Mac proxy expected | Record |
|---|---|---|
| Wheel full left, then full right | `steer` about −32767 → +32767 | |
| Gas released, then fully pressed | `thr` 0 → about +32767 | |
| Brake released, then fully pressed | `brk` 0 → about +32767 | |
| Press A, B, X, Y, LSB, RSB, left paddle, right paddle, one at a time | a number appears after `raw:` | index for each button |

About the button and pedal values:

- **Button names are provisional.** The name table in `wheel_monitor.c` uses
  the commonly cited Windows indices for the G920, which haven't been
  confirmed on this setup yet. The Mac proxy currently passes SDL's own
  button numbers through, and those differ from Windows. So a button may show
  under the wrong name or not at all. The `raw:` numbers are what matter:
  record them.
- **Windows is the reference.** On Windows, record the steer, throttle, and
  brake values at rest and at full travel. The pedal range there may differ
  from the Mac proxy's 0 → +32767, for example running from positive when
  released to negative when pressed. That's exactly what this step is meant to
  find out.

To slow the output down, restart the monitor with `./wheel_monitor -r -n 5`.

### Done when

- Steering, throttle, and brake all respond to the controls.
- The packet counter increases steadily, with at most occasional `lost`
  messages.
- You have recorded the `raw:` index for each of the eight buttons, and the
  steer, throttle, and brake values at rest and at full travel.

---

## Troubleshooting

### Pi

| Symptom | Likely cause |
|---|---|
| `-- no packets` forever | Wrong `<pi-ip>` in the proxy (run `hostname -I` again). The laptop and Pi are on different networks. Campus Wi-Fi may be blocking device-to-device traffic, in which case use an Ethernet cable. Test with `ping <pi-ip>` from the laptop. |
| `bind (is receiver or another monitor already running?)` | Something else already has port 8000, usually `receiver.c` or a second monitor. Stop it. |
| `!! N packet(s) lost` now and then | Normal on Wi-Fi. If it's constant, switch to Ethernet. |
| `!! counter went X -> 0: proxy restarted` | The proxy was restarted. Expected. |
| `!! N-byte packet ... ignored` | Something other than a wheel proxy is sending to port 8000. |
| Pi reboots, freezes, or SSH drops | Undervoltage. Run `vcgencmd get_throttled`; anything other than `0x0` means the power supply is inadequate. Use the official 5 V 3 A USB-C supply. |
| `fatal error: state.h: No such file or directory` | `state.h` isn't in the same folder as `wheel_monitor.c`. |

### Mac proxy

| Symptom | Likely cause |
|---|---|
| `No joystick found` | Wheel not powered (the brick is required), a charge-only USB cable, or a USB hub. Plug straight into the laptop. |
| `invalid int value` for `--steer` etc. | A placeholder was left in. Use the numbers from `--probe`. |
| Probe runs but every axis stays at 0 | The wheel hasn't finished calibrating, or the script was modified to remove `SDL_VIDEODRIVER=dummy`. |
| `No matching distribution found for pygame` | You installed `pygame` instead of `pygame-ce`. |
| `externally-managed-environment` from pip | The virtual environment isn't active. Run `source .venv/bin/activate`. |
| `ModuleNotFoundError: No module named 'pygame'` | The virtual environment isn't active in this Terminal window. |

### Windows proxy

| Symptom | Likely cause |
|---|---|
| `OSError` on startup mentioning the address | `LOCAL_HOST` in `proxy_gui.py` isn't this laptop's IP (step 2B.3). |
| `No module named 'logitech_steering_wheel'` | Not running from the repo folder. `cd` into it first. |
| `ModuleNotFoundError: No module named 'PyQt5'` | The virtual environment isn't active, or `pip install PyQt5` was skipped. |
| Clicking **connect** prints nothing, or not `connected ...` | The Logitech software isn't installed or running, or Windows's generic driver took over (see 2B.1). Try the other Logitech software. |
| The window's numbers don't move | The wheel hasn't finished calibrating, or the SDK lost the device. Click **stop**, unplug and replug the wheel, then click **connect**. |
| Values move in the window but the Pi sees nothing | Wrong `REMOTE_HOST`, the laptop and Pi are on different networks, or the firewall blocked Python. |
| Console floods with `No data` | Expected with `wheel_monitor` (see 2B.6). |
