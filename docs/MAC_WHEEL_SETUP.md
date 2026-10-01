# Mac wheel control

This setup uses the tutorial's `pygame-ce`/SDL input path and converts it to
the extended dual-drive packet required by `wheel_dual.py` and the STM32 dual
firmware. The tutorial's standalone `mac_proxy.py` sends only the base
276-byte packet and is useful for probing or `wheel_monitor`; it cannot enable
the dual-drive firmware by itself.

## Local configuration

- Pi address: `172.26.116.157`
- Control page: `http://127.0.0.1:18866`
- G920 mapping observed on this Mac: steering axis 0, gas axis 1, brake axis 2
- At rest: steering approximately 0; gas/brake approximately +1
- Python environment: `.venv-wheel` with `pygame-ce`

Probe the wheel when the USB layout changes:

```sh
tools/mac_wheel_probe.sh
```

Start the Mac client:

```sh
tools/start_mac_dual.sh
```

The client is read-only until **Start following** is clicked in the local web
page. Keep both wheels raised for first testing and release both pedals before
starting.

## Raspberry Pi relay

The Pi must run the matching files from `pi/`: `wheel_dual.py`,
`wheel_dual_protocol.py`, `wheel_protocol.py`, `self_test_button.py`,
`link_health.py`, and `test_points.py`. Start it with the Mac's current IP:

```sh
python3 wheel_dual.py pi --uart /dev/serial0 --bind 172.26.116.157 --pc-ip <mac-ip>
```

On the tested Pi 5 GPIO UART, use `/dev/ttyAMA0` instead of `/dev/serial0`.
The Pi address belongs in `--bind`; `<mac-ip>` is the Mac address on the same
network.

For the currently configured Pi and Mac, deploy the matching files and start
the relay in one command. SSH asks for the Pi password locally:

```sh
tools/deploy_start_pi_dual.sh
```
