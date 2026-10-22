# Team 14 Lab 3: Dual CAN

**[Wiring, running, and testing tutorial](docs/LAB3_TESTING_TUTORIAL.md)**

Windows G920 → UDP → Raspberry Pi → dual CAN → Front and Rear STM32 nodes.

Includes cross-bus relay, steering PWM, motor test outputs, synchronized lamps,
self-test buttons, watchdogs, FORCE telemetry and timing markers.
PID uses a software speed model; physical timing measurements remain pending.

## Main files

| Path | Purpose |
|---|---|
| `pi/cockpit.c` | Pi UDP/CAN controller |
| `stm32/lab3_app/` | Front/Rear firmware and pin configuration |
| `stm32/logic/` | Shared actuator, PID and state-machine drivers |
| `shared/` | CAN validation, deduplication and relay policies |
| `tools/windows_wheel_udp.py` | Windows wheel sender and dummy FORCE receiver |
| `tools/lab3_bad_input.py` | Out-of-range input test |
| `tools/lab3_capture.py` | Passive CAN capture |
| `tools/test_lab3.sh` | Host regression suite |

[Implementation notes](docs/LAB3_COMPLETION.md) ·
[Measurement worksheet](docs/LAB3_MEASUREMENTS.csv)

The older `stm32/stm32_app` and Logitech proxy directory are retained for
reference. Use **`stm32/lab3_app`** and the linked tutorial for this branch.
