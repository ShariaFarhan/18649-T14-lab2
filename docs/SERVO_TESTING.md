# Steering servo test setup

The test firmware exposes manual servo commands over the Nucleo's ST-LINK
virtual serial port. It deliberately leaves PWM off at startup so an
uncalibrated servo cannot sweep into a mechanical stop unexpectedly.

## Wiring

The configured servo signal is **PA8**, available at the Arduino header socket
marked **D7** and at **CN10 pin 23**. Use D7 because it is clearly labeled on
the board; D7, CN10-23, PA8, and TIM1_CH1 are names for the same electrical
signal.

A typical hobby servo has three connections. Confirm the order from the servo
datasheet or case label because wire colors are not guaranteed.

| Servo connection | Typical color | Connect to |
|---|---|---|
| Signal | Orange, yellow, or white | Nucleo D7 |
| Power (`V+`) | Red | External regulated +5 V |
| Ground (`GND`) | Brown or black | External supply negative |

The external supply negative must also connect to a Nucleo `GND` pin. The
servo therefore has a direct high-current return to its supply plus a common
ground reference with the Nucleo:

```text
Nucleo D7 ------------------------------ servo signal

External supply +5 V ------------------- servo V+ (usually red)
External supply negative --------------- servo GND (usually brown/black)
                         |
                         +--------------- Nucleo GND

USB ------------------------------------ Nucleo USB/ST-LINK connector
```

Do **not** connect the external supply's positive terminal to a Nucleo GPIO,
3.3 V, 5 V, or VIN pin. The servo is powered by the external supply; only its
signal and common ground connect to the Nucleo. PA8 outputs a 3.3 V control
signal even though the servo power supply is 5 V.

Before connecting the servo, turn the bench-supply output off, set it to the
servo's rated voltage (5.0 V for the current setup), set an appropriate current
limit for the specific servo, and verify polarity with a meter. Turn the output
on only after all wiring is checked. If the supply enters current limit or its
voltage collapses during movement, the servo can jitter or fail to reach its
commanded position.

## Build and flash

Activate the Zephyr environment and build from the Zephyr workspace:

```sh
source /Users/ryanwk/Documents/Embedded_Systems_649/zephyrproject/.venv/bin/activate
cd /Users/ryanwk/Documents/Embedded_Systems_649/zephyrproject
west build -p always -b nucleo_f401re \
  /Users/ryanwk/Documents/Embedded_Systems_649/18649-T14-lab2/stm32_app \
  -d /Users/ryanwk/Documents/Embedded_Systems_649/18649-T14-lab2/build
west flash -r openocd \
  -d /Users/ryanwk/Documents/Embedded_Systems_649/18649-T14-lab2/build
```

With this board's current ST-LINK firmware, OpenOCD may report status 1 during
its final reset even after printing that the image was written. If the serial
shell responds with the servo and blinker startup messages, the firmware is
running and no additional flashing workaround is needed.

Open a second terminal and run:

```sh
/Users/ryanwk/Documents/Embedded_Systems_649/18649-T14-lab2/tools/servo_console.sh
```

## Serial commands

```text
servo init       # enable PWM and command the configured center
servo set -25    # command any integer from -100 through 100
servo center     # return to center
servo status     # print the last pulse width
servo off        # stop the PWM signal
```

Start near center and move outward in small increments. The initial software
calibration is 1200 us at -100, 1500 us at 0, and 1800 us at +100, with a
20 ms period. These endpoints are conservative starting values, not measured
limits for the chassis. If the linkage buzzes or strains, immediately run
`servo off`, reduce the corresponding endpoint in `logic/steering/servo.h`,
rebuild, and reflash.

## Wiring safety

- Keep the wheels off the ground during initial endpoint testing.
- Do not force the servo shaft by hand or command it against a mechanical stop.
- Stop immediately if the linkage stalls, buzzes continuously, or becomes hot.
- Disconnect supply power before moving wires or removing the servo horn.
