# Blinker wiring and test setup

The firmware initializes all four blinker outputs off and exposes manual test
commands through the same ST-LINK serial console used by the servo test.

## Pin assignment

| Lamp | STM32 pin | Nucleo header |
|---|---|---|
| Front left | PC6 | CN10-4 |
| Front right | PC7 | CN10-19 / D9 |
| Rear left | PC8 | CN10-2 |
| Rear right | PC9 | CN10-1 |

The board normally prints `D` labels for the Arduino headers and `CN10` beside
the whole Morpho connector; it does not print `PC6`, `PC8`, or `PC9` beside
their sockets. Front-right can use the clearly marked Arduino **D9** socket.
For the other three, locate the pin-1 marker at the end of CN10. The first pair
of CN10 sockets is pins 1 and 2, the next pair is pins 3 and 4, and so on:

```text
CN10 end with pin-1 marker

CN10-1   CN10-2     rear-right, rear-left
CN10-3   CN10-4     unused, front-left
```

Do not choose the connector orientation from a photograph; start from the
physical pin-1 marker on the actual board.

## Wiring ordinary breadboard LEDs

Use one current-limiting resistor per LED. A 330 ohm resistor is a typical
starting value for an ordinary indicator LED driven by a 3.3 V GPIO, but use
the value required by the team's actual LEDs.

The LED's long leg is normally the anode (`+`). The short leg and the flat side
of the LED body normally identify the cathode (`-`). Wire each channel as:

```text
GPIO output ---- 330 ohm resistor ---- LED long leg/anode (+)
LED short leg/cathode (-) ------------- Nucleo GND
```

The resistor may be placed on either side of its LED as long as it remains in
series. All four cathodes may share one breadboard ground rail. Connect that
rail to one Nucleo `GND` pin, such as CN7-20, CN7-22, or CN10-20.

```text
CN10-4 (PC6, front-left)  ---- resistor ---->| ----+
D9      (PC7, front-right) ---- resistor ---->| ----+
CN10-2 (PC8, rear-left)   ---- resistor ---->| ----+---- Nucleo GND
CN10-1 (PC9, rear-right)  ---- resistor ---->| ----+
```

The Nucleo GPIOs power these small test LEDs directly; no external LED supply
is needed. Do not connect an LED directly to a GPIO without its resistor, and
do not apply 5 V or 12 V to these GPIOs. Automotive lamps, LED strips, and
high-power LED modules require transistor or MOSFET drivers and must not be
connected using this direct-GPIO circuit.

## Serial tests

Open the console with `tools/servo_console.sh`, then run:

```text
blinker status
blinker left
blinker right
blinker hazard
blinker off
```

Left and right have a 1000 ms period: 500 ms on and 500 ms off. Hazards have a
500 ms period: 250 ms on and 250 ms off.

Test self-cancel without moving the servo:

```text
blinker left
blinker steering -40
blinker steering 0

blinker right
blinker steering 40
blinker steering 0
```

The outbound threshold is 35 percent and the return-to-center threshold is 10
percent. `servo set` and `servo center` also feed their steering commands into
the blinker state machine, so the combined behavior can be tested after the
servo linkage is calibrated.
