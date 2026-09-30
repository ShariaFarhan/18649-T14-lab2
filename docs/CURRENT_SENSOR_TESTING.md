# ACS712 Current Sensor Testing

The firmware currently reads two ACS712 divider outputs:

| Load | STM32 pin | Nucleo header |
| --- | --- | --- |
| Right motor | PC3 / ADC1_IN13 | CN7 pin 37 |
| Steering servo | PC4 / ADC1_IN14 | CN10 pin 34 |

Each ACS712 `OUT` connects to the top of the existing divider. The divider node
connects to the STM32 ADC pin. The ACS712 logic-side `GND`, Nucleo `GND`, and
external supply grounds must be common. The high-current `IP+`/`IP-` path goes
in series with the load; it does not connect to an STM32 pin.

## Shell procedure

After flashing, open the serial shell and verify both unloaded voltages:

```text
current read
```

With the right motor stopped and the servo supply off or disconnected, capture
the individual sensor offsets:

```text
current zero
```

Then run one load at a time and read the signed voltage change:

```text
motor set right forward 15
current read
motor set right free 0
```

The `delta` value is relative to the last `current zero`. Its sign depends on
the orientation of `IP+` and `IP-`; current magnitude uses its absolute value.
Do not intentionally stall a motor during initial testing.

The installed sensors are marked `ACS712T ELC-05B`, identifying the +/-5 A
variant. The firmware converts voltage change using its nominal 185 mV/A
sensitivity, the measured 5.27 V sensor supply, and the installed 1 kohm/2 kohm
divider. `current read` therefore reports signed milliamps as well as raw ADC
counts and ADC-node millivolts. The sign depends on the `IP+`/`IP-` direction.

The 1 kohm/2 kohm divider presents a 3 kohm load to the ACS712 output, below
the datasheet's specified minimum 4.7 kohm load resistance. It is usable for
initial testing, but should be replaced by a higher-resistance divider for the
final system. Update the divider constants in `current_sense.c` if its ratio or
the sensor supply voltage changes.
