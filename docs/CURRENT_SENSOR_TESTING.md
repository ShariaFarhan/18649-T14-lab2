# ACS712 Current Sensor Testing

The firmware reads three ACS712 divider outputs:

| Load | STM32 pin | Nucleo header |
| --- | --- | --- |
| Left motor | PC2 / ADC1_IN12 | CN7 pin 35 |
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

With both motors stopped and servo PWM disabled, capture the sensor offsets.
If the ACS712 VCC and servo share a supply, leave that supply on: the servo's
powered-idle current becomes the baseline and movement reports the additional
current. An unpowered sensor is rejected instead of recording its near-zero
voltage as a valid offset.

```text
current zero
```

Use the bounded tests instead of sending start/read/stop commands in one burst:

```text
current motor_run left
current motor_run right
current servo_sweep
```

Each motor test runs that wheel at 50% for 600 ms, captures min/max/average/peak
current, and unconditionally requests coast afterward. The servo test captures
a -50 to +50 sweep, returns to center, and disables PWM.

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

The integrated dual-wheel heartbeat reports absolute milliamps for left motor,
right motor and servo. `65535` means the startup zero calibration was unavailable
(most commonly because sensor VCC was off). The current telemetry does not yet
enforce an overcurrent shutdown threshold.
