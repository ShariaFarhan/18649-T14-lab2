# Dual encoder bring-up

Two hardware quadrature decoders, **3960 counts per wheel revolution**, x4 mode.
This is the selected 90:1 × 11 × 4 assumption, not a measured calibration.
This change reads encoders and computes speed; it does not add PID or command motor motion.

| Wheel | A phase | B phase | Timer |
|---|---|---|---|
| Left | PA0 / CN7-28 / A0 | PA1 / CN7-30 / A1 | TIM2 |
| Right | PB6 / CN10-17 / D10 | PB7 / CN7-21 | TIM4 |

Connect encoder power and common ground according to the actual encoder specification.
Confirm A/B signal voltage and any pull-up/level conversion first. Motor power and
encoder power are different connections; do not infer wire function from color.

## Implementation

- Zephyr's `st,stm32-qdec` driver initializes clocks, pins and encoder mode 3 (x4).
- `encoder.c` reads CNT and ARR directly, read-only. The standard sensor API reports
  integer angle modulo a turn, which loses the resolution needed for this test.
- Sample target is 10ms, priority 5; RPM uses the **measured elapsed time**.
- Counter wrap uses the actual `ARR + 1`. Zephyr may choose a multiple of CPR,
  so TIM4 must not be assumed to wrap at 65536.
- Counts accumulate in signed 64-bit software totals. Both wheels are read by one
  thread and copied through one mutex-protected snapshot.
- A sample gap over 250ms or ambiguous counter delta marks the sample invalid and
  re-establishes its baseline. Check `valid` and `dropped`; do not silently use
  stale RPM in a future controller. The method requires less than half a timer
  cycle of motion between valid samples.
- `rpm_milli = delta_counts × 60,000,000,000 / (3960 × elapsed_us)`.
  `100000` means 100 RPM. Average is the signed arithmetic mean of both wheels;
  set directions so both forward wheel speeds are positive.
- No wheel diameter is assumed, so output is RPM, not metres/second.

Existing motor TIM3 (PA6/PA7), servo TIM1 (PA8), and their original comments remain.
Default PWM2, SPI1/SPI2 and ADC1 are disabled to release conflicting pins.
USART1 moves to PA9/PA10 to free PB6/PB7; USART2 remains the debug console.
When integrating current sensing later, enable ADC1 on PC2/PC3/PC4, not PA0.

## Hand-rotation checks

Start with motor drive power disconnected; power the logic/encoder correctly.
The encoder sampling thread starts automatically. On the ST-LINK shell:

```text
encoder status
encoder zero
```

1. Mark the left wheel. Turn it forward exactly one revolution, then run
   `encoder status`: left `count` should be about 3960 in magnitude; right
   should stay near zero. Repeat for the right wheel.
2. For better calibration, zero and turn ten complete revolutions in one
   direction. Expected magnitude: about 39600. Do not power-test until the
   counts/direction make sense.
3. If forward produces negative counts, change that wheel's sign:

```text
encoder sign left -1
encoder sign right -1
```

Only change the wheel that needs it. Each sign change resets that wheel's
software count. Runtime signs reset on reboot; preserve measured directions
using `ENCODER_LEFT_DEFAULT_SIGN` / `ENCODER_RIGHT_DEFAULT_SIGN` in `encoder.h`.
4. Reverse hand rotation: count should decrease. A stopped wheel should report
   zero RPM after the next sample; total count remains. Signs can be restored
   with `encoder sign left 1` / `encoder sign right 1`.

`encoder zero` resets software counts, not hardware CNT. RPM is initially invalid
until the next sample. At a 10ms sample interval, one count corresponds to about
1.515 RPM; small low-speed jitter is expected before any filtering.

## Validation

Host test compiles the actual math module with address/undefined-behavior sanitizers:

```sh
mkdir -p build-host
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g -I. \
  tests/test_encoder_math.c logic/drive/encoder_math.c -o build-host/test_encoder_math
./build-host/test_encoder_math
python3 tests/test_brake_host.py
```

Tests cover both directions, 16/32-bit and non-power-of-two counter wrap, a full
3960-count turn, elapsed-time scaling, invalid inputs and overflow protection.
NUCLEO-F401RE firmware is built against Zephyr v4.1.0; actual pulse counting,
electrical compatibility and the 3960 assumption still require the checks above.
