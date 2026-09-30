# Dual encoder bring-up

Two hardware quadrature decoders, **1320 counts per wheel revolution**, x4 mode.
Selected from the user's ten-turn check on 2026-09-30: left 13186 counts (1318.6/turn), right 13222 (1322.2/turn), combined mean 1320.4. Use nominal 1320 CPR; the small difference is consistent with manual endpoint alignment. This is also consistent with 30:1 × 11 × 4, but the gearbox ratio itself was not independently measured.
This change reads encoders and computes speed; it does not add PID or command motor motion.

| Wheel | A phase | B phase | Timer |
|---|---|---|---|
| Left | PA0 / CN7-28 / A0 | PA1 / CN7-30 / A1 | TIM2 |
| Right | PB6 / CN10-17 / D10 | PB7 / CN7-21 | TIM4 |

Connect encoder power and common ground according to the actual encoder specification.
Confirm A/B signal voltage and any pull-up/level conversion first. Motor power and
encoder power are different connections; do not infer wire function from color.

## Standalone USB-only profile

Build in a separate directory with `-DLAB2_ENCODER_ONLY=ON` (for example, `west build -b nucleo_f401re stm32_app -d build-encoder-only -- -DLAB2_ENCODER_ONLY=ON`). This adds `boards/encoder_only.overlay`, disables TIM1/TIM3 PWM and USART1, and links only the encoder application and USB-console commands. It does not need a Pi and does not command motor or servo motion. Keep actuator power disconnected for hand-turn checks; encoder power/common ground are separate.

The board connected on 2026-09-30 enumerated as NOD_F401RE on D: and COM10. Verify the current port and drive before flashing; the earlier board used G:/COM9. Flashing the standalone profile replaces the running integrated application; original source profiles remain available.

## Continuous output

On the ST-LINK console (115200 8N1), run `encoder watch` to print left/right counts and signed RPM every 200 ms. Run `encoder stop` to stop. The dedicated display thread is lower priority than the encoder sampler, so the sampling configuration remains unchanged. `encoder zero`, `encoder sign` and `encoder status` remain available while watching. For Windows PowerShell, run `tools/Watch-Encoder.ps1 -Port COM10` from the repository root; it starts the stream and reads continuously. Ctrl+C stops the stream and closes the port. Close any existing serial session first. A normal serial terminal displays the rows continuously; a PowerShell reader must also poll the port continuously rather than only after `Read-Host` input.

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
- `rpm_milli = delta_counts × 60,000,000,000 / (1320 × elapsed_us)`.
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
   `encoder status`: left `count` should be about 1320 in magnitude; right
   should stay near zero. Repeat for the right wheel.
2. For better calibration, zero and turn ten complete revolutions in one
   direction. Expected magnitude: about 13200. Do not power-test until the
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
1320-count turn, elapsed-time scaling, invalid inputs and overflow protection.
NUCLEO-F401RE firmware is built against Zephyr v4.1.0; actual pulse counting,
electrical compatibility and repeatability still require the checks above.

## Confirmed wheel direction

On 2026-09-30, the user confirmed that left-wheel direction was correct and right-wheel direction was reversed. Defaults are now left `+1`, right `-1`; this changes signed counts and RPM, not A/B wiring or CPR. `encoder sign right -1` applies the same correction at runtime and resets the right software count baseline.

The earlier 3960 CPR setting has been replaced by measured nominal 1320 CPR in both the C configuration and devicetree. This changes RPM conversion, not physical edge counting; the earlier RPM scale was one third of the corrected scale. Left sign stays +1, right sign stays -1.
