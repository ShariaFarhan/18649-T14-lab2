# Lab 2 — NUCLEO-F401RE pin assignment

Proposed shared pinout for **one NUCLEO-F401RE / STM32F401RET6**. Checked against ST UM1724 Rev 17, Table 29, and Zephyr's STM32F401 pin definitions. This is a pin-allocation document, not a compiled overlay or a hardware-tested wiring release. Confirm the physical board model before wiring.



## STM32 signal table

Left/right are viewed from the car's forward-facing direction. On 2026-09-30, the user identified bridge A as the physical right motor and bridge B as left. The logical motor mapping is corrected here; encoder wiring/signs are unchanged.

Both motor directions were subsequently reversed in software: forward is
IN1=LOW, IN2=HIGH; backward is IN1=HIGH, IN2=LOW. On the left this is
PB14/PB15; on the right this is PB12/PB13.
Encoder signs were not changed by these manual motor direction adjustments;
verify positive encoder RPM in the intended forward direction before closed-loop use.

| Signal | STM32 pin | Board header | Peripheral / role |
|---|---|---|---|
| MOTOR_L_PWM → driver ENB | PA7 | CN10-15 (D11) | TIM3_CH2, AF2 |
| MOTOR_R_PWM → driver ENA | PA6 | CN10-13 (D12) | TIM3_CH1, AF2 |
| MOTOR_L_IN1 → driver IN3 | PB14 | CN10-28 | GPIO output |
| MOTOR_L_IN2 → driver IN4 | PB15 | CN10-26 | GPIO output |
| MOTOR_R_IN1 → driver IN1 | PB12 | CN10-16 | GPIO output |
| MOTOR_R_IN2 → driver IN2 | PB13 | CN10-30 | GPIO output |
| ENCODER_L_A | PA0 | CN7-28 (A0) | TIM2_CH1, AF1; or GPIO input |
| ENCODER_L_B | PA1 | CN7-30 (A1) | TIM2_CH2, AF1; or GPIO input |
| ENCODER_R_A | PB6 | CN10-17 (D10) | TIM4_CH1, AF2; or GPIO input |
| ENCODER_R_B | PB7 | CN7-21 | TIM4_CH2, AF2; or GPIO input |
| SERVO_PWM → servo signal | PA8 | CN10-23 (D7) | TIM1_CH1, AF1 |
| CURRENT_L → conditioned sensor output | PC2 | CN7-35 | ADC1_IN12 |
| CURRENT_R → conditioned sensor output | PC3 | CN7-37 | ADC1_IN13 |
| CURRENT_SERVO → conditioned sensor output | PC4 | CN10-34 | ADC1_IN14 |
| FL — front-left LED | PC6 | CN10-4 | GPIO output |
| FR — front-right LED | PC7 | CN10-19 (D9) | GPIO output |
| RL — rear-left LED | PC8 | CN10-2 | GPIO output |
| RR — rear-right LED | PC9 | CN10-1 | GPIO output |
| PI_LINK_TX — STM32 sends | PA9 | CN10-21 (D8) | USART1_TX, AF7 |
| PI_LINK_RX — STM32 receives | PA10 | CN10-33 (D2) | USART1_RX, AF7 |
| CMD_RX test marker | PC10 | CN7-1 | GPIO output; toggle after valid command reception |
| PWM_SET test marker | PC11 | CN7-2 | GPIO output; toggle immediately after timer update |
| Common signal GND | GND | CN7-20; CN7-22; CN10-20 | Connect grounds; these are the same reference |

PWM_OUT, DIR_A, SRV and FL/FR/RL/RR test points are taps of the actual signals above, **not additional GPIO assignments**. Do not electrically join separate signals.

## Why these pins

- Motors share TIM3: one common PWM frequency, two independent duty cycles.
- Servo uses TIM1: its pulse period does not change the motor PWM frequency.
- TIM2 and TIM4 are reserved for encoder counting. Do not use either timer for another PWM at the same time. The encoder implementation can use timer encoder mode or GPIO decoding, but must choose one mode per pin.
- USART1 uses PA9/PA10. Zephyr's default PB6/PB7 USART1 mapping would conflict with the right encoder and must be overridden.
- PA2/PA3 and USART2 remain reserved for the onboard ST-LINK debug console. PA13/PA14 remain reserved for SWD; PA5 remains available for onboard LD2; PC13 remains the onboard USER button. Lab2 self-test still uses the wheel button.
- PC2/PC3/PC4 avoid the PC0/PC1 versus PB8/PB9 solder-bridge alternatives on CN7-38/36.

## Pi-to-STM32 wiring

Selected baseline: **3.3 V GPIO UART, 115200 baud, 8N1**, with Pi 4 / Pi 5 and STM32 independently powered.

| Raspberry Pi 4 / Pi 5 | Connect to Nucleo |
|---|---|
| GPIO14 / TXD, physical pin 8 | PA10 / RX, CN10-33 |
| GPIO15 / RXD, physical pin 10 | PA9 / TX, CN10-21 |
| GND, physical pin 6 | GND, CN7-20 |

Enable the GPIO UART and ensure no login console uses that device. On Pi 4, verify the `/dev/serial0` mapping. On the tested Pi 5, `sudo dtoverlay uart0-pi5` enables GPIO14/15 as `/dev/ttyAMA0` until reboot; `/dev/serial0` points to the separate debug UART (`ttyAMA10`) and must not be used for these wires. Sharia's `pi/link_forwarder.c` currently specifies `/dev/ttyUSB0` with a TODO: change it for the GPIO-UART baseline. Keep USART2 for debug logs; do not print console text onto the USART1 binary protocol link.

If the team already has a **3.3 V USB-to-TTL adapter**, it can instead connect adapter TX → PA10, adapter RX → PA9, adapter GND → common GND. Then the Pi device may be `/dev/ttyUSB0` (verify it). Do not connect both UART sources simultaneously. Do not connect an adapter's power pin or an RS-232-voltage signal to these GPIOs.

## Required mini-breadboard test rows

| Row | Label | Wire from |
|---|---|---|
| 1 | GND | Common GND |
| 2 | GND | Common GND |
| 3 | UDP_RX | Pi GPIO23, physical pin 16; UDP receive marker |
| 4 | CMD_TX | Pi GPIO24, physical pin 18; UART-send marker |
| 5 | CMD_RX | STM32 PC10 / CN7-1 |
| 6 | PWM_SET | STM32 PC11 / CN7-2 |
| 7 | PWM_OUT | Tap left motor PWM, PA7 / CN10-15 |
| 8 | DIR_A | Tap left motor IN2, PB15 / CN10-26 |
| 9 | SRV | Tap PA8 / CN10-23 |
| 10 | FL | Tap PC6 / CN10-4 |
| 11 | FR | Tap PC7 / CN10-19 |
| 12 | RL | Tap PC8 / CN10-2 |
| 13 | RR | Tap PC9 / CN10-1 |

One signal per connected breadboard row, in the handout's order. The four software markers are implemented in the running Python/STM32 bench path. PWM_OUT taps the left motor (PA7), and PWM_SET marks its final requested duty write. With the reversed left direction and LOW/LOW dynamic braking, DIR_A taps PB15: HIGH in forward drive, LOW in braking. For reverse-to-brake tests, PB14 is the changing input instead.

## Devicetree changes required

| Node / configuration | Required assignment |
|---|---|
| `usart1` | `usart1_tx_pa9`, `usart1_rx_pa10`; 115200; no hardware flow control |
| `usart2` | Keep PA2/PA3 for debug console |
| `timers3` / `pwm3` | `tim3_ch1_pa6`, `tim3_ch2_pa7` |
| `timers1` / `pwm1` | `tim1_ch1_pa8` |
| `adc1` | `adc1_in12_pc2`, `adc1_in13_pc3`, `adc1_in14_pc4`; channels 12/13/14 |
| Encoder timer reservation | TIM2 PA0/PA1; TIM4 PB6/PB7 |
| Direction, lamps, test markers | GPIO definitions according to the signal table |
| Conflicting defaults | Disable unused SPI1, SPI2 and I2C3; disable default PWM2/`pwmleds` usage before assigning TIM2 to an encoder; replace the default ADC PA0 assignment |

The current overlay has been built with Zephyr 4.1.0. Keep the encoder, UART, PWM and test-point assignments synchronized with the source; listed current-sensor ADC pins remain reserved until current sensing is integrated.

## Electrical connections to confirm on the actual kit

- These assignments assume a dual driver with ENA/ENB and IN1-IN4, such as the L298N mentioned in the handout. Motors connect to the driver outputs, never to STM32 GPIOs. If ENA/ENB are jumpered to a supply, remove/configure those jumpers before attaching PWM. Verify the driver's brake truth table: disabling enable may coast rather than brake.
- Current-sensor outputs entering PC2/PC3/PC4 must remain within the ADC input range (0 to VDDA, normally about 3.3 V). Sensor model, supply and any scaling circuit are not yet known. Do not directly connect a possible 5 V analog output.
- Check encoder output voltage/type and required pull-ups before connecting A/B. Use 3.3 V-compatible signals or appropriate level conversion.
- Servo power comes from its properly adjusted external supply, not the MCU GPIO or board logic rail. Confirm that the servo accepts the 3.3 V control signal; add a suitable buffer if its datasheet requires one.
- Each LED needs a series resistor. All signal grounds are common; route motor/servo return current through their supply wiring, not through thin MCU signal-ground jumpers. Do not join the Pi and Nucleo 5 V power outputs.

## Sources

- [ST UM1724 Rev 17 — Figure 17 and Table 29](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
- [Zephyr Nucleo F401RE board documentation](https://docs.zephyrproject.org/latest/boards/st/nucleo_f401re/doc/index.html)
- [Zephyr board devicetree defaults](https://github.com/zephyrproject-rtos/zephyr/blob/main/boards/st/nucleo_f401re/nucleo_f401re.dts)
- [Zephyr STM32F401 pin multiplexing definitions](https://github.com/zephyrproject-rtos/hal_stm32/blob/main/dts/st/f4/stm32f401r%28d-e%29tx-pinctrl.dtsi)
- [Raspberry Pi UART configuration](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts)
- Lab2 handout: single STM32, serial command/status link, three current sensors, four lamps and the 13-row test-point board.

## UART transport bench profile

Build with `-DLAB2_UART_DIAGNOSTIC=ON -DEXTRA_CONF_FILE=uart_diagnostic.conf`.
This preserves the COM9 motor/servo/encoder shell and enables USART1 receive + heartbeat.
Incoming UART commands are printed only; they do not actuate outputs.
The diagnostic heartbeat reports state 4 (not vehicle-ready) and 65535 for unavailable currents.

On Pi: `python3 tests/pi_uart_bench.py /dev/ttyAMA0`.
On STM32 console: `link status`, then `drive status`.
The Pi script sends 26 valid commands and one deliberately bad CRC.
Expect 26 accepted, one rejected, correct decoded values, and link-lost after silence.
This checks this branch's legacy 9-byte command payload only, not the separate v2 integration forwarder or UDP path.

Bench result (2026-09-30): Pi 5 GPIO UART at 115200 8N1 + NUCLEO-F401RE.
STM32 accepted 26/26 valid commands, rejected the one corrupt frame, and reported zero sequence gaps.
Decoded values matched 1234 / 5678 / 90 / buttons 5 / flags 1.
Pi received 85 CRC-valid heartbeats with zero sequence gaps during this short test.
After silence, STM32 reported link-lost; this is functional timeout observation, not a measured deadline bound.
COM9 remained independently usable after reconnecting ST-LINK USB.
Drive status stayed disarmed with brake on. GPIO UART enablement on Pi is runtime-only until made persistent.
