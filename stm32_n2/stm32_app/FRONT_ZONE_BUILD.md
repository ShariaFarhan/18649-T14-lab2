# Front-zone (N2) build

From the repository root, with the Zephyr workspace installed beside this repo:

```sh
source ../zephyrproject/.venv/bin/activate
export ZEPHYR_BASE="$(cd ../zephyrproject/zephyr && pwd)"
west build -b nucleo_f401re stm32_n2/stm32_app -d build-front -p always
```

The N2 app selects `front_zone.conf` and `boards/nucleo_f401re_front.overlay`
automatically. It builds only the Lab 3 CAN, steering, and front-blinker
sources. Keep `build-front` separate from any older build directory.

After verifying the physical wiring and power, flash with:

```sh
west flash -r openocd -d build-front
```

The main loop receives validated CAN events from both MCP2515s and passes them
to `front_zone`. The zone checks safety and duplicates, then `front_blinkers.c`
maps blink mode/phase to the front GPIO lamps; the zone also schedules heartbeat
and dummy force frames. At startup or on lost commands/heartbeats it requests centered steering
and flashing hazards. The overlay has a servo-current ADC channel, but current
sensing is not yet connected to this app. The physical test button is also not
yet defined or integrated. A successful build
does not establish that CAN wiring, bitrate, servo power, or fault behavior has
been verified on the vehicle.
