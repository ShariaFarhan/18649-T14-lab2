# Right-wheel RPM measurement

`tools/measure_right_rpm.py` prints a plan by default. It does not open COM10
or move a motor unless `--run` is supplied and the operator types `RIGHT`.
Use the manual motor/encoder firmware, not the steering/light-only firmware.
Keep the faulty left motor disconnected. Fix wiring/heating faults before testing.

The planned sweep is 10, 20, ... 100% PWM, 5 seconds each. It logs the right
encoder's RPM (1320 CPR) to `build-rpm/right-pwm-rpm.csv`, excluding the first
2 seconds at each duty from the displayed steady mean. It never uses the left
encoder or arms the two-wheel speed controller.

For a cold start below 40% duty, one tentative startup assist applies 40% for
approximately 150 ms, then the first requested duty. It does not repeat the
assist when the wheel stalls. `--kick-duty 0` disables it. These values are
not calibrated torque/current limits. PC scheduling and serial replies add
timing uncertainty; the pulse is not a firmware-enforced timeout.

No encoder movement for 1 second, invalid samples, reversed speed, lost replies,
or Ctrl+C abort the sweep and request right-motor coast (EN LOW). A broken
serial link or killed PC process cannot guarantee shutdown; disconnect motor
power manually if control is lost. Never push a powered wheel by hand.

Print a single-duty plan without moving hardware:

```powershell
python tools/measure_right_rpm.py --duties 10
```

Only after the wheel is raised and the wiring is checked, add `--run` to execute.
This script's startup assist does not change standalone `motor set` commands.
