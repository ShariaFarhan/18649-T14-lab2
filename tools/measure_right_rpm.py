"""Right motor PWM/RPM sweep. Dry-run by default; one optional startup pulse, never stall retries."""
import argparse
import csv
import re
import statistics
import time
from pathlib import Path

ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
RIGHT = re.compile(r'right count=(-?\d+) raw=\d+ delta=-?\d+ rpm_milli=(-?\d+) dt_us=\d+ sign=(-?\d+) valid=(\d+) dropped=(\d+)')


class Console:
    def __init__(self, port):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=.05, write_timeout=.3)

    def command(self, command, expected, timeout=.6):
        self.serial.reset_input_buffer()
        self.serial.write((command + '\r').encode())
        data = bytearray()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            data.extend(self.serial.read(self.serial.in_waiting or 1))
            text = ANSI.sub('', data.decode(errors='replace'))
            if re.search(expected, text):
                return text
        raise RuntimeError(f'No expected reply to {command!r}: {data.decode(errors="replace")[-300:]}')

    def sample(self):
        text = self.command('encoder status', RIGHT.pattern)
        count, rpm, sign, valid, dropped = map(int, RIGHT.search(text).groups())
        if not valid:
            raise RuntimeError('Right encoder sample is invalid')
        return count, rpm / 1000., sign, dropped

    def coast(self):
        # Never brake or drive the faulty left channel during cleanup.
        self.command('motor set right free 0', r'Motor right set to free; duty=0%')


def sweep(args):
    console = Console(args.port)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        console.serial.write(b'\x15\r')  # Clear a partially typed shell command.
        time.sleep(.1)
        console.command('encoder stop', r'Live encoder output stopped')
        state = console.command('drive status', r'armed=\d+ brake=\d+ fault=\d+')
        if 'armed=0' not in state:
            raise RuntimeError('Closed-loop drive is armed; disarm it manually before this test')
        # motor_init is intentionally omitted: it changes both bridges.
        console.command('motor set left free 0', r'Motor left set to free; duty=0%')
        console.coast()
        baseline = console.sample()
        if abs(baseline[1]) > 2:
            raise RuntimeError('Wait for the right wheel to stop before starting')
        with output.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=['pwm_percent','seconds','right_count','right_rpm','sign','dropped','steady'])
            writer.writeheader()
            for index, duty in enumerate(args.duties):
                # One cold-start assist, before sampling. Never repeat it on a stalled wheel.
                if index == 0 and args.kick_duty > duty and args.kick_ms > 0:
                    print(f'Start assist: {args.kick_duty}% for about {args.kick_ms} ms, then {duty}%', flush=True)
                    console.command(f'motor set right forward {args.kick_duty}',
                                    rf'Motor right set to forward; duty={args.kick_duty}%')
                    time.sleep(args.kick_ms / 1000.)
                console.command(f'motor set right forward {duty}', rf'Motor right set to forward; duty={duty}%')
                start = time.monotonic()
                last_motion = start
                last_count = baseline[0]
                dropped_before = baseline[3]
                speeds = []
                while time.monotonic() - start < args.seconds:
                    count, rpm, sign, dropped = console.sample()
                    now = time.monotonic()
                    elapsed = now - start
                    if count != last_count:
                        last_motion = now
                    last_count = count
                    writer.writerow(dict(pwm_percent=duty, seconds=round(elapsed,3), right_count=count,
                                         right_rpm=rpm, sign=sign, dropped=dropped, steady=int(elapsed>=args.settle)))
                    f.flush()
                    if rpm < -5:
                        raise RuntimeError('Forward command has negative RPM; check direction/sign, do not compensate with more PWM')
                    if dropped != dropped_before:
                        raise RuntimeError('Encoder sampling was interrupted')
                    if now - last_motion >= 1:
                        raise RuntimeError(f'No encoder movement at {duty}% for 1 s: aborting, no repeated boost or duty increase')
                    if elapsed >= args.settle:
                        speeds.append(rpm)
                    print(f'PWM {duty:3}% | {elapsed:5.1f}s | right {rpm:8.3f} RPM', flush=True)
                    baseline = count, rpm, sign, dropped
                    time.sleep(.15)
                print(f'PWM {duty}% steady mean: {statistics.mean(speeds):.3f} RPM' if speeds else 'No steady samples', flush=True)
        print(f'Saved {output.resolve()}')
    finally:
        try:
            console.coast()
            print('Right EN disabled (coast); no left-wheel drive command was sent.')
        except Exception as exc:
            print(f'Could not confirm motor disable: {exc}. Disconnect motor power manually.')
        finally:
            console.serial.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM10')
    parser.add_argument('--duties', default='10,20,30,40,50,60,70,80,90,100')
    parser.add_argument('--seconds', type=float, default=5)
    parser.add_argument('--settle', type=float, default=2)
    parser.add_argument('--kick-duty', type=int, default=40,
                        help='One initial startup assist, 0 disables; temporary PWM, not measured torque')
    parser.add_argument('--kick-ms', type=int, default=150,
                        help='Requested startup hold time; host/serial timing is approximate')
    parser.add_argument('--output', default='build-rpm/right-pwm-rpm.csv')
    parser.add_argument('--run', action='store_true', help='Actually command right motor; otherwise only print plan')
    args = parser.parse_args()
    try:
        args.duties = [int(x) for x in args.duties.split(',')]
    except ValueError:
        parser.error('duties must be comma-separated integers')
    if not args.duties or any(not 1 <= d <= 100 for d in args.duties) or args.duties != sorted(set(args.duties)):
        parser.error('duties must increase strictly within 1..100')
    if not 0 <= args.settle < args.seconds <= 60:
        parser.error('require 0 <= settle < seconds <= 60')
    if not 0 <= args.kick_duty <= 50 or not 0 <= args.kick_ms <= 250:
        parser.error('startup assist requires duty 0..50 and duration 0..250 ms')
    print(f'RIGHT ONLY: PWM {args.duties}, {args.seconds}s each, last {args.seconds-args.settle}s used for mean RPM')
    print(f'Initial assist: {args.kick_duty}% / approximately {args.kick_ms} ms; excluded from RPM measurements')
    if not args.run:
        print('DRY RUN: no serial connection, no motor command. Resolve the starting/heating fault before using --run.')
        return
    answer = input('Right wheel raised, left motor disconnected, wiring/encoder checked, no overheating, motor firmware loaded? Type RIGHT to run: ')
    if answer != 'RIGHT':
        print('Cancelled; no serial connection.')
        return
    sweep(args)


if __name__ == '__main__':
    main()
