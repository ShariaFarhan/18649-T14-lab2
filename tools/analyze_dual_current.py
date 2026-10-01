#!/usr/bin/env python3
"""Bounded, wheels-up comparison of left/right motor current telemetry."""

import argparse
import csv
import json
import select
import socket
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pi"))
from wheel_dual_protocol import drive_packet  # noqa: E402


RELEASED = 65535
CENTER = 32768


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


class TestLink:
    def __init__(self, pi_ip):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect((pi_ip, 8000))
        self.socket.setblocking(False)
        self.counter = 0
        self.rows = []

    def command(self, pedal_percent, enabled):
        throttle = round(RELEASED * (1.0 - pedal_percent / 100.0))
        raw = dict(x=CENTER, z=throttle, r=RELEASED, buttons=0)
        self.socket.send(drive_packet(self.counter, raw, enabled, 0, False))
        self.counter = (self.counter + 1) & 0xFFFFFFFF

    def run_stage(self, name, pedal_percent, enabled, duration, collect_after=0.0,
                  allowed_states=("NORMAL",)):
        started = time.monotonic()
        next_send = started
        latest = None
        while time.monotonic() - started < duration:
            now = time.monotonic()
            if now >= next_send:
                self.command(pedal_percent, enabled)
                next_send += 0.02
            ready, _, _ = select.select([self.socket], [], [], max(0.0, min(0.02, next_send - now)))
            if not ready:
                continue
            try:
                reply = json.loads(self.socket.recv(4096))
            except (json.JSONDecodeError, OSError):
                continue
            latest = reply.get("stm")
            if not latest:
                continue
            if latest.get("system_state") not in allowed_states:
                raise RuntimeError(f"STM32 entered {latest.get('system_state')}")
            if latest.get("drive_fault"):
                raise RuntimeError(f"drive fault: {latest.get('fault_reason')}")
            if time.monotonic() - started >= collect_after:
                currents = [latest.get("current_left_ma"), latest.get("current_right_ma")]
                if None in currents:
                    raise RuntimeError("current telemetry unavailable; power sensors and reset STM32")
                self.rows.append(dict(
                    stage=name,
                    elapsed_s=round(time.monotonic() - started, 4),
                    pedal_percent=pedal_percent,
                    pwm_percent=latest["drive_pwm"],
                    left_rpm=latest["left_rpm"],
                    right_rpm=latest["right_rpm"],
                    left_ma=currents[0],
                    right_ma=currents[1],
                ))
        if latest is None:
            raise RuntimeError("no STM32 heartbeat received")

    def stop(self):
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            try:
                self.command(0, False)
            except OSError:
                pass
            time.sleep(0.02)
        self.socket.close()


def summarize(rows, stage):
    selected = [row for row in rows if row["stage"] == stage]
    if not selected:
        return None
    result = {"stage": stage, "samples": len(selected)}
    for key in ("pwm_percent", "left_rpm", "right_rpm", "left_ma", "right_ma"):
        values = [row[key] for row in selected]
        result[key] = statistics.mean(values)
        result[key + "_p95"] = percentile(values, 0.95)
    result["rpm_difference_percent"] = (
        100.0 * abs(result["left_rpm"] - result["right_rpm"])
        / max(1.0, (abs(result["left_rpm"]) + abs(result["right_rpm"])) / 2.0)
    )
    result["current_ratio_right_left"] = result["right_ma"] / max(1.0, result["left_ma"])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", default="172.26.116.157")
    parser.add_argument("--levels", nargs="+", type=int, default=[25, 50, 75])
    parser.add_argument("--settle", type=float, default=1.5)
    parser.add_argument("--sample", type=float, default=2.0)
    parser.add_argument("--output", default="dual_current_test.csv")
    args = parser.parse_args()
    if any(level < 0 or level > 75 for level in args.levels):
        parser.error("levels must be between 0 and 75 percent")

    link = TestLink(args.pi)
    stages = []
    try:
        print("Sending safe disabled frames...")
        link.run_stage("disabled", 0, False, 0.8, 0.4,
                       allowed_states=("STARTUP", "LINK_LOST", "NORMAL"))
        print("Arming at zero throttle...")
        link.run_stage("idle", 0, True, 1.0, 0.4)
        stages.append("idle")
        for level in args.levels:
            name = f"pedal_{level}"
            print(f"Running {level}% pedal for {args.settle + args.sample:.1f} s...")
            link.run_stage(name, level, True, args.settle + args.sample, args.settle)
            stages.append(name)
            link.run_stage("coast", 0, True, 0.8)
    finally:
        print("Disabling drive and applying the firmware's safe stop...")
        link.stop()

    output = Path(args.output).resolve()
    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(link.rows[0]))
        writer.writeheader()
        writer.writerows(link.rows)

    print("\nAverages (p95 values are retained in the computed result):")
    print("stage       PWM    left RPM  right RPM  left mA  right mA  R/L current  RPM diff")
    for stage in stages:
        item = summarize(link.rows, stage)
        print(f"{stage:10} {item['pwm_percent']:5.1f} {item['left_rpm']:10.1f} "
              f"{item['right_rpm']:10.1f} {item['left_ma']:8.1f} {item['right_ma']:9.1f} "
              f"{item['current_ratio_right_left']:11.2f} {item['rpm_difference_percent']:8.1f}%")
    print(f"Raw samples: {output}")


if __name__ == "__main__":
    main()
