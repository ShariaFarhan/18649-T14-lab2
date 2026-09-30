"""Scope markers using libgpiod v2 (Pi 4/5; discover header GPIO chip by line names)."""
from pathlib import Path


class TestPoints:
    def __init__(self):
        import gpiod
        from gpiod.line import Direction, Value
        self.Value = Value
        self.pins = {'udp_rx': 23, 'cmd_tx': 24}
        self.levels = dict.fromkeys(self.pins, False)
        self.counts = dict.fromkeys(self.pins, 0)
        self.request = None
        visited = set()
        for device in sorted(Path('/dev').glob('gpiochip[0-9]*')):
            resolved = device.resolve()
            if resolved in visited:
                continue
            visited.add(resolved)
            with gpiod.Chip(str(device)) as chip:
                info = chip.get_info()
                if not info.label.startswith(('pinctrl-rp1', 'pinctrl-bcm')) or info.num_lines <= 24:
                    continue
                if any(chip.get_line_info(pin).name != f'GPIO{pin}' for pin in self.pins.values()):
                    continue
                self.chip = str(resolved)
            # Never fall back to stealing another GPIO chip if these lines are busy.
            self.request = gpiod.request_lines(self.chip, consumer='lab2-test-points', config={
                tuple(self.pins.values()): gpiod.LineSettings(direction=Direction.OUTPUT,
                                                             output_value=Value.INACTIVE)
            })
            break
        if self.request is None:
            raise RuntimeError('Pi header GPIO23/24 chip not found; test markers are required')

    def toggle(self, name):
        level = not self.levels[name]
        self.request.set_value(self.pins[name], self.Value.ACTIVE if level else self.Value.INACTIVE)
        self.levels[name] = level
        self.counts[name] += 1

    def snapshot(self):
        values = self.request.get_values(list(self.pins.values()))
        return {'chip': self.chip, 'counts': dict(self.counts),
                'levels': {name: value == self.Value.ACTIVE for name, value in zip(self.pins, values)}}

    def close(self):
        if self.request is not None:
            self.request.release()
            self.request = None
