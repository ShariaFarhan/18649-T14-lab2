"""Pi GPIO UART transport check for LAB2_UART_DIAGNOSTIC firmware only.

Uses this branch's legacy 9-byte command payload. It is not the v2 forwarder.
Run: python3 pi_uart_bench.py /dev/ttyAMA0
"""
import os
import select
import struct
import sys
import termios
import time
import tty


def crc(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def frame(seq, payload):
    body = bytes((len(payload), seq)) + payload
    return b'\x7e\xa5' + body + struct.pack('<H', crc(body))


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyAMA0'
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    pending = bytearray()
    received = []
    rejected = 0

    def collect(duration):
        nonlocal rejected
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if not select.select([fd], [], [], max(0, deadline-time.monotonic()))[0]:
                continue
            pending.extend(os.read(fd, 4096))
            while len(pending) >= 2:
                if pending[:2] != b'\x7e\xa5':
                    del pending[0]
                    continue
                if len(pending) < 4:
                    break
                length = pending[2]
                if length > 32:
                    del pending[0]
                    continue
                total = length + 6
                if len(pending) < total:
                    break
                packet = bytes(pending[:total])
                del pending[:total]
                if crc(packet[2:-2]) != struct.unpack('<H', packet[-2:])[0]:
                    rejected += 1
                elif length == 8:
                    received.append((packet[3], struct.unpack('<BBHHH', packet[4:-2])))

    try:
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~termios.CRTSCTS
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        collect(0.3)
        if not received or received[-1][1][0] != 4 or received[-1][1][2:] != (65535, 65535, 65535):
            raise RuntimeError('Diagnostic heartbeat not detected; no commands sent.')
        payload = struct.pack('<hhhHB', 1234, 5678, 90, 5, 0)
        for seq in range(25):
            packet = frame(seq, payload)
            if os.write(fd, packet) != len(packet):
                raise RuntimeError('Short UART write')
            collect(0.04)
        bad = bytearray(frame(25, payload))
        bad[-1] ^= 1
        os.write(fd, bad)
        collect(0.04)
        os.write(fd, frame(25, payload))
        collect(0.35)
        gaps = sum(((b[0]-a[0]) & 255) != 1 for a,b in zip(received, received[1:]))
        print(f'Device: {device}; 115200 8N1')
        print('Sent: 26 valid commands + 1 intentionally corrupt CRC')
        print(f'Received: {len(received)} valid heartbeats; bad CRC={rejected}; sequence gaps={gaps}')
        print(f'Last heartbeat: {received[-1][1]} (state, reserved, three unavailable-current markers)')
        print('Check STM32: received=26, rejected=1; values 1234/5678/90/5/0; link timeout after silence.')
        if rejected or gaps:
            raise RuntimeError('Heartbeat transport errors observed')
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original)
        os.close(fd)


if __name__ == '__main__':
    main()
