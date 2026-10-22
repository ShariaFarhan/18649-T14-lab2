"""Read Windows joystick axes; send the cockpit's 276-byte state packet.

Uses the Lab2 WinMM reader without initializing force feedback. Windows raw
calibration on the Pi: --steer 0,33151,65535 --throttle 65535,0 --brake 65535,0.
"""
import argparse
import ctypes
import json
from pathlib import Path
import socket
import struct
import threading
import time


class Joy(ctypes.Structure):
    _fields_ = [(k, ctypes.c_uint) for k in
                ('size', 'flags', 'x', 'y', 'z', 'r', 'u', 'v',
                 'buttons', 'number', 'pov', 'r1', 'r2')]


def packet(seq, joy):
    data = bytearray(276)
    struct.pack_into('<I', data, 0, seq)
    struct.pack_into('<i', data, 4, joy.x)
    struct.pack_into('<i', data, 8, joy.z)
    struct.pack_into('<i', data, 24, joy.r)
    for bit in range(32):
        data[52 + bit] = 128 if joy.buttons & (1 << bit) else 0
    return data

def decode_force(data):
    if len(data)!=1:
        return None
    value=struct.unpack('b',data)[0]
    return value if -100<=value<=100 else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pi', required=True)
    parser.add_argument('--device', type=int, default=0)
    parser.add_argument('--status', required=True)
    parser.add_argument('--force-port',type=int,default=8001,
                        help='Receive dummy FORCE telemetry only; never apply motor effects')
    parser.add_argument('--diagnostic-port', type=int,
                        help='Optional second UDP port for passive arrival-gap measurement')
    args = parser.parse_args()
    api = ctypes.WinDLL('winmm')
    api.joyGetPosEx.argtypes = [ctypes.c_uint, ctypes.POINTER(Joy)]
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.connect((args.pi, 8000))
    expected_pi=socket.gethostbyname(args.pi)
    force_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    force_socket.bind(('0.0.0.0',args.force_port));force_socket.settimeout(.2)
    force_state=dict(value=None,count=0,received_at=None)
    def receive_force():
        while True:
            try:
                data,address=force_socket.recvfrom(128)
                value=decode_force(data)
                if address[0]==expected_pi and value is not None:
                    force_state.update(value=value,count=force_state['count']+1,received_at=time.time())
            except socket.timeout:
                continue
    threading.Thread(target=receive_force,daemon=True).start()
    diagnostic = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.diagnostic_port else None
    seq = 0
    next_tick = time.monotonic()
    latest = {}
    # Disk/antivirus latency must not block the 20 ms acquisition/send loop.
    def status_writer():
        while True:
            snapshot = latest.copy()
            if snapshot:
                try:
                    Path(args.status).write_text(json.dumps(snapshot), encoding='utf-8')
                except OSError:
                    pass
            time.sleep(.5)
    threading.Thread(target=status_writer, daemon=True).start()
    last_send = None
    max_gap = max_read = max_send = 0.
    long_gaps = read_errors = 0
    ready=False
    while True:
        joy = Joy()
        joy.size, joy.flags = ctypes.sizeof(joy), 255
        read_start = time.monotonic()
        rc = api.joyGetPosEx(args.device, ctypes.byref(joy))
        max_read = max(max_read, (time.monotonic()-read_start)*1000)
        error = None
        if rc!=0:
            ready=False
        elif any(getattr(joy,k)!=32767 for k in ('x','z','r')) or joy.buttons:
            ready=True
        if rc == 0 and ready:
            try:
                send_start = time.monotonic()
                data = packet(seq, joy)
                udp.send(data)
                sent_at = time.monotonic()
                max_send = max(max_send, (sent_at-send_start)*1000)
                if last_send is not None:
                    gap = (sent_at-last_send)*1000
                    max_gap = max(max_gap, gap)
                    if gap >= 60:
                        long_gaps += 1
                        print(f'SEND_GAP seq={seq} gap_ms={gap:.3f}', flush=True)
                last_send = sent_at
                if diagnostic:
                    try:
                        diagnostic.sendto(data, (args.pi, args.diagnostic_port))
                    except OSError:
                        pass
                seq = (seq + 1) & 0xffffffff
            except OSError as exc:
                error = str(exc)
        elif rc!=0:
            read_errors += 1
        now = time.monotonic()
        latest = dict(timestamp=time.time(), connected=rc == 0, ready=ready, error=error,
            sent=seq, source=udp.getsockname(), max_gap_ms=round(max_gap,3),
            max_read_ms=round(max_read,3), max_send_ms=round(max_send,3),
            gaps_over_60ms=long_gaps, read_errors=read_errors,
            force=dict(force_state),
            raw={k: getattr(joy, k) for k in ('x', 'z', 'r', 'buttons')})
        next_tick += .02
        if next_tick < now:
            next_tick += (int((now-next_tick)/.02)+1)*.02
        time.sleep(max(0, next_tick - time.monotonic()))


if __name__ == '__main__':
    main()
