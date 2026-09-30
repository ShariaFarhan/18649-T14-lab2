"""PC browser -> UDP/8001 -> Pi GPIO UART -> STM32 motor/steering/lighting bench firmware.

Separate from the Logitech UDP/8000 protocol. PC uses the standard library;
Pi additionally uses the installed libgpiod v2 Python bindings for scope markers.
Run on each host: voltage_bench.py pc|pi --config <private JSON configuration>
"""
import argparse
import hmac
import http.server
import json
import os
import select
import socket
import struct
import threading
import time
import uuid
from pathlib import Path


def crc(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value


def packet(seq, throttle, brake, stale=False, steering=0, lights=0, servo=False, light_seq=0, test_counter=0, sync=False):
    buttons = lights | (4 if servo else 0) | ((test_counter & 31) << 3) | (light_seq << 8)
    payload = struct.pack('<hhhHB', steering, throttle, brake, buttons, 0x80 | (1 if stale else 0) | (2 if sync else 0))
    body = bytes((len(payload), seq & 255)) + payload
    return b'\x7e\xa5' + body + struct.pack('<H', crc(body))


def percentages(message):
    values = message['throttle'], message['brake']
    if any(type(v) is not int or not 0 <= v <= 100 for v in values):
        raise ValueError('Use whole percentages from 0 to 100')
    return values


CONTROL_FIELDS = ('throttle', 'brake', 'steering', 'lights', 'servo', 'light_seq')
SAFE_COMMAND = (0, 100, 0, 0, False, 0)


def controls(message):
    throttle, brake = percentages(message)
    steering = message.get('steering', 0)
    lights = message.get('lights', 0)
    servo = message.get('servo', False)
    event = message.get('light_seq', 0)
    if type(steering) is not int or not -100 <= steering <= 100:
        raise ValueError('Steering must be an integer from -100 to 100')
    if type(lights) is not int or not 0 <= lights <= 3 or type(servo) is not bool:
        raise ValueError('Invalid light mode or servo enable')
    if type(event) is not int or not 0 <= event <= 255:
        raise ValueError('Invalid light event')
    return throttle, brake, steering, lights, servo, event


def pi(config):
    import termios
    import tty
    import fcntl
    from test_points import TestPoints
    fd = os.open(config.get('uart', '/dev/ttyAMA0'), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    old = termios.tcgetattr(fd)
    markers = TestPoints()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('0.0.0.0', config['udp_port']))
    udp.setblocking(False)
    seq = 0
    command = SAFE_COMMAND
    test_counter, sync_button, source_ready, inject_bad = 0, False, False, False
    peer = None
    session = None
    last_seq = -1
    last_pc = last_stm = 0.0
    hb = None
    buffer = bytearray()
    next_send = time.monotonic()
    try:
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200
        attrs[2] = (attrs[2] | termios.CLOCAL | termios.CREAD) & ~termios.CRTSCTS
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        print('Pi voltage bench ready: UDP -> GPIO UART; startup brake', flush=True)
        while True:
            now = time.monotonic()
            readable, _, _ = select.select([udp, fd], [], [], max(0, next_send-now))
            if udp in readable:
                data, address = udp.recvfrom(2048)
                markers.toggle('udp_rx') # Reception edge, before decoding the packet.
                try:
                    msg = json.loads(data)
                    if not isinstance(msg, dict) or not isinstance(msg.get('key'), str):
                        continue
                    if not hmac.compare_digest(msg['key'], config['key']):
                        continue
                    values = controls(msg)
                    counter = msg.get('test_counter', 0)
                    sync = msg.get('sync', False)
                    ready = msg.get('source_ready', False)
                    bad = msg.get('bad_input', False)
                    if type(counter) is not int or not 0 <= counter <= 31 or any(type(v) is not bool for v in (sync, ready, bad)):
                        continue
                    incoming_seq = msg['seq']
                    incoming_session = msg['session']
                    if type(incoming_seq) is not int or incoming_seq < 0 or not isinstance(incoming_session, str):
                        continue
                    if session == incoming_session:
                        if incoming_seq <= last_seq:
                            continue
                    elif session is not None and time.monotonic()-last_pc < 0.2:
                        continue
                    command, peer, session, last_seq = values, address, incoming_session, incoming_seq
                    test_counter, sync_button, source_ready, inject_bad = counter, sync, ready, bad
                    last_pc = time.monotonic()
                    next_send = min(next_send, last_pc) # Send each accepted UDP update immediately.
                except (ValueError, KeyError, TypeError):
                    pass
            if fd in readable:
                buffer.extend(os.read(fd, 4096))
                while len(buffer) >= 4:
                    if buffer[:2] != b'\x7e\xa5' or buffer[2] > 32:
                        del buffer[0]
                        continue
                    size = buffer[2] + 6
                    if len(buffer) < size:
                        break
                    frame = bytes(buffer[:size])
                    del buffer[:size]
                    if size != 14 or crc(frame[2:-2]) != struct.unpack('<H', frame[-2:])[0]:
                        continue
                    state, marker, duty, bits, pulse = struct.unpack('<BBHHH', frame[4:-2])
                    if marker == 0xB3:
                        hb = dict(state=state, duty=duty & 255, test_ack=(duty >> 8) & 31,
                                  reason=(bits >> 12) & 15, self_test=bool(bits & 32), braking=bool(bits & 1),
                                  lights=(bits >> 1) & 3, servo=bool(bits & 8),
                                  crossed=bool(bits & 16), led_bits=(bits >> 8) & 15, servo_us=pulse)
                        last_stm = time.monotonic()
            now = time.monotonic()
            if now >= next_send:
                stale = not source_ready or now-last_pc > 0.12 or now-last_stm > 0.15
                throttle, brake, steering, lights, servo, event = SAFE_COMMAND if stale else command
                if inject_bad and not stale: throttle = 101 # Explicit out-of-range test for STM32.
                outgoing = packet(seq, throttle, brake, stale, steering, lights, servo, event,
                                  test_counter, sync_button)
                markers.toggle('cmd_tx') # Start of the software UART-send operation.
                if os.write(fd, outgoing) != len(outgoing):
                    raise RuntimeError('Short UART write')
                seq = (seq+1) & 255
                next_send = now + 0.02
                if peer:
                    reply = dict(key=config['key'], stm=hb if now-last_stm <= 0.15 else None,
                                 pc_stale=now-last_pc > 0.12, test_points=markers.snapshot())
                    udp.sendto(json.dumps(reply).encode(), peer)
    finally:
        try:
            markers.toggle('cmd_tx')
            os.write(fd, packet(seq, 0, 100, True))
            termios.tcdrain(fd)
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, old)
            os.close(fd)
            udp.close()
            markers.close()


PAGE = Path(__file__).with_name('actuator_bench.html').read_text(encoding='utf-8')


def pc(config):
    destination = (socket.gethostbyname(config['pi_host']), config['udp_port'])
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.connect(destination)
    udp.setblocking(False)
    lock = threading.Lock()
    state = dict(active=False, command=SAFE_COMMAND, last_browser=0.0, last_pi=0.0, stm=None, test_points=None,
                 source_ready=False, test_counter=0, sync=True, last_test_press=-1.0, bad_input=False)
    done = threading.Event()
    wake = threading.Event()
    session = uuid.uuid4().hex

    def sender():
        seq = 0
        while not done.is_set():
            with lock:
                if time.monotonic()-state['last_browser'] > 0.6:
                    if state['active']: print('Browser heartbeat timed out; brake', flush=True)
                    state['active'] = False
                    state['source_ready'] = False
                command = state['command'] if state['active'] else SAFE_COMMAND
                msg = dict(key=config['key'], session=session, seq=seq,
                           source_ready=state['source_ready'], test_counter=state['test_counter'],
                           sync=state['sync'], bad_input=state['bad_input'], **dict(zip(CONTROL_FIELDS, command)))
            seq += 1
            try:
                udp.send(json.dumps(msg).encode())
                while select.select([udp], [], [], 0)[0]:
                    reply = json.loads(udp.recv(2048))
                    if isinstance(reply.get('key'), str) and hmac.compare_digest(reply['key'], config['key']):
                        with lock:
                            state['stm'] = reply.get('stm')
                            state['test_points'] = reply.get('test_points')
                            state['last_pi'] = time.monotonic()
                            if state['stm'] and state['stm'].get('test_ack') == state['test_counter']:
                                state['sync'] = False
            except (OSError, ValueError, TypeError, AttributeError):
                pass
            wake.wait(0.02)
            wake.clear()

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def answer(self, code, content, kind='application/json'):
            data = content.encode() if isinstance(content, str) else json.dumps(content).encode()
            self.send_response(code)
            self.send_header('Content-Type', kind+'; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                self.answer(200, PAGE, 'text/html')
            else:
                self.answer(404, 'Not found', 'text/plain')

        def do_POST(self):
            expected = f'127.0.0.1:{config.get("web_port",8765)}'
            if self.headers.get('Host') != expected or self.headers.get('Origin') not in (None, 'http://'+expected):
                self.answer(403, 'Local page only', 'text/plain')
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 <= size <= 1024:
                    raise ValueError('Request too large')
                msg = json.loads(self.rfile.read(size) or '{}')
                with lock:
                    if self.path == '/apply':
                        state['command'] = controls(msg)
                        print('Apply:', state['command'], flush=True)
                        state['active'] = True
                        state['source_ready'] = True
                        state['bad_input'] = False
                        state['last_browser'] = time.monotonic()
                    elif self.path == '/selftest':
                        now = time.monotonic()
                        if state['sync'] or not state['stm'] or now-state['last_pi'] >= 0.3:
                            raise ValueError('等待 Pi / STM32 连接并同步后再按自检')
                        if state['last_test_press'] < 0 or now-state['last_test_press'] >= 0.05:
                            state['test_counter'] = (state['test_counter']+1) & 31
                            state['last_test_press'] = now
                        state['source_ready'] = True
                        state['last_browser'] = now
                    elif self.path == '/badinput':
                        state['source_ready'] = True
                        state['bad_input'] = True
                        state['last_browser'] = time.monotonic()
                    elif self.path == '/stop':
                        state['active'] = False
                        state['command'] = SAFE_COMMAND
                        state['bad_input'] = False
                    elif self.path == '/status':
                        state['last_browser'] = time.monotonic()
                    else:
                        raise ValueError('Unknown action')
                    online = time.monotonic()-state['last_pi'] < 0.3
                    wake.set()
                    reply = dict(active=state['active'], pi_online=online, button_ready=not state['sync'] and online,
                                 stm=state['stm'] if online else None, test_points=state['test_points'] if online else None)
                self.answer(200, reply)
            except (ValueError, KeyError, TypeError) as exc:
                self.answer(400, str(exc), 'text/plain')

    threading.Thread(target=sender, daemon=True).start()
    server = http.server.ThreadingHTTPServer(('127.0.0.1', config.get('web_port',8765)), Handler)
    print(f'Control page: http://127.0.0.1:{server.server_port}', flush=True)
    try:
        server.serve_forever()
    finally:
        done.set()
        server.server_close()
        udp.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('role', choices=['pc', 'pi'])
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    configuration = json.loads(Path(args.config).read_text(encoding='utf-8'))
    (pc if args.role == 'pc' else pi)(configuration)
