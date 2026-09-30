"""Native Windows wheel data in the handout's 276-byte UDP envelope."""
import struct
WHEEL_STATE = struct.Struct('<6i2i4I128s24i')
assert WHEEL_STATE.size == 272

def crc(data):
    value=0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8): value=(value>>1) ^ (0x8408 if value&1 else 0)
    return value

def uart_frame(seq, steering, throttle, brake, buttons=0, stale=False):
    payload=struct.pack('<iiiHB',steering,throttle,brake,buttons,1 if stale else 0)
    body=bytes((15,seq&255))+payload
    return b'\x7e\xa5'+body+struct.pack('<H',crc(body))

def wheel_packet(counter, raw):
    buttons=bytes(0x80 if raw['buttons'] & (1<<i) else 0 for i in range(32))+bytes(96)
    state=WHEEL_STATE.pack(raw['x'],raw['z'],0,0,0,raw['r'],0,0,
                           *([0xFFFFFFFF]*4),buttons,*([0]*24))
    return struct.pack('<I',counter&0xFFFFFFFF)+state

def unpack_wheel(data):
    if len(data)!=276:raise ValueError('Expected exactly 276 bytes')
    seq=struct.unpack_from('<I',data)[0]
    values=WHEEL_STATE.unpack_from(data,4)
    buttons=sum(1<<i for i,v in enumerate(values[12][:16]) if v&0x80)
    return seq,values[0],values[1],values[5],buttons
