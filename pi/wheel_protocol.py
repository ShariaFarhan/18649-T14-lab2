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

def uart_frame(seq, steering, throttle, brake, buttons=0, stale=False, right_enable=False):
    flags=(1 if stale else 0)|(2 if right_enable and not stale else 0)
    payload=struct.pack('<iiiHB',steering,throttle,brake,buttons,flags)
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

def unpack_heartbeat(payload):
    if len(payload) not in (8,19):raise ValueError('Invalid test heartbeat length')
    zone,marker,raw,packed,pulse=struct.unpack_from('<BBHHH',payload)
    if marker not in (0xB4,0xB5,0xB6) or (len(payload)==19)!=(marker==0xB6):
        raise ValueError('Not a compatible test heartbeat')
    result=dict(zone=zone,raw=raw,percent=(packed&255)-100,pulse_us=pulse,
                light_mode=(packed>>8)&3 if marker!=0xB4 else None,
                light_pins=(packed>>10)&15 if marker!=0xB4 else None,right_drive_available=marker==0xB6)
    if marker==0xB6:
        target,actual,duty,flags=struct.unpack_from('<iiHB',payload,8)
        result.update(target_rpm=target/1000,right_rpm=actual/1000,right_pwm=duty,
                      right_armed=bool(flags&1),right_brake=bool(flags&2),
                      right_fault=bool(flags&4),right_boost=bool(flags&8))
    return result

def drive_packet(counter, raw, right_enable=False):
    # Test-only trailer; the first 276 bytes retain the original wheel layout.
    return wheel_packet(counter,raw)+b'RD01'+bytes([bool(right_enable)])

def unpack_drive_packet(data):
    if len(data)==276:return (*unpack_wheel(data),False)
    if len(data)!=281 or data[276:280]!=b'RD01' or data[280] not in (0,1):
        raise ValueError('Invalid right-drive test trailer')
    return (*unpack_wheel(data[:276]),bool(data[280]))
