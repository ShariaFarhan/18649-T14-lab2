"""Dual-motor test protocol: distinct enable bit, UDP trailer and heartbeat marker."""
import struct
from wheel_protocol import crc,wheel_packet,unpack_wheel

def uart_frame(seq,steering,throttle,brake,buttons=0,stale=False,drive_enable=False):
    flags=(1 if stale else 0)|(8 if drive_enable and not stale else 0)
    payload=struct.pack('<iiiHB',steering,throttle,brake,buttons,flags)
    body=bytes((15,seq&255))+payload
    return b'\x7e\xa5'+body+struct.pack('<H',crc(body))

def drive_packet(counter,raw,drive_enable=False):
    return wheel_packet(counter,raw)+b'DD01'+bytes([bool(drive_enable)])

def unpack_drive_packet(data):
    if len(data)==276:return (*unpack_wheel(data),False)
    if len(data)!=281 or data[276:280]!=b'DD01' or data[280] not in (0,1):
        raise ValueError('Invalid dual-drive trailer')
    return (*unpack_wheel(data[:276]),bool(data[280]))

def unpack_heartbeat(payload):
    if len(payload)!=23:raise ValueError('Expected dual-drive heartbeat')
    zone,marker,raw,packed,pulse,target,left,right,duty,flags=struct.unpack('<BBHHHiiiHB',payload)
    if marker!=0xB8:raise ValueError('Not dual-drive firmware')
    reasons=['正常','指令超时','左编码器无效','右编码器无效','左轮计数反向','右轮计数反向',
             '左轮持续未转','右轮持续未转','驱动输出失败','输入无效','控制循环超时']
    reason=flags>>4
    return dict(zone=zone,raw=raw,percent=(packed&255)-100,pulse_us=pulse,
                light_mode=(packed>>8)&3,light_pins=(packed>>10)&15,dual_drive_available=True,
                target_rpm=target/1000,left_rpm=left/1000,right_rpm=right/1000,
                drive_rpm=(left+right)/2000,drive_pwm=duty,
                drive_armed=bool(flags&1),drive_brake=bool(flags&2),drive_fault=bool(flags&4),
                drive_boost=bool(flags&8),fault_reason=reasons[reason] if reason<len(reasons) else '未知故障')
