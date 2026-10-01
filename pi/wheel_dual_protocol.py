"""Dual-motor test protocol: distinct enable bit, UDP trailer and heartbeat marker."""
import struct
from wheel_protocol import crc,wheel_packet,unpack_wheel

def uart_frame(seq,steering,throttle,brake,buttons=0,stale=False,drive_enable=False,test_counter=0,test_sync=False):
    flags=(1 if stale else 0)|(8 if drive_enable and not stale else 0)
    if not 0<=test_counter<=31:raise ValueError('Invalid test counter')
    payload=struct.pack('<iiiHBBB',steering,throttle,brake,buttons,flags,test_counter,bool(test_sync))
    body=bytes((17,seq&255))+payload
    return b'\x7e\xa5'+body+struct.pack('<H',crc(body))

def drive_packet(counter,raw,drive_enable=False,test_counter=0,test_sync=False):
    if not 0<=test_counter<=31:raise ValueError('Invalid test counter')
    return wheel_packet(counter,raw)+b'DD02'+bytes([bool(drive_enable),test_counter,bool(test_sync)])

def unpack_drive_packet(data):
    if len(data)!=283 or data[276:280]!=b'DD02' or data[280] not in (0,1) or data[281]>31 or data[282] not in (0,1):
        raise ValueError('Invalid dual-drive trailer')
    return (*unpack_wheel(data[:276]),bool(data[280]),data[281],bool(data[282]))

def unpack_heartbeat(payload):
    if len(payload)!=25:raise ValueError('Expected error-state dual-drive heartbeat')
    zone,marker,raw,packed,pulse,target,left,right,duty,flags,ack,system=struct.unpack('<BBHHHiiiHBBB',payload)
    if marker!=0xB9 or ack>31 or system>5:raise ValueError('Not error-state dual-drive firmware')
    reasons=['正常','指令超时','左编码器无效','右编码器无效','左轮计数反向','右轮计数反向',
             '左轮持续未转','右轮持续未转','驱动输出失败','输入无效','控制循环超时']
    reason=flags>>4
    systems=['NORMAL','STARTUP','LINK_LOST','BAD_INPUT','SELF_TEST','OUTPUT_FAULT']
    return dict(zone=zone,raw=raw,percent=(packed&255)-100,pulse_us=pulse,
                light_mode=(packed>>8)&3,light_pins=(packed>>10)&15,dual_drive_available=True,
                target_rpm=target/1000,left_rpm=left/1000,right_rpm=right/1000,
                drive_rpm=(left+right)/2000,drive_pwm=duty,
                drive_armed=bool(flags&1),drive_brake=bool(flags&2),drive_fault=bool(flags&4),
                drive_boost=bool(flags&8),fault_reason=reasons[reason] if reason<len(reasons) else '未知故障',
                test_counter_ack=ack,system_reason=system,system_state=systems[system])
