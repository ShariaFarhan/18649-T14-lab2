"""CAN-level CTRL/BLINK integration test. Use only the staged no-actuator firmware.
Runs the existing cockpit on loopback UDP with synthetic wheel data, then stops it.
Does not measure physical LED edges or prove timing deadlines.
"""
import json
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import time

root=Path(__file__).resolve().parent
can_frame=struct.Struct('=IB3x8s')
socks={}
for name in ('can0','can1'):
    s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    s.bind((name,));s.setblocking(False);socks[s]=name
udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
stages=[
    ('neutral',2.0,0,0,None),
    ('left_press',.10,0,1<<9,None),
    ('left_blink',1.10,0,0,None),
    ('right_press',.10,0,1<<8,None),
    ('right_blink',1.10,0,0,None),
    ('turn_right',.15,32767,0,None),
    ('cancel',.30,0,0,None),
    ('bad_input',.35,0,0,'b'),
    ('restore',.65,0,0,'b'),
    ('udp_pause',.35,0,0,'pause'),
    ('resume',.65,0,0,None),
]
report={};counter=0
log=(root/'ctrl-blink-test.log').open('wb')
proc=subprocess.Popen([str(root/'cockpit-control'),'--ctrl-blink-only',
                       '--bind','127.0.0.1','--pc-ip','127.0.0.1'],
                      stdin=subprocess.PIPE,stdout=log,stderr=subprocess.STDOUT)
try:
    time.sleep(.2)
    if proc.poll() is not None:raise RuntimeError('Cockpit did not start; see ctrl-blink-test.log')
    for label,duration,steer,buttons,action in stages:
        if action=='b':proc.stdin.write(b'b\n');proc.stdin.flush()
        end=time.monotonic()+duration;next_send=time.monotonic()
        result={'normal_nodes':set(),'reasons':set(),'blink':set(),'fault_sources':set(),
                'CTRL_A':0,'CTRL_B':0,'drive_enable_seen':False,'last_blink':None}
        while time.monotonic()<end:
            now=time.monotonic()
            if action!='pause' and now>=next_send:
                packet=bytearray(276);struct.pack_into('<I',packet,0,counter);counter+=1
                struct.pack_into('<i',packet,4,steer)
                for bit in range(32):
                    if buttons&(1<<bit):packet[52+bit]=0x80
                udp.sendto(packet,('127.0.0.1',8000))
                next_send=now+.020
            ready,_,_=select.select(list(socks),[],[],.002)
            for s in ready:
                ident,dlc,d=can_frame.unpack(s.recv(16))
                if dlc!=8:continue
                if ident==0x010:
                    result['CTRL_A' if socks[s]=='can0' else 'CTRL_B']+=1
                    result['drive_enable_seen'] |= bool(d[0]&0x80)
                elif ident in (0x101,0x102):
                    if d[1]==0:result['normal_nodes'].add(ident-0x100+1)
                    result['reasons'].add(d[3])
                elif ident==0x200:
                    result['blink'].add((d[1],d[2]));result['last_blink']=(d[1],d[2])
                elif ident==0x020 and d[2]==2:result['fault_sources'].add(d[1])
        report[label]={k:sorted(v) if isinstance(v,set) else v for k,v in result.items()}
finally:
    proc.send_signal(signal.SIGTERM)
    try:proc.wait(timeout=3)
    except subprocess.TimeoutExpired:proc.kill();proc.wait()
    log.close();udp.close()
    for s in socks:s.close()
checks={
    'both_nodes_normal':report['neutral']['normal_nodes']==[2,3],
    'left_phase_on_off':all((1,p) in report['left_blink']['blink'] for p in (0,1)),
    'right_phase_on_off':all((2,p) in report['right_blink']['blink'] for p in (0,1)),
    'auto_cancel':report['cancel']['last_blink']==(0,0),
    'bad_input_faults_both_nodes':report['bad_input']['fault_sources']==[2,3],
    'restore_normal':report['restore']['normal_nodes']==[2,3],
    'lost_udp_hazards':any(p[0]==3 for p in report['udp_pause']['blink']),
    'resume_normal':report['resume']['normal_nodes']==[2,3],
    'no_motor_enable':not any(v['drive_enable_seen'] for v in report.values()),
}
output={'checks':checks,'stages':report}
(root/'ctrl-blink-test.json').write_text(json.dumps(output,indent=2))
print(json.dumps(checks,indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
