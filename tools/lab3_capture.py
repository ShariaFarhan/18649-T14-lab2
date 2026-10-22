"""Read-only SocketCAN capture. Arrival intervals are NOT end-to-end latency."""
import argparse
import collections
import json
import select
import socket
import struct
import time
from pathlib import Path

def percentile(values,p):
    return sorted(values)[min(len(values)-1,int((len(values)-1)*p))] if values else None

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--output',required=True)
    a=p.parse_args()
    if a.seconds<=0:p.error('duration must be positive')
    sockets=[];records=collections.defaultdict(lambda:dict(raw=0,repeat=0,old=0,seq=None,last=None,gaps=[]))
    events=[];state={};start=time.monotonic()
    try:
        for interface in ('can0','can1'):
            s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            s.setsockopt(socket.SOL_SOCKET,35,1) # Linux SO_TIMESTAMPNS (old timespec ABI)
            s.bind((interface,));s.setblocking(False);sockets.append(s)
        while time.monotonic()-start<a.seconds:
            for s in select.select(sockets,[],[],.1)[0]:
                frame,anc,_,_=s.recvmsg(16,128)
                if len(frame)!=16:continue
                cid,dlc,d=struct.unpack('=IB3x8s',frame)
                if cid & 0xe0000000 or dlc!=8:continue
                stamp=time.time()
                for level,kind,b in anc:
                    if level==socket.SOL_SOCKET and kind==35:
                        sec,ns=struct.unpack('ll',b);stamp=sec+ns/1e9
                key=f'can{sockets.index(s)}:0x{cid:03x}';r=records[key];r['raw']+=1
                seq=d[0]&15;delta=None if r['seq'] is None else (seq-r['seq'])&15
                if delta==0:r['repeat']+=1;continue
                if delta is not None and delta>=8:r['old']+=1;continue
                if r['last'] is not None:r['gaps'].append((stamp-r['last'])*1000)
                r['last']=stamp;r['seq']=seq
                value=list(d[1:4]) if cid in (0x100,0x101,0x102) else list(d[1:3]) if cid in (0x20,0x200) else None
                if value is not None and state.get(key)!=value:
                    state[key]=value;events.append(dict(t=round(time.monotonic()-start,6),stream=key,value=value))
    finally:
        for s in sockets:s.close()
    result=dict(duration_s=time.monotonic()-start,
                note='Kernel receive intervals; no GPIO or end-to-end latency claim.',events=events,streams={})
    for k,r in records.items():
        g=r.pop('gaps');r.pop('seq');r.pop('last')
        r.update(max_interval_ms=max(g) if g else None,p99_interval_ms=percentile(g,.99))
        result['streams'][k]=r
    Path(a.output).write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result['streams'],indent=2))

if __name__=='__main__':main()
