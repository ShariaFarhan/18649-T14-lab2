"""Read-only Lab5 CAN decoder. Prints REF cycles/time and complete Raft RPCs."""
import argparse,json,select,socket,struct,time,zlib
TYPES={1:'PRE_VOTE',2:'PRE_VOTE_REPLY',3:'REQ_VOTE',4:'VOTE_REPLY',5:'APPEND',6:'APPEND_REPLY',7:'CLIENT',8:'CLIENT_REPLY',10:'TIME_QUERY',11:'TIME_REPLY'}
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=float,default=60);p.add_argument('--output');a=p.parse_args()
    sockets=[];partial={};start=time.monotonic();records=[]
    try:
        for name in ('can0','can1'):
            s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW);s.bind((name,));s.setblocking(False);sockets.append(s)
        while time.monotonic()-start<a.seconds:
            for s in select.select(sockets,[],[],.1)[0]:
                cid,dlc,d=struct.unpack('=IB3x8s',s.recv(16))
                if cid&0xe0000000 or dlc!=8:continue
                row=dict(t=round(time.monotonic()-start,6),bus=f'can{sockets.index(s)}',id=f'0x{cid:03x}')
                if 0x45<=cid<=0x4f:
                    row.update(type='REF',leader=(cid-0x40)>>2,hop=cid&3,cycle=d[0],term_low=d[1],global_us=int.from_bytes(d[2:],'little'))
                elif 0x80<=cid<=0xbf:
                    row.update(type='REF_ACK',source=((cid-0x80)>>4)&3,hop=cid&3,term=int.from_bytes(d[:2],'little'),destination=(cid>>2)&3,reference_time_us=int.from_bytes(d[2:],'little'))
                elif 0xc0<=cid<=0xff:
                    row.update(type='TIME_SAMPLE',source=((cid-0xc0)>>4)&3,hop=cid&3,destination=(cid>>2)&3,nonce=int.from_bytes(d[:4],'little'),global_low=int.from_bytes(d[4:],'little'))
                elif 0x140<=cid<=0x1bf:
                    row.update(type='TIME_QUERY' if cid<0x180 else 'TIME_PROOF',source=(cid>>4)&3,hop=cid&3,destination=(cid>>2)&3,term=int.from_bytes(d[:2],'little'),nonce=int.from_bytes(d[2:6],'little'),flags=d[6])
                elif 0x501<=cid<=0x573 or 0x1c1<=cid<=0x1c3:
                    kind=11 if cid<0x500 else (cid-0x500)//16+1;source=(d[0]>>4)&3;dest=d[0]>>6;fragment=d[0]&15
                    if kind not in TYPES or fragment>=6:continue
                    key=(row['bus'],kind,source,dest,d[1]);parts=partial.setdefault(key,{})
                    parts[fragment]=d[2:]
                    if len(parts)<6:continue
                    data=b''.join(parts[i] for i in range(6));partial.pop(key,None)
                    if zlib.crc32(data[:28])!=int.from_bytes(data[28:32],'little'):continue
                    term,index,prev,commit,event,aux=struct.unpack_from('<6I',data,4)
                    row.update(type=TYPES[kind],source=source,destination=dest,ok=data[3],term=term,index=index,previous_term=prev,commit=commit,event=event,aux=aux)
                else:continue
                print(json.dumps(row),flush=True);records.append(row)
                if len(partial)>1024:partial.clear()
    except KeyboardInterrupt:pass
    finally:
        for s in sockets:s.close()
    if a.output:
        from pathlib import Path
        Path(a.output).write_text(json.dumps(records,indent=2))
if __name__=='__main__':main()
