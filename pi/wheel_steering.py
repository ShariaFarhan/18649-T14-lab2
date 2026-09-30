"""Wheel-only steering demonstration. No Logitech SDK, force effects or motor commands."""
import argparse,json,os,select,socket,struct,threading,time
from pathlib import Path
from wheel_protocol import crc,uart_frame,wheel_packet,unpack_wheel


def relay(args):
    import fcntl,termios,tty
    from test_points import TestPoints
    fd=os.open(args.uart,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
    fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
    old=termios.tcgetattr(fd);points=TestPoints()
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind((args.bind,8000));udp.setblocking(False)
    payload=(33151,65535,65535,0);peer=None;last_rx=last_hb=0.;last_seq=None
    serial_seq=0;pending=bytearray();status=None;next_send=time.monotonic()
    try:
        tty.setraw(fd);a=termios.tcgetattr(fd);a[4]=a[5]=termios.B115200
        a[2]=(a[2]|termios.CLOCAL|termios.CREAD)&~termios.CRTSCTS
        termios.tcsetattr(fd,termios.TCSANOW,a);termios.tcflush(fd,termios.TCIOFLUSH)
        print(f'Wheel relay: UDP/8000 from {args.pc_ip} -> {args.uart}; motors disabled in steering firmware',flush=True)
        while True:
            now=time.monotonic()
            ready,_,_=select.select([fd,udp],[],[],max(0,next_send-now))
            if udp in ready:
                data,address=udp.recvfrom(2048)
                if address[0]==args.pc_ip:
                    points.toggle('udp_rx')
                    try:
                        seq,*raw=unpack_wheel(data)
                        delta=(seq-last_seq)&0xFFFFFFFF if last_seq is not None else 1
                        if last_seq is None or time.monotonic()-last_rx>0.12 or 0<delta<0x80000000:
                            payload=tuple(raw);last_seq=seq;peer=address;last_rx=time.monotonic()
                            next_send=min(next_send,last_rx)
                    except ValueError:pass
            if fd in ready:
                pending.extend(os.read(fd,4096))
                while len(pending)>=4:
                    if pending[:2]!=b'\x7e\xa5' or pending[2]>32:
                        del pending[0];continue
                    size=pending[2]+6
                    if len(pending)<size:break
                    packet=bytes(pending[:size]);del pending[:size]
                    if size!=14 or crc(packet[2:-2])!=struct.unpack('<H',packet[-2:])[0]:continue
                    zone,marker,raw,percent,pulse=struct.unpack('<BBHHH',packet[4:-2])
                    if marker==0xB4:
                        if status is None:
                            print(f'STM32 steering heartbeat: state={zone}, PWM={pulse} us',flush=True)
                        status=dict(zone=zone,raw=raw,percent=percent-100,pulse_us=pulse)
                        last_hb=time.monotonic()
            now=time.monotonic()
            if now>=next_send:
                stale=now-last_rx>0.12 or now-last_hb>0.15
                frame=uart_frame(serial_seq,*payload,stale=stale)
                points.toggle('cmd_tx')
                if os.write(fd,frame)!=len(frame):raise RuntimeError('Short UART write')
                serial_seq=(serial_seq+1)&255;next_send=now+.02
                if peer:
                    udp.sendto(json.dumps(dict(stm=status if now-last_hb<=.15 else None,
                                source_stale=now-last_rx>.12,test_points=points.snapshot())).encode(),peer)
    finally:
        try:
            points.toggle('cmd_tx');os.write(fd,uart_frame(serial_seq,*payload,stale=True));termios.tcdrain(fd)
        finally:
            termios.tcsetattr(fd,termios.TCSANOW,old);os.close(fd);udp.close();points.close()


def pc(args):
    import ctypes
    from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
    class Joy(ctypes.Structure):
        _fields_=[(k,ctypes.c_uint) for k in ('size','flags','x','y','z','r','u','v','buttons','number','pov','r1','r2')]
    api=ctypes.WinDLL('winmm');api.joyGetPosEx.argtypes=[ctypes.c_uint,ctypes.POINTER(Joy)]
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.connect((args.pi,8000));udp.setblocking(False)
    lock=threading.Lock();done=threading.Event()
    state=dict(enabled=False,wheel_ok=False,raw=None,stm=None,last_hb=0.,last_browser=0.,sent=0)
    page=Path(__file__).with_name('wheel_steering.html').read_text(encoding='utf-8')
    def worker():
        counter=0
        while not done.is_set():
            j=Joy();j.size=ctypes.sizeof(j);j.flags=255
            rc=api.joyGetPosEx(args.device,ctypes.byref(j))
            raw={k:int(getattr(j,k)) for k in ('x','z','r','buttons')}
            ok=rc==0 and all(0<=raw[k]<=65535 for k in ('x','z','r'))
            with lock:
                state['raw']=raw if ok else None;state['wheel_ok']=ok
                if not ok or time.monotonic()-state['last_browser']>.6:state['enabled']=False
                enabled=state['enabled']
            if enabled:
                try:
                    udp.send(wheel_packet(counter,raw));counter=(counter+1)&0xFFFFFFFF
                    with lock:state['sent']+=1
                except OSError:pass
            try:
                while select.select([udp],[],[],0)[0]:
                    reply=json.loads(udp.recv(2048))
                    with lock:state['stm']=reply.get('stm');state['last_hb']=time.monotonic()
            except (OSError,ValueError,AttributeError):pass
            done.wait(.02)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def reply(self,code,value,kind='application/json'):
            b=value.encode() if isinstance(value,str) else json.dumps(value).encode()
            self.send_response(code);self.send_header('Content-Type',kind+'; charset=utf-8')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
        def do_GET(self):
            if self.path=='/':self.reply(200,page,'text/html')
            else:self.reply(404,'Not found','text/plain')
        def do_POST(self):
            host=f'127.0.0.1:{args.web_port}'
            if self.headers.get('Host')!=host or self.headers.get('Origin') not in (None,'http://'+host):
                self.reply(403,'Local page only','text/plain');return
            size=int(self.headers.get('Content-Length','0'))
            if size>1024:self.reply(400,'Too large','text/plain');return
            self.rfile.read(size)
            with lock:
                if self.path=='/start':
                    if not state['wheel_ok']:self.reply(409,'方向盘未连接','text/plain');return
                    state['enabled']=True;state['last_browser']=time.monotonic()
                elif self.path=='/stop':state['enabled']=False
                elif self.path=='/status':state['last_browser']=time.monotonic()
                else:self.reply(404,'Not found','text/plain');return
                result=dict(state);result['pi_online']=time.monotonic()-state['last_hb']<.4
                if not result['pi_online']:result['stm']=None
            self.reply(200,result)
    threading.Thread(target=worker,daemon=True).start()
    server=ThreadingHTTPServer(('127.0.0.1',args.web_port),Handler)
    print(f'Wheel read-only until Start: http://127.0.0.1:{args.web_port}',flush=True)
    try:server.serve_forever()
    finally:done.set();server.server_close();udp.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='role',required=True)
    a=sub.add_parser('pi');a.add_argument('--uart',default='/dev/serial0');a.add_argument('--bind',default='0.0.0.0');a.add_argument('--pc-ip',required=True)
    a=sub.add_parser('pc');a.add_argument('--pi',required=True);a.add_argument('--device',type=int,default=0);a.add_argument('--web-port',type=int,default=8766)
    args=p.parse_args();(relay if args.role=='pi' else pc)(args)
