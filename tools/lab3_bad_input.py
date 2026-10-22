"""On Pi: inject out-of-range CTRL through an already running cockpit.
Requires cockpit --control-socket PATH. Restores valid input on Ctrl-C/exit.
"""
import argparse
import socket
import time

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--socket',required=True)
    p.add_argument('--seconds',type=float,default=1.)
    a=p.parse_args()
    if not 0<a.seconds<=10:p.error('--seconds must be >0 and <=10')
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:
        s.connect(a.socket)
        try:
            s.send(b'bad-input on')
            print('Out-of-range steering enabled; expect FAULT and ERR. Control transport remains live.')
            time.sleep(a.seconds)
        finally:
            s.send(b'bad-input off')
            print('Valid steering restored; check automatic recovery.')

if __name__=='__main__':main()
