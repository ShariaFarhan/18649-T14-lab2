"""Passive SocketCAN capture for stage 1; run on the Pi, no injected frames."""
import argparse
import collections
import json
import select
import socket
import struct
import time

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=30)
args = p.parse_args()
can_frame = struct.Struct('=IB3x8s')
sockets, records = {}, collections.defaultdict(list)
pending, matched, mismatched = {}, collections.Counter(), collections.Counter()
for name in ('can0', 'can1'):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_SOCKET, 35, 1)  # SO_TIMESTAMPNS, kernel receive time on this Pi.
    s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                 b''.join(struct.pack('=II', ident, 0xC00007FF) for ident in (0x100,0x101,0x102)))
    s.bind((name,)); s.setblocking(False); sockets[s] = name
end = time.monotonic() + args.seconds
while time.monotonic() < end:
    ready, _, _ = select.select(list(sockets), [], [], min(.1, max(0,end-time.monotonic())))
    for s in ready:
        raw, ancillary, _, _ = s.recvmsg(16, 128)
        ident, dlc, data = can_frame.unpack(raw)
        if dlc != 8: continue
        stamp = None
        for level, kind, value in ancillary:
            if level == socket.SOL_SOCKET and kind == 35:
                sec, ns = struct.unpack('@ll', value[:struct.calcsize('@ll')]); stamp=sec+ns/1e9
        if stamp is None: raise RuntimeError('Kernel timestamp missing')
        name = sockets[s]
        records[(name,ident)].append((stamp,data))
        key=(ident,data[0]&15)
        previous=pending.get(key)
        if previous and previous[0] != name and abs(stamp-previous[1]) < .05:
            (matched if previous[2] == data else mismatched)[ident] += 1
            del pending[key]
        else: pending[key]=(name,stamp,data)
for s in sockets:s.close()
report={'seconds':args.seconds,'streams':{},'matched_AB':{},'mismatched_AB':{}}
for (name,ident),items in sorted(records.items()):
    intervals=sorted((items[i][0]-items[i-1][0])*1000 for i in range(1,len(items)))
    elapsed=items[-1][0]-items[0][0]
    applied0=int.from_bytes(items[0][1][6:8],'little')
    applied1=int.from_bytes(items[-1][1][6:8],'little')
    report['streams'][f'{name}:0x{ident:03x}']={
        'frames':len(items),
        'period_mean_ms':round(sum(intervals)/len(intervals),3) if intervals else None,
        'period_min_ms':round(min(intervals),3) if intervals else None,
        'period_max_ms':round(max(intervals),3) if intervals else None,
        'period_p99_ms':round(intervals[int(.99*(len(intervals)-1))],3) if intervals else None,
        'logical_peer_HB_per_second':round(((applied1-applied0)&65535)/elapsed,2) if elapsed else None,
        'sequence_discontinuities':sum(((items[i][1][0]-items[i-1][1][0])&15)!=1 for i in range(1,len(items))),
        'nonzero_bus_health_frames':sum(data[2]!=0 for _,data in items),
    }
for ident in (0x100,0x101,0x102):
    report['matched_AB'][f'0x{ident:03x}']=matched[ident]
    report['mismatched_AB'][f'0x{ident:03x}']=mismatched[ident]
print(json.dumps(report,indent=2))
ok=len(records)==6 and not sum(mismatched.values())
ok=ok and all(v['frames']>=args.seconds*40 and v['sequence_discontinuities']==0 and
             90<v['logical_peer_HB_per_second']<110 for v in report['streams'].values())
raise SystemExit(0 if ok else 1)
