"""Validate the common Lab5 config and emit the same embedded config for every node."""
import argparse,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def generate(c):
    nodes=c['nodes']
    if c['N']!=3 or nodes!={'cockpit':1,'steering':2,'drivetrain':3}:
        raise ValueError('This hardware port implements three fixed voters: cockpit=1, steering=2, drivetrain=3')
    leader=c['default_leader']
    if leader in (None,'none','random'):leader=0
    elif leader in nodes:leader=nodes[leader]
    else:raise ValueError('default_leader must be a configured name or none/random')
    ranges={'heartbeat_ms':(1,1000),'missed_heartbeats':(2,20),'election_timeout_jitter_ms':(1,1000),
            'log_size':(1,30000),'append_heartbeat_ms':(1,1000),'event_period_ms':(20,60000),
            'network_delay_us':(0,100000),'random_seed':(1,0xffffffff)}
    for key,(low,high) in ranges.items():
        if type(c[key]) is not int or not low<=c[key]<=high:raise ValueError(f'Invalid {key}')
    if type(c['Debug']) is not bool:raise ValueError('Debug must be boolean')
    grace=c.get('startup_grace_ms',500)
    if type(grace) is not int or not 0<=grace<=10000:raise ValueError('Invalid startup_grace_ms')
    matrix=c.get('path_delay_us',[[0 if i==j else c['network_delay_us'] for j in range(3)] for i in range(3)])
    if len(matrix)!=3 or any(len(row)!=3 for row in matrix) or any(type(x) is not int or not 0<=x<=100000 for row in matrix for x in row):
        raise ValueError('path_delay_us must be a 3x3 nonnegative microsecond matrix')
    v=[3,leader,c['heartbeat_ms'],c['missed_heartbeats'],c['election_timeout_jitter_ms'],c['log_size'],
       c['append_heartbeat_ms'],c['event_period_ms'],c['network_delay_us'],c['random_seed'],int(c['Debug'])]
    rows=[[0]*4]+[[0]+row for row in matrix]
    delays='{'+','.join('{'+','.join(map(str,row))+'}' for row in rows)+'}'
    return '/* Generated from lab5/config.json; rebuild all nodes after changing it. */\n#define L5_DEFAULT_CONFIG { '+', '.join(map(str,v))+', '+delays+', '+str(grace)+' }\n'
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--check',action='store_true');a=p.parse_args()
    expected=generate(json.loads((ROOT/'lab5/config.json').read_text()))
    path=ROOT/'lab5/config_generated.h'
    if a.check:
        if not path.exists() or path.read_text()!=expected:raise SystemExit('Run python3 tools/lab5_config.py and rebuild all nodes')
    else:path.write_text(expected,encoding='utf-8')
if __name__=='__main__':main()
