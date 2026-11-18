"""Render the Pi's committed EVENT view keyed by log index (no duplicate rows)."""
import argparse,json,time
from pathlib import Path
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('log');p.add_argument('--once',action='store_true');a=p.parse_args()
    root=Path(a.log+'.events')
    try:
        while True:
            rows={}
            for file in sorted(root.glob('*.json')):
                try:
                    row=json.loads(file.read_text());rows[row['index']]=row
                except (OSError,ValueError,KeyError):continue
            if not a.once:print('\033[2J\033[H',end='')
            print(f'Committed EVENT display: {len(rows)} unique entries')
            print('index      term    source   counter   status')
            for index in sorted(rows):
                r=rows[index];print(f"{index:<10} {r['term']:<7} {r['source']:<8} {r['counter']:<9} {'replayed' if r['replayed'] else 'live'}")
            if a.once:break
            time.sleep(.5)
    except KeyboardInterrupt:pass
if __name__=='__main__':main()
