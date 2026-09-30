"""Live bench state checks. Run only with motors/servo disconnected.

Uses the same local HTTP endpoints as the page, through real UDP/Pi/UART/STM32.
"""
import argparse
import json
import time
import urllib.request


def call(path, values=None):
    request = urllib.request.Request('http://127.0.0.1:8765'+path,
        data=json.dumps(values or {}).encode(), headers={'Content-Type':'application/json'})
    return json.load(urllib.request.urlopen(request, timeout=2))


def wait(reason, duty=None):
    deadline = time.monotonic()+2
    while time.monotonic()<deadline:
        h = call('/status').get('stm')
        if h and h['reason']==reason and (duty is None or h['duty']==duty):
            if reason:
                assert h['braking'] and h['lights']==3, h
            return h
        time.sleep(.02)
    raise AssertionError(f'Expected reason {reason}, last status {h}')


def apply(throttle=30):
    call('/apply',dict(throttle=throttle,brake=0,steering=-50,
                       servo=True,lights=0,light_seq=1))


def double_press():
    call('/selftest')
    time.sleep(.15)
    call('/selftest')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bare-board',action='store_true',required=True)
    parser.add_argument('--output-fault',action='store_true',help='Start after bench output_fault')
    args=parser.parse_args()
    try:
        if args.output_fault:
            wait(5)
            apply();wait(5)
            double_press();wait(0,30)
            print('Output fault remained latched under valid commands; double press reinitialized successfully.')
            return
        apply();wait(0,30)
        call('/selftest');wait(4)
        apply(80);wait(4)
        time.sleep(.55) # A new pair, not a continuation of the initial single press.
        double_press();wait(0,80)
        print('Single press fails; normal throttle cannot unlock; double press recovers.')
        call('/badinput');wait(3)
        apply(30);wait(0,30)
        print('Out-of-range input fails; fresh valid input recovers.')
        # Sub-50ms repeated clicks must not count as a recovery pair.
        time.sleep(.55)
        call('/selftest');call('/selftest');wait(4)
        print('Fast repeated clicks did not unlock self-test (debounce).')
        time.sleep(.55)
        double_press();wait(0,30)
    finally:
        call('/stop')


if __name__=='__main__':main()
