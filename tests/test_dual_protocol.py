import struct,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pi'))
from wheel_dual_protocol import uart_frame,drive_packet,unpack_drive_packet,unpack_heartbeat
from wheel_dual import reconcile_drive_enable,auto_enable_drive
class DualProtocolTests(unittest.TestCase):
    def test_pedal_control_starts_released_and_recovers_without_button(self):
        s=dict(enabled=False,drive_enable=False,wheel_ok=True,last_hb=10.,raw=dict(z=65535,r=65535),
               stm=dict(dual_drive_available=True,zone=1,drive_fault=False,drive_armed=False))
        auto_enable_drive(s,10.);self.assertFalse(s['drive_enable'])
        s['enabled']=True;s['raw']['z']=0
        auto_enable_drive(s,10.);self.assertFalse(s['drive_enable'])
        s['raw']['z']=65535;auto_enable_drive(s,10.)
        self.assertTrue(s['drive_enable'])
        s['drive_acknowledged']=True
        reconcile_drive_enable(s,10.01)
        auto_enable_drive(s,10.02);self.assertFalse(s['drive_enable'])
        s['last_hb']=10.12
        auto_enable_drive(s,10.12);self.assertTrue(s['drive_enable'])
    def test_enable_is_distinct_and_stale_wins(self):
        raw=dict(x=33151,z=65535,r=65535,buttons=512)
        packet=drive_packet(7,raw,True)
        self.assertEqual(unpack_drive_packet(packet),(7,33151,65535,65535,512,True))
        self.assertFalse(unpack_drive_packet(packet[:276])[-1])
        for trailer in (b'RD01\x01',b'LD01\x01',b'DD01\x02'):
            with self.assertRaises(ValueError):unpack_drive_packet(packet[:276]+trailer)
        self.assertEqual(uart_frame(0,0,0,0,drive_enable=True)[-3],8)
        self.assertEqual(uart_frame(0,0,0,0,drive_enable=True,stale=True)[-3],1)
    def test_signed_mean_and_fault_reason(self):
        p=struct.pack('<BBHHHiiiHB',1,0xB8,33151,100,1500,200000,180000,220000,60,1)
        s=unpack_heartbeat(p)
        self.assertEqual((s['left_rpm'],s['right_rpm'],s['drive_rpm']),(180,220,200))
        self.assertTrue(s['drive_armed'])
        p=struct.pack('<BBHHHiiiHB',4,0xB8,33151,100,1500,0,-10000,30000,0,4|(4<<4))
        s=unpack_heartbeat(p)
        self.assertEqual(s['drive_rpm'],10)
        self.assertTrue(s['drive_fault'])
        self.assertEqual(s['fault_reason'],'左轮计数反向')
        with self.assertRaises(ValueError):unpack_heartbeat(p[:1]+b'\xb6'+p[2:])
    def test_disarm_does_not_reenable(self):
        s=dict(drive_enable=True,drive_acknowledged=False,drive_requested_at=10.,stm=dict(drive_armed=False))
        reconcile_drive_enable(s,10.1);self.assertTrue(s['drive_enable'])
        s['stm']['drive_armed']=True;reconcile_drive_enable(s,10.2)
        s['stm']['drive_armed']=False;reconcile_drive_enable(s,10.3)
        self.assertFalse(s['drive_enable'])
        s['stm']['drive_armed']=True;reconcile_drive_enable(s,10.4)
        self.assertFalse(s['drive_enable'])
if __name__=='__main__':unittest.main()
