import importlib.util
import struct
import unittest
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'tools/windows_wheel_udp.py'
spec=importlib.util.spec_from_file_location('wheel',p);wheel=importlib.util.module_from_spec(spec);spec.loader.exec_module(wheel)
class PacketTests(unittest.TestCase):
    def test_windows_axes_buttons(self):
        j=wheel.Joy();j.x=33151;j.z=65535;j.r=0;j.buttons=(1<<9)|(1<<2)
        d=wheel.packet(123,j)
        self.assertEqual(len(d),276)
        self.assertEqual(struct.unpack_from('<I',d)[0],123)
        self.assertEqual(struct.unpack_from('<i',d,4)[0],33151)
        self.assertEqual(struct.unpack_from('<i',d,8)[0],65535)
        self.assertEqual(struct.unpack_from('<i',d,24)[0],0)
        self.assertEqual(d[52+9],128);self.assertEqual(d[52+8],0);self.assertEqual(d[52+2],128)
    def test_dummy_force_only(self):
        for value in (-100,-1,0,1,100):self.assertEqual(wheel.decode_force(struct.pack('b',value)),value)
        for bad in (b'',b'abc',b'\x7f',b'\x80'):self.assertIsNone(wheel.decode_force(bad))
if __name__=='__main__':unittest.main()
