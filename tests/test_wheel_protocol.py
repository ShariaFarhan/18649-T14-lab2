"""Check raw wheel boundaries and the UDP/UART wire layout."""
import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from wheel_protocol import crc, wheel_packet, unpack_wheel, uart_frame, unpack_heartbeat, drive_packet, unpack_drive_packet


class WheelProtocolTests(unittest.TestCase):
    def test_full_unsigned_axis_range_survives_both_links(self):
        for value in (0, 32768, 33151, 65535):
            packet = wheel_packet(0xffffffff, dict(x=value, z=65535, r=0, buttons=0x8001))
            self.assertEqual(len(packet), 276)
            self.assertEqual(unpack_wheel(packet), (0xffffffff, value, 65535, 0, 0x8001))
            frame = uart_frame(255, value, 65535, 0, 0x8001)
            self.assertEqual(frame[:4], bytes.fromhex('7ea50fff'))
            self.assertEqual(len(frame), 21)
            self.assertEqual(struct.unpack('<iiiHB', frame[4:-2]), (value, 65535, 0, 0x8001, 0))
            self.assertEqual(struct.unpack('<H', frame[-2:])[0], crc(frame[2:-2]))

    def test_crc_reference(self):
        self.assertEqual(crc(b'123456789'), 0x6f91)

    def test_stale_and_sequence_wrap(self):
        frame = uart_frame(256, 33151, 65535, 65535, stale=True)
        self.assertEqual(frame[3], 0)
        self.assertEqual(frame[-3], 1)

    def test_wrong_udp_lengths_rejected(self):
        for size in (0, 275, 277):
            with self.assertRaises(ValueError):
                unpack_wheel(bytes(size))

    def test_light_telemetry_preserves_signed_steering(self):
        for percent in (-100,0,100):
            packed=(percent+100)|(1<<8)|(5<<10)
            hb=unpack_heartbeat(struct.pack('<BBHHH',1,0xB5,33151,packed,1500))
            self.assertEqual((hb['percent'],hb['light_mode'],hb['light_pins']),(percent,1,5))
        old=unpack_heartbeat(struct.pack('<BBHHH',0,0xB4,33151,100,0))
        self.assertEqual(old['percent'],0)
        self.assertIsNone(old['light_mode'])
        with self.assertRaises(ValueError):
            unpack_heartbeat(struct.pack('<BBHHH',0,0xB3,0,0,0))

    def test_explicit_right_drive_flag_and_stale_override(self):
        raw=dict(x=33151,z=65535,r=65535,buttons=512)
        data=drive_packet(7,raw,True)
        self.assertEqual(len(data),281)
        self.assertEqual(unpack_drive_packet(data),(7,33151,65535,65535,512,True))
        self.assertFalse(unpack_drive_packet(data[:276])[-1])
        with self.assertRaises(ValueError):unpack_drive_packet(data[:-1]+b'\x02')
        self.assertEqual(uart_frame(0,0,0,0,right_enable=True)[-3],2)
        self.assertEqual(uart_frame(0,0,0,0,right_enable=True,stale=True)[-3],1)
        payload=struct.pack('<BBHHHiiHB',1,0xB6,33151,100,1500,280000,279400,76,1)
        state=unpack_heartbeat(payload)
        self.assertTrue(state['right_drive_available'] and state['right_armed'])
        self.assertEqual((state['target_rpm'],state['right_rpm'],state['right_pwm']),(280,279.4,76))


if __name__ == '__main__':
    unittest.main()
