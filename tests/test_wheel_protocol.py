"""Check raw wheel boundaries and the UDP/UART wire layout."""
import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pi'))
from wheel_protocol import crc, wheel_packet, unpack_wheel, uart_frame


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


if __name__ == '__main__':
    unittest.main()
