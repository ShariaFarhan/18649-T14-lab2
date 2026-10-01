import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pi'))
from link_health import health_snapshot,advance_deadline
class HealthTests(unittest.TestCase):
    def test_separate_deadlines(self):
        s=health_snapshot(1.09,1.,1.08,1.08)
        self.assertFalse(s['udp_stale'] or s['feedback_stale'])
        s=health_snapshot(1.151,1.,1.14,1.14)
        self.assertTrue(s['udp_stale']);self.assertFalse(s['feedback_stale'])
        s=health_snapshot(1.151,1.14,1.,1.14)
        self.assertFalse(s['udp_stale']);self.assertTrue(s['feedback_stale'])
    def test_absolute_schedule_no_drift_or_replay_burst(self):
        self.assertAlmostEqual(advance_deadline(1.,1.006),1.02)
        self.assertAlmostEqual(advance_deadline(1.02,1.027),1.04)
        self.assertAlmostEqual(advance_deadline(1.04,1.045),1.06)
        self.assertAlmostEqual(advance_deadline(1.06,1.131),1.14)
        self.assertEqual(advance_deadline(1.14,1.135),1.14)
if __name__=='__main__':unittest.main()
