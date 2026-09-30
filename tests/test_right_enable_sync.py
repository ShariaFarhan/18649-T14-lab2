"""No hardware: exercise a pending enable, acknowledgment, and subsequent MCU disarm."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pi'))
from wheel_steering import reconcile_right_enable
class EnableSyncTests(unittest.TestCase):
    def state(self):
        return dict(right_enable=True,right_acknowledged=False,right_requested_at=10.,stm=dict(right_armed=False))
    def test_pending_then_ack_then_disarm(self):
        s=self.state()
        reconcile_right_enable(s,10.1)
        self.assertTrue(s['right_enable'])
        s['stm']['right_armed']=True
        reconcile_right_enable(s,10.2)
        self.assertTrue(s['right_acknowledged'])
        s['stm']['right_armed']=False
        reconcile_right_enable(s,10.3)
        self.assertFalse(s['right_enable'])
        s['stm']['right_armed']=True
        reconcile_right_enable(s,10.4)
        self.assertFalse(s['right_enable'])
    def test_missing_ack_expires(self):
        s=self.state()
        reconcile_right_enable(s,10.499)
        self.assertTrue(s['right_enable'])
        reconcile_right_enable(s,10.5)
        self.assertFalse(s['right_enable'])
if __name__=='__main__':unittest.main()
