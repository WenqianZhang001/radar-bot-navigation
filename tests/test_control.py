import math
import unittest
from radar_navigation.control import SafetyGate,reactive
from radar_navigation.core import Grid,Filter
from radar_navigation.protocol import Frame


class ControlTests(unittest.TestCase):
    def ready(self):
        gate=SafetyGate()
        gate.armed_at=gate.command_at=gate.cloud_at=10.
        gate.points=[(2.,0.)]
        gate.command=(.08,0.)
        return gate

    def test_start_disarmed(self):
        self.assertEqual(SafetyGate().output(1),(0.,0.))

    def test_deadlines_and_clock_reversal(self):
        for field in ('armed_at','cloud_at','command_at'):
            gate=self.ready()
            setattr(gate,field,9.)
            self.assertEqual(gate.output(10),(0.,0.))
        self.assertEqual(self.ready().output(9),(0.,0.))

    def test_obstacle_empty_invalid_and_reverse(self):
        gate=self.ready()
        self.assertEqual(gate.output(10),(.08,0.))
        for points in ([],[(.5,0)]):
            gate.points=points
            self.assertEqual(gate.output(10),(0.,0.))
        gate.points=[(2,0)]
        for cmd in ((math.nan,0),(-.1,0),(0,math.inf)):
            gate.command=cmd
            self.assertEqual(gate.output(10),(0.,0.))

    def test_rotation_clearance_and_caps(self):
        gate=self.ready()
        gate.command=(1,1)
        self.assertEqual(gate.output(10),(.1,.35))
        gate.points=[(.1,.1)]
        gate.command=(0,.2)
        self.assertEqual(gate.output(10),(0.,0.))

    def test_reactive(self):
        self.assertEqual(reactive([]),(0,0))
        self.assertEqual(reactive([(2,0)]),(.08,0))
        self.assertLess(reactive([(.7,.1)])[1],0)

    def test_aging_no_refresh_from_accumulation(self):
        filt=Filter()
        grid=Grid()
        filt.update(Frame('front',1000000000,1,[[0,2,.3,0]]))
        grid.update(filt.fresh,stamp=1.)
        filt.update(Frame('front',1100000000,2,[]))
        self.assertEqual(filt.fresh,[])
        grid.update(filt.fresh,stamp=1.1)
        grid.expire(3.,1.5)
        self.assertTrue(all(v==-1 for v in grid.values()))

    def test_confirmation_and_snr_applied_to_map_evidence(self):
        filt=Filter(confirmations=2,min_snr=10.)
        frame=Frame('front',100,1,[[0,2,.3,0]],{'side_info_db':[(5.,0.)]})
        filt.update(frame)
        self.assertEqual(filt.fresh,[])
        frame.metadata['side_info_db']=[(20.,0.)]
        filt.update(frame)
        self.assertEqual(filt.fresh,[])
        frame.stamp_ns+=1
        filt.update(frame)
        self.assertEqual(len(filt.fresh),1)
