import math
import unittest
from radar_navigation.base_coordinates import native_command,canonical_wheels,canonical_pose,canonical_covariance


class CoordinateTests(unittest.TestCase):
    def test_commands_and_measured_wheel_mapping_agree(self):
        for v,w in [(0.,0.),(.04,0.),(0.,.2),(.02,-.14),(.03,.12)]:
            nv,nw=native_command(v,w)
            native_left,native_right=nv-nw*.287/2,nv+nw*.287/2
            left,right=canonical_wheels(native_left,native_right)
            self.assertAlmostEqual((left+right)/2,v)
            self.assertAlmostEqual((right-left)/.287,w)

    def test_world_and_body_half_turn_preserve_a_curved_path(self):
        # Native forward moves radar-facing robot backward; positive yaw stays left.
        for theta in [0.,.2,-1.3,math.pi]:
            x,y=math.sin(theta),1-math.cos(theta)
            p,q=canonical_pose((x,y,0),(0,0,math.sin(theta/2),math.cos(theta/2)))
            self.assertAlmostEqual(p[0],-math.sin(theta))
            self.assertAlmostEqual(p[1],math.cos(theta)-1)
            yaw=2*math.atan2(q[2],q[3]);self.assertAlmostEqual(yaw,theta)
            # Chain R(pi) * T(native) * R(pi) has same yaw and world origin.
            nx,ny=math.cos(theta)*(-1),math.sin(theta)*(-1)
            self.assertAlmostEqual(-nx,math.cos(yaw))
            self.assertAlmostEqual(-ny,math.sin(yaw))

    def test_covariance_cross_terms_rotate_with_axes(self):
        vector=[1,2,3,4,5,6];rotated=[-1,-2,3,-4,-5,6]
        covariance=[a*b for a in vector for b in vector]
        self.assertEqual(canonical_covariance(covariance),[a*b for a in rotated for b in rotated])
        self.assertEqual(canonical_covariance(canonical_covariance(covariance)),covariance)
        with self.assertRaises(ValueError):canonical_covariance([0]*35)
