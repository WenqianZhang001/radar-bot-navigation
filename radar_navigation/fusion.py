"""Timestamp-gated fusion and planar extrinsic calibration."""
import math
from .core import Pose


class Fusion:
    """Fuse already transformed clouds; expire missing sensors and deduplicate voxels."""
    def __init__(self, max_age=.2, voxel=.1):
        self.max_age, self.voxel = max_age, voxel
        self.clouds = {}

    def update(self, sensor, stamp, points):
        previous = self.clouds.get(sensor)
        if previous and stamp < previous[0]:
            return []
        self.clouds[sensor] = (stamp, points)
        newest = max(t for t, _ in self.clouds.values())
        self.clouds = {k: v for k, v in self.clouds.items() if newest-v[0] <= self.max_age}
        cells = {}
        for _, cloud in self.clouds.values():
            for p in cloud:
                key = tuple(math.floor(c/self.voxel) for c in p[:3])
                cells[key] = p
        return list(cells.values())


def calibrate(source, target):
    """Least-squares SE(2) alignment of corresponding surveyed reflector positions.

    Inputs use ROS axes. Requires >=2 distinct corresponding positions.
    Returns pose and RMS residual; correspondence selection is external.
    """
    if len(source) != len(target) or len(source) < 2:
        raise ValueError("At least two corresponding positions required")
    sx, sy = (sum(p[i] for p in source)/len(source) for i in (0,1))
    tx, ty = (sum(p[i] for p in target)/len(target) for i in (0,1))
    a = sum((p[0]-sx)*(q[0]-tx)+(p[1]-sy)*(q[1]-ty) for p,q in zip(source,target))
    b = sum((p[0]-sx)*(q[1]-ty)-(p[1]-sy)*(q[0]-tx) for p,q in zip(source,target))
    if math.hypot(a,b) < 1e-12:
        raise ValueError("Degenerate reflector geometry")
    angle = math.atan2(b,a)
    pose = Pose(tx-math.cos(angle)*sx+math.sin(angle)*sy,
                ty-math.sin(angle)*sx-math.cos(angle)*sy, angle)
    residual = math.sqrt(sum((pose.apply((*p[:2],0,0))[0]-q[0])**2+
                             (pose.apply((*p[:2],0,0))[1]-q[1])**2
                             for p,q in zip(source,target))/len(source))
    return pose, residual
