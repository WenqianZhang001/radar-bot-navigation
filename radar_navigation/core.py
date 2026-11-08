"""Classical perception and bounded log-odds occupancy, independent of ROS."""
import math
from dataclasses import dataclass


@dataclass
class Pose:
    x: float = 0.
    y: float = 0.
    yaw: float = 0.
    z: float = 0.

    def apply(self, point):
        x, y, z, velocity = point
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + c*x - s*y, self.y + s*x + c*y, self.z + z, velocity)


def ros_points(frame):
    return [(y, -x, z, v) for x, y, z, v in frame.points]


class Filter:
    """Range/height/quality gates and optional temporal voxel confirmation.

    Inputs and retained observations must share a fixed world frame. Dynamic
    returns bypass persistence so new moving obstacles are not delayed.
    """
    def __init__(self, voxel=.15, window=.5, confirmations=1, min_range=.15,
                 max_range=8.9, min_z=-.2, max_z=1.5, min_snr=0., doppler=True):
        self.voxel, self.window, self.confirmations = voxel, window, confirmations
        self.min_range, self.max_range = min_range, max_range
        self.min_z, self.max_z, self.min_snr = min_z, max_z, min_snr
        self.doppler = doppler
        self.history = {}
        self.last_time = None

    def update(self, frame, pose=Pose(), ego_velocity=(0., 0.)):
        now = frame.stamp_ns / 1e9
        if self.last_time is not None and now < self.last_time:
            self.history.clear()
        self.last_time = now
        self.history = {k: [a for a in values if now-a[0] <= self.window]
                        for k, values in self.history.items()}
        self.history = {k: v for k, v in self.history.items() if v}
        accepted, seen = [], set()
        self.fresh = []
        for i, p in enumerate(ros_points(frame)):
            if not all(map(math.isfinite, p)):
                continue
            distance = math.hypot(p[0], p[1])
            if not self.min_range <= distance <= self.max_range:
                continue
            info = frame.metadata.get("side_info_db", [])
            if info and (info[i][0] < self.min_snr or
                         (self.min_snr > 0 and not math.isfinite(info[i][0]))):
                continue
            world = pose.apply(p)
            if not self.min_z <= world[2] <= self.max_z:
                continue
            key = tuple(math.floor(v/self.voxel) for v in world[:2])
            # Ego velocity is expressed in the sensor's ROS axes.
            residual = p[3] + (ego_velocity[0]*p[0] + ego_velocity[1]*p[1])/distance
            moving = self.doppler and abs(residual) > .2
            if moving:
                accepted.append(world)
                self.fresh.append(world)
                continue
            if key not in seen:
                self.history.setdefault(key, []).append((now, world))
                seen.add(key)
                if len(self.history[key]) >= self.confirmations:
                    self.fresh.append(world)
        for values in self.history.values():
            if len(values) >= self.confirmations:
                accepted.append(values[-1][1])
        return accepted


class Grid:
    """Fixed odometry-frame occupancy. No-return regions remain unknown."""
    def __init__(self, width=200, height=200, resolution=.1, origin=(-10., -10.), clear=False):
        self.width, self.height, self.resolution = width, height, resolution
        self.origin, self.clear = origin, clear
        self.odds = [0.] * (width*height)
        self.seen = [False] * len(self.odds)
        self.updated = [None] * len(self.odds)

    def cell(self, x, y):
        return (math.floor((x-self.origin[0])/self.resolution),
                math.floor((y-self.origin[1])/self.resolution))

    def index(self, cell):
        x, y = cell
        return y*self.width+x if 0 <= x < self.width and 0 <= y < self.height else None

    def expire(self, now, ttl):
        """Expired evidence becomes unknown, never asserted free."""
        for idx, stamp in enumerate(self.updated):
            if stamp is not None and (now-stamp > ttl or now < stamp):
                self.odds[idx], self.seen[idx], self.updated[idx] = 0., False, None

    def update(self, points, origin=(0., 0.), stamp=None):
        occupied, free = set(), set()
        for x, y, *_ in points:
            end = self.cell(x, y)
            idx = self.index(end)
            if idx is not None:
                occupied.add(idx)
            if self.clear:
                distance = math.hypot(x-origin[0], y-origin[1])
                steps = max(1, math.ceil(distance/self.resolution*2))
                for j in range(steps):
                    idx = self.index(self.cell(origin[0]+(x-origin[0])*j/steps,
                                               origin[1]+(y-origin[1])*j/steps))
                    if idx is not None:
                        free.add(idx)
        for idx in free - occupied:
            self.odds[idx] = max(-4., self.odds[idx]-.4)
            self.seen[idx] = True
            self.updated[idx] = stamp
        for idx in occupied:
            self.odds[idx] = min(4., self.odds[idx]+.85)
            self.seen[idx] = True
            self.updated[idx] = stamp

    def values(self):
        return [round(100/(1+math.exp(-v))) if known else -1
                for v, known in zip(self.odds, self.seen)]
