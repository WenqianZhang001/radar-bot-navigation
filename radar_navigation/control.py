"""Conservative command arbitration shared by ROS nodes and unit tests."""
import math


class SafetyGate:
    def __init__(self, timeout=.3, arm_timeout=.3, max_speed=.10,
                 max_turn=.35, stop_distance=.65, half_width=.30):
        self.timeout, self.arm_timeout = timeout, arm_timeout
        self.max_speed, self.max_turn = max_speed, max_turn
        self.stop_distance, self.half_width = stop_distance, half_width
        self.armed_at = self.cloud_at = self.command_at = -math.inf
        self.points = []
        self.command = (0., 0.)
        self.reason = 'disarmed'

    def output(self, now):
        self.reason = 'disarmed'
        if not 0 <= now-self.armed_at <= self.arm_timeout:
            return 0., 0.
        self.reason = 'stale_perception'
        if not 0 <= now-self.cloud_at <= self.timeout:
            return 0., 0.
        self.reason = 'no_detections'
        if not self.points:
            return 0., 0.
        self.reason = 'stale_command'
        if not 0 <= now-self.command_at <= self.timeout:
            return 0., 0.
        linear, angular = self.command
        self.reason = 'invalid_command'
        if not all(map(math.isfinite, self.command)) or linear < 0:
            return 0., 0.
        self.reason = 'obstacle'
        if any(0 <= x < self.stop_distance and abs(y) < self.half_width
               for x, y in self.points) and linear > 0:
            return 0., 0.
        # Rotation sweeps the entire footprint; never rotate near a detection.
        if angular and any(math.hypot(x,y) < .4 for x,y in self.points):
            return 0., 0.
        self.reason = 'enabled'
        return min(linear,self.max_speed), max(-self.max_turn,min(angular,self.max_turn))


def reactive(points):
    """Slow forward motion or turn away from the nearest frontal return."""
    if not points:
        return 0., 0.
    front = [(x,y) for x,y in points if 0 < x < .9 and abs(y) < .35]
    if front:
        _, y = min(front)
        return 0., -.25 if y >= 0 else .25
    return .08, 0.
