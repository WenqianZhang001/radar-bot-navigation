"""Independent command/enable/odometry leases for the physical base."""
import math


class BaseLease:
    def __init__(self, actuate=False, command_timeout=.25, enable_timeout=.3,
                 odom_timeout=.3, max_speed=.05, max_turn=.2):
        if not all(math.isfinite(v) and v>0 for v in (command_timeout,enable_timeout,odom_timeout,max_speed,max_turn)) or max_speed>.10 or max_turn>.35:
            raise ValueError('Finite positive base leases/limits required; absolute caps 0.10 m/s, 0.35 rad/s')
        self.actuate = actuate
        self.command_timeout, self.enable_timeout, self.odom_timeout = command_timeout, enable_timeout, odom_timeout
        self.max_speed, self.max_turn = max_speed, max_turn
        self.command_at = self.enable_at = self.odom_at = self.hardware_at = -math.inf
        self.command = (0., 0.)
        self.reason = 'dry'

    def accept(self, linear, angular, now, unsupported=False):
        if unsupported or not all(math.isfinite(v) for v in (linear, angular)) or linear < 0:
            self.command_at = -math.inf
            return
        self.command = min(linear, self.max_speed), max(-self.max_turn, min(angular, self.max_turn))
        self.command_at = now

    def output(self, now):
        for label, okay in [
            ('dry', self.actuate),
            ('disabled', 0 <= now-self.enable_at <= self.enable_timeout),
            ('hardware_not_ready', 0 <= now-self.hardware_at <= self.odom_timeout),
            ('stale_odometry', 0 <= now-self.odom_at <= self.odom_timeout),
            ('stale_command', 0 <= now-self.command_at <= self.command_timeout)]:
            if not okay:
                self.reason = label
                return 0., 0.
        self.reason = 'enabled'
        return self.command
