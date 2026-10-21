"""Two-dimensional dead-reckoning model for measured WHILL motion."""

import math
from dataclasses import dataclass


FORWARD_SPEED_MPS = 1.23 / 3.309
BACKWARD_SPEED_MPS = 1.25 / 3.432
RIGHT_TURN_RATE_RADPS = math.asin(0.324675) / 0.820
LEFT_TURN_RATE_RADPS = math.asin(0.4133333) / 1.096


@dataclass(frozen=True)
class MotionState:
    """Estimated planar pose and instantaneous body-frame velocity."""

    x: float
    y: float
    yaw: float
    linear_velocity: float
    angular_velocity: float


def normalize_angle(angle):
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DeadReckoningModel:
    """Integrate keyboard commands with an exact unicycle motion model."""

    def __init__(
            self,
            forward_speed=FORWARD_SPEED_MPS,
            backward_speed=BACKWARD_SPEED_MPS,
            left_turn_rate=LEFT_TURN_RATE_RADPS,
            right_turn_rate=RIGHT_TURN_RATE_RADPS):
        self.forward_speed = float(forward_speed)
        self.backward_speed = float(backward_speed)
        self.left_turn_rate = float(left_turn_rate)
        self.right_turn_rate = float(right_turn_rate)
        self.reset()

    def reset(self):
        """Reset the estimated pose to the startup origin."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    @staticmethod
    def _logical_axes(keys):
        """Convert cardinal and dedicated diagonal keys into two axes."""
        keys = set(keys)
        forward = bool(keys & {'w', 'z', 'c'})
        backward = bool(keys & {'s', 'x', 'v'})
        left = bool(keys & {'a', 'z', 'x'})
        right = bool(keys & {'d', 'c', 'v'})
        linear_axis = int(forward) - int(backward)
        angular_axis = int(left) - int(right)
        return linear_axis, angular_axis

    def commanded_velocity(self, keys, enabled=True):
        """Return calibrated linear and angular velocity for active keys."""
        if not enabled:
            return 0.0, 0.0
        linear_axis, angular_axis = self._logical_axes(keys)
        if linear_axis > 0:
            linear_velocity = self.forward_speed
        elif linear_axis < 0:
            linear_velocity = -self.backward_speed
        else:
            linear_velocity = 0.0

        if angular_axis > 0:
            angular_velocity = self.left_turn_rate
        elif angular_axis < 0:
            angular_velocity = -self.right_turn_rate
        else:
            angular_velocity = 0.0
        return linear_velocity, angular_velocity

    def integrate(self, duration, keys, enabled=True):
        """Advance the pose estimate by one bounded time interval."""
        duration = max(0.0, min(float(duration), 0.25))
        linear_velocity, angular_velocity = self.commanded_velocity(
            keys, enabled=enabled)
        old_yaw = self.yaw
        new_yaw = old_yaw + angular_velocity * duration

        if abs(angular_velocity) < 1e-12:
            self.x += linear_velocity * math.cos(old_yaw) * duration
            self.y += linear_velocity * math.sin(old_yaw) * duration
        else:
            radius = linear_velocity / angular_velocity
            self.x += radius * (math.sin(new_yaw) - math.sin(old_yaw))
            self.y -= radius * (math.cos(new_yaw) - math.cos(old_yaw))

        self.yaw = normalize_angle(new_yaw)
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        return self.state()

    def state(self):
        """Return an immutable snapshot of the current estimate."""
        return MotionState(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
        )
