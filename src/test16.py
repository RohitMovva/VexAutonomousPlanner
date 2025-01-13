import unittest
import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple

# Copy of the Constraints class for testing
@dataclass
class Constraints:
    max_vel: float
    max_acc: float
    max_dec: float
    friction_coef: float
    max_jerk: float
    track_width: float

    def max_speed_at_curvature(self, curvature: float) -> float:
        if abs(curvature) < 1e-6:
            return self.max_vel
            
        max_turn_speed = ((2 * self.max_vel / self.track_width) * self.max_vel) / \
                        (abs(curvature) * self.max_vel + (2 * self.max_vel / self.track_width))
                        
        return min(max_turn_speed, self.max_vel)

    def max_accel_at_curvature(self, starting_velocity: float, curvature: float, next_curvature: float, delta_dist: float) -> float:
        print(f"starting_velocity: {starting_velocity}, curvature: {curvature}, next_curvature: {next_curvature}, delta_dist: {delta_dist}")
        starting_angular_vel = starting_velocity * curvature
        left_start_vel, right_start_vel = self.get_wheel_speeds(starting_velocity, starting_angular_vel)
        print(f"left_start_vel: {left_start_vel}, right_start_vel: {right_start_vel}")
        # How do we find delta dist ????
        left_end_vel = math.sqrt(left_start_vel**2 + 2 * self.max_acc * delta_dist) # THIS IS THE ISSUE RAHHHH
        right_end_vel = math.sqrt(right_start_vel**2 + 2 * self.max_acc * delta_dist) # This too ig
        print(f"left_end_vel: {left_end_vel}, right_end_vel: {right_end_vel}")

        left_lin_velo = left_end_vel / (1 - (self.track_width * abs(next_curvature) / 2))
        right_lin_velo = right_end_vel / (1 + (self.track_width * abs(next_curvature) / 2))
        print(f"left_lin_velo: {left_lin_velo}, right_lin_velo: {right_lin_velo}")
        
        left_lin_acc = (left_lin_velo ** 2 - starting_velocity ** 2) / (2 * delta_dist)
        right_lin_acc = (right_lin_velo ** 2 - starting_velocity ** 2) / (2 * delta_dist)
        print(f"left_lin_acc: {left_lin_acc}, right_lin_acc: {right_lin_acc}")
        
        max_accel = min(left_lin_acc, right_lin_acc)
        max_accel = np.clip(max_accel, -self.max_acc, self.max_acc)
        
        return max_accel

    def get_wheel_speeds(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        left_vel = linear_vel - (angular_vel * self.track_width / 2)
        right_vel = linear_vel + (angular_vel * self.track_width / 2)
        return left_vel, right_vel

class TestConstraints(unittest.TestCase):
    def setUp(self):
        # Initialize with reasonable robot constraints
        self.constraints = Constraints(
            max_vel=2.0,        # 2.0 m/s max velocity
            max_acc=1.0,        # 1.0 m/s² max acceleration
            max_dec=1.0,        # 1.0 m/s² max deceleration
            friction_coef=0.7,   # typical rubber-concrete friction
            max_jerk=1.0,       # 1.0 m/s³ max jerk
            track_width=0.5     # 0.5m track width
        )

    def test_straight_path(self):
        """Test acceleration on a straight path"""
        max_accel = self.constraints.max_accel_at_curvature(
            starting_velocity=1.0,  # 1.0 m/s
            curvature=0.0,         # straight path
            next_curvature=0.0,    # remaining straight
            delta_dist=1.0         # over 1.0m
        )
        self.assertLessEqual(max_accel, self.constraints.max_acc)
        self.assertGreaterEqual(max_accel, -self.constraints.max_dec)

    def test_curved_path(self):
        """Test acceleration on a curved path"""
        max_accel = self.constraints.max_accel_at_curvature(
            starting_velocity=1.0,  # 1.0 m/s
            curvature=1.0,         # 1.0 m⁻¹ curvature (1m radius turn)
            next_curvature=1.0,    # maintaining same curvature
            delta_dist=1.0         # over 1.0m
        )
        self.assertLessEqual(max_accel, self.constraints.max_acc)
        self.assertGreaterEqual(max_accel, -self.constraints.max_dec)

    def test_zero_velocity(self):
        """Test acceleration from a stop"""
        max_accel = self.constraints.max_accel_at_curvature(
            starting_velocity=0.0,  # starting from stop
            curvature=0.0,         # straight path
            next_curvature=0.0,    # remaining straight
            delta_dist=1.0         # over 1.0m
        )
        self.assertEqual(max_accel, self.constraints.max_acc)

    def test_max_velocity(self):
        """Test acceleration at max velocity"""
        max_accel = self.constraints.max_accel_at_curvature(
            starting_velocity=self.constraints.max_vel,
            curvature=0.0,
            next_curvature=0.0,
            delta_dist=1.0
        )
        self.assertLessEqual(max_accel, 0.0)  # Should not accelerate above max velocity

    def test_curvature_transition(self):
        """Test acceleration during curvature transition"""
        max_accel = self.constraints.max_accel_at_curvature(
            starting_velocity=1.0,
            curvature=0.0,         # straight
            next_curvature=1.0,    # transitioning to curve
            delta_dist=1.0
        )
        # Should be more conservative than straight line acceleration
        straight_accel = self.constraints.max_accel_at_curvature(1.0, 0.0, 0.0, 1.0)
        print(max_accel, straight_accel)
        self.assertLessEqual(max_accel, straight_accel)

    def test_small_delta_distance(self):
        """Test acceleration over small distance"""
        max_accel = self.constraints.max_accel_at_curvature(
            starting_velocity=1.0,
            curvature=0.0,
            next_curvature=0.0,
            delta_dist=0.1  # small distance
        )
        self.assertLessEqual(max_accel, self.constraints.max_acc)
        self.assertGreaterEqual(max_accel, -self.constraints.max_dec)

if __name__ == '__main__':
    unittest.main()