import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple

# Import the motion profiling code (assuming it's in motion_profile.py)
from motion_profiling_v2.motion_profile_generator import Constraints, forward_backward_pass, generate_motion_profile, get_wheel_trajectory

class SimpleSplineManager:
    """A simple circular path for testing"""
    def __init__(self, radius: float = 1.0):
        self.radius = radius
        self.total_length = 2 * np.pi * radius

    def get_total_arc_length(self) -> float:
        return self.total_length

    def distance_to_time(self, distance: float) -> float:
        return distance / self.total_length * 2 * np.pi

    def get_curvature(self, t: float) -> float:
        return 1.0 / self.radius

    def get_heading(self, t: float) -> float:
        return t

def test_constraints():
    """Test the Constraints class functionality"""
    constraints = Constraints(
        max_vel=2.0,
        max_acc=2.0,
        max_dec=2.0,
        friction_coef=0.8,
        max_jerk=4.0,
        track_width=0.5
    )

    # Test max speed at different curvatures
    print("\nTesting max speeds at different curvatures:")
    curvatures = [0.0, 0.5, 1.0, 2.0]
    for curv in curvatures:
        speed = constraints.max_speed_at_curvature(curv)
        print(f"Curvature: {curv:.1f}, Max Speed: {speed:.2f}")

    # Test wheel speed calculations
    print("\nTesting wheel speed calculations:")
    linear_vel = 1.0
    angular_vel = 0.5
    left_vel, right_vel = constraints.get_wheel_speeds(linear_vel, angular_vel)
    print(f"Linear: {linear_vel}, Angular: {angular_vel}")
    print(f"Left wheel: {left_vel:.2f}, Right wheel: {right_vel:.2f}")

def test_motion_profile():
    """Test the complete motion profile generation"""
    # Create test constraints
    constraints = Constraints(
        max_vel=2.0,
        max_acc=2.0,
        max_dec=2.0,
        friction_coef=0.8,
        max_jerk=4.0,
        track_width=0.5
    )

    # Create a circular test path
    spline_manager = SimpleSplineManager(radius=2.0)

    # Generate motion profile
    times, positions, linear_vels, accels, headings, angular_vels = generate_motion_profile(
        spline_manager,
        constraints,
        dt=0.025,
        dd=0.005
    )

    # Get wheel trajectories
    left_vels, right_vels = get_wheel_trajectory(linear_vels, angular_vels, constraints.track_width)

    # Verify constraints
    print("\nVerifying constraints:")
    print(f"Max velocity: {max(linear_vels):.2f} (limit: {constraints.max_vel})")
    print(f"Max acceleration: {max(accels):.2f} (limit: {constraints.max_acc})")
    print(f"Min acceleration: {min(accels):.2f} (limit: -{constraints.max_dec})")

    # Plot results
    plt.figure(figsize=(15, 10))

    # Velocity profile
    plt.subplot(2, 2, 1)
    plt.plot(times, linear_vels, label='Linear Velocity')
    plt.plot(times, [constraints.max_vel] * len(times), '--', label='Max Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity Profile')
    plt.legend()
    plt.grid(True)

    # Acceleration profile
    plt.subplot(2, 2, 2)
    plt.plot(times, accels)
    plt.plot(times, [constraints.max_acc] * len(times), '--', label='Max Acceleration')
    plt.plot(times, [-constraints.max_dec] * len(times), '--', label='Max Deceleration')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Acceleration Profile')
    plt.legend()
    plt.grid(True)

    # Angular velocity profile
    plt.subplot(2, 2, 3)
    plt.plot(times, angular_vels)
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Angular Velocity Profile')
    plt.grid(True)

    # Wheel velocities
    plt.subplot(2, 2, 4)
    plt.plot(times, left_vels, label='Left Wheel')
    plt.plot(times, right_vels, label='Right Wheel')
    plt.xlabel('Time (s)')
    plt.ylabel('Wheel Velocity (m/s)')
    plt.title('Wheel Velocities')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def run_all_tests():
    """Run all tests"""
    print("=== Testing Constraints ===")
    test_constraints()
    
    print("\n=== Testing Motion Profile Generation ===")
    test_motion_profile()

if __name__ == "__main__":
    run_all_tests()