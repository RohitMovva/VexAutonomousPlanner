import math
import numpy as np
from typing import List, Tuple
from dataclasses import dataclass

@dataclass
class ProfilePoint:
    dist: float
    vel: float

@dataclass
class Constraints:
    max_vel: float
    max_acc: float
    max_dec: float
    friction_coef: float
    max_jerk: float
    track_width: float

    def max_speed_at_curvature(self, curvature: float) -> float:
        """Calculate maximum safe speed based on curvature considering both turning and friction limits"""
        if abs(curvature) < 1e-6:
            return self.max_vel
            
        # Calculate maximum speed based on track width (to prevent tipping)
        max_turn_speed = ((2 * self.max_vel / self.track_width) * self.max_vel) / \
                        (abs(curvature) * self.max_vel + (2 * self.max_vel / self.track_width))
                        
        return min(max_turn_speed, self.max_vel)

    def max_accel_at_curvature(self, starting_velocity: float, curvature: float, next_curvature: float, delta_dist: float) -> float:
        """
        Calculate maximum safe acceleration over a distance considering differential drive constraints.
        
        Args:
            starting_velocity (float): Current linear velocity of the robot
            curvature (float): Current path curvature (1/radius)
            next_curvature (float): Next path curvature
            delta_dist (float): Distance over which acceleration occurs
            
        Returns:
            float: Maximum safe acceleration value
        """
        # Convert curvature to angular velocity (ω = v * κ)
        current_angular_vel = starting_velocity * curvature
        
        # Get current wheel speeds
        left_vel, right_vel = self.get_wheel_speeds(starting_velocity, current_angular_vel)
        
        # Calculate maximum possible velocities for each wheel at the next curvature
        next_angular_vel = starting_velocity * next_curvature
        next_max_vel = self.max_speed_at_curvature(next_curvature)
        max_left_vel, max_right_vel = self.get_wheel_speeds(next_max_vel, next_angular_vel)
        
        # Calculate maximum acceleration for each wheel
        # Using v² = v₀² + 2a∆x formula
        if delta_dist > 0:
            left_accel = (max_left_vel**2 - left_vel**2) / (2 * delta_dist)
            right_accel = (max_right_vel**2 - right_vel**2) / (2 * delta_dist)
        else:
            return 0.0
            
        # Convert wheel accelerations to robot center acceleration
        # The limiting acceleration will be the minimum of the two wheels
        left_limited_accel = min(left_accel, self.max_acc)
        right_limited_accel = min(right_accel, self.max_acc)
        center_accel = min(left_limited_accel, right_limited_accel)
        
        return max(self.max_acc*-1, center_accel)

    def get_wheel_speeds(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """Calculate differential drive wheel speeds from linear and angular velocity"""
        left_vel = linear_vel - (angular_vel * self.track_width / 2)
        right_vel = linear_vel + (angular_vel * self.track_width / 2)
        return left_vel, right_vel

def forward_backward_pass(
    spline_manager,
    constraints: Constraints,
    delta_dist: float,
    start_vel: float = 0.01,
    end_vel: float = 0.01
) -> List[float]:
    """
    Performs forward-backward smoothing with proper consideration of all constraints
    """
    # Derive angular constraints from linear constraints and track width
    max_angular_vel = 2 * constraints.max_vel / constraints.track_width
    max_angular_accel = 2 * constraints.max_acc / constraints.track_width
    
    # print(f"Maximum angular velocity: {max_angular_vel:.2f} rad/s")
    # print(f"Maximum angular acceleration: {max_angular_accel:.2f} rad/s²")
    
    # Rebuild spline tables
    spline_manager.rebuild_tables()
    velocities = []
    curvatures = []
    
    # Calculate total path length and gather curvatures
    total_dist = spline_manager.get_total_arc_length()
    current_dist = 0
    
    while current_dist < total_dist:
        t = spline_manager.distance_to_time(current_dist)
        curvature = spline_manager.get_curvature(t)
        curvatures.append(curvature)
        velocities.append(0)  # Initialize velocities
        current_dist += delta_dist
    
    # Add final point
    velocities.append(end_vel)
    t = spline_manager.distance_to_time(total_dist)
    curvatures.append(spline_manager.get_curvature(t))
    
    # Forward pass
    velocities[0] = start_vel
    old_angular_vel = start_vel * curvatures[0]  # Track previous angular velocity
    
    print("\nForward Pass:")
    print("-------------")
    
    for i in range(len(velocities) - 1):
        current_vel = velocities[i]
        curvature = curvatures[i]
        
        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_accel = constraints.max_acc
        else:
            # Using the provided formulas
            max_vel_ang = max_angular_vel / abs(curvature)
            max_vel_kin = 2 * constraints.max_vel / (constraints.track_width * abs(curvature) + 2)
            max_curve_vel = constraints.max_speed_at_curvature(curvature)
            max_linear_vel = min(max_vel_ang, max_vel_kin, max_curve_vel)
            
            max_accel_ang = max_angular_accel / abs(curvature)
            print(f"Components: {max_angular_accel:.2f}, {abs(curvature):.2f}")
            max_accel_kin = 2 * constraints.max_acc / (constraints.track_width * abs(curvature) + 2)
            max_acc_thing = constraints.max_accel_at_curvature(current_vel, curvature, curvatures[i + 1], delta_dist)
            print(f"Items: {max_accel_ang:.2f}, {max_accel_kin:.2f}, {max_acc_thing:.2f}")
            max_accel = min(max_accel_ang, max_accel_kin, max_acc_thing)

        # Calculate next velocity using the provided formula
        next_vel = min(max_linear_vel, math.sqrt(current_vel**2 + 2 * max_accel * delta_dist))
        
        velocities[i + 1] = next_vel
        
        # Final velocity adjustment for track width
        velocities[i + 1] = min(velocities[i + 1], 
                               abs(constraints.max_vel / (1 + (constraints.track_width * abs(curvature) / 2))))

        # Calculate actual angular acceleration
        current_angular_vel = current_vel * curvature
        next_angular_vel = next_vel * curvatures[i + 1]
        average_velocity = (next_vel + current_vel) / 2
        delta_time = delta_dist / average_velocity
        actual_angular_accel = (next_angular_vel - current_angular_vel) / delta_time
        
        # print(f"\nPoint {i}:")
        # print(f"  Curvature: {curvature:.4f}")
        print(f"  Next lin velo: {next_vel:.5f} Cur lin velo: {current_vel:.5f}")
        print(f"  Cur ang velo: {current_angular_vel:.2f} Next ang velo: {next_angular_vel:.2f}")
        print(f"  Actual angular acceleration: {actual_angular_accel:.2f}")
        # print(f"  Max allowed angular acceleration: {max_angular_accel:.2f}")

        old_angular_vel = next_angular_vel



        # print(f"  Final velocity: {velocities[i - 1]:.2f}")
        print()

    # Backward pass
    velocities[-1] = end_vel
    old_angular_vel = end_vel * curvatures[-1]
    
    print("\nBackward Pass:")
    print("--------------")
    
    for i in range(len(velocities) - 1, 0, -1):
        current_vel = velocities[i]
        curvature = curvatures[i]
        
        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_decel = constraints.max_dec
        else:
            max_vel_ang = max_angular_vel / abs(curvature)
            max_vel_kin = 2 * constraints.max_vel / (constraints.track_width * abs(curvature) + 2)
            max_curve_vel = constraints.max_speed_at_curvature(curvature)
            max_linear_vel = min(max_vel_ang, max_vel_kin, max_curve_vel)
            
            max_decel_ang = max_angular_accel / abs(curvature)
            max_decel_kin = 2 * constraints.max_dec / (constraints.track_width * abs(curvature) + 2)
            max_decel_thing = constraints.max_accel_at_curvature(current_vel, curvature, curvatures[i - 1], delta_dist)
            max_decel = min(max_decel_ang, max_decel_kin, max_decel_thing)
        
        # Calculate maximum achievable velocity considering deceleration
        prev_vel = math.sqrt(current_vel**2 + 2 * max_decel * delta_dist)
        prev_vel = min(prev_vel, velocities[i - 1], max_linear_vel)
        
        # Calculate actual angular acceleration
        current_angular_vel = current_vel * curvature
        prev_angular_vel = prev_vel * curvatures[i - 1]
        average_velocity = (current_vel + prev_vel) / 2
        delta_time = delta_dist / average_velocity
        actual_angular_accel = (current_angular_vel - prev_angular_vel) / delta_time
        
        # print(f"\nPoint {i}:")
        # print(f"  Curvature: {curvature:.4f}")
        # print(f"  Current linear velocity: {current_vel:.2f}")
        # print(f"  Previous linear velocity: {prev_vel:.2f}")
        # print(f"  Current angular velocity: {current_angular_vel:.2f}")
        # print(f"  Previous angular velocity: {prev_angular_vel:.2f}")
        # print(f"  Actual angular acceleration: {actual_angular_accel:.2f}")
        # print(f"  Max allowed angular acceleration: {max_angular_accel:.2f}")
        
        velocities[i - 1] = prev_vel
        old_angular_vel = prev_angular_vel
        
        # Final velocity adjustment for track width
        velocities[i - 1] = min(velocities[i - 1], 
                               abs(constraints.max_vel / (1 + (constraints.track_width * abs(curvature) / 2))))
        
        # print(f"  Final velocity: {velocities[i - 1]:.2f}")
        # print()
    
    return velocities

def generate_motion_profile(
    spline_manager,
    constraints: Constraints,
    dt: float = 0.025,
    dd: float = 0.005
) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]:
    """
    Generates a complete motion profile including velocities, accelerations, and angular components
    """
    # Generate velocity profile
    velocities = forward_backward_pass(spline_manager, constraints, dd)
    
    # Initialize result arrays
    times = []
    positions = []
    linear_vels = []
    angular_vels = []
    accelerations = []
    headings = []
    nodes_map = []
    coords = []
    
    # Initialize tracking variables
    current_time = 0
    current_pos = 0
    current_vel = velocities[0]
    last_heading = 0
    total_length = spline_manager.get_total_arc_length()
    old_left_vel = 0
    old_right_vel = 0
    
    # Print initial conditions
    print(f"\nInitial conditions:")
    print(f"Total path length: {total_length:.6f}")
    print(f"Initial velocity: {current_vel:.6f}")
    print(f"Target end velocity: {velocities[-1]:.6f}")
    print(f"Number of velocity points: {len(velocities)}")
    prev_t = 0
    print(f"Init Distance: {current_pos:.6f}")
    old_angular_vel = 0
    while current_pos < (total_length):
        t = spline_manager.distance_to_time(current_pos)
        if (t%1 < prev_t%1 and t < spline_manager.distance_to_time(total_length)):
            nodes_map.append(len(times))

        prev_t = t
        curvature = spline_manager.get_curvature(t)
        # print(f"Curvature: {curvature:.6f}")
        # print(f"T: {t:.6f}")
        # print(f"Distance: {current_pos:.6f}")
        heading = spline_manager.get_heading(t)
        coord = spline_manager.get_point_at_parameter(t)
        
        # Get interpolated target velocity
        target_vel = np.interp(current_pos, 
                             [i * dd for i in range(len(velocities))], 
                             velocities)
        
        next_target_vel = np.interp(current_pos + dd,
                            [i * dd for i in range(len(velocities))], 
                            velocities)
        
        target_vel = (target_vel + next_target_vel) / 2
        
        target_vel = max(target_vel, 0.001)  # Small minimum velocity
        
        
        # Print debug info for last few iterations
        remaining_dist = total_length - current_pos
        if remaining_dist < dd:  # Only print when very close to end
            print(f"\nDebug info near end:")
            print(f"Current position: {current_pos:.6f}")
            print(f"Remaining distance: {remaining_dist:.6f}")
            print(f"Current velocity: {current_vel:.6f}")
            print(f"Target velocity: {target_vel:.6f}")
            print(f"Time: {current_time:.6f}")
        
        target_vel = max(target_vel, 0.001)  # Small minimum velocity
        
        # Calculate acceleration
        accel = (target_vel - current_vel) / dt
        angular_vel = target_vel * curvature
        ang_accel = (angular_vel - old_angular_vel) / dt
        print(f"Angular acceleration: {ang_accel:.4f}")
        old_angular_vel = angular_vel
        # Apply acceleration limits
        accel = np.clip(accel, -constraints.max_dec, constraints.max_acc)
        
        # Update velocity and position
        current_vel = current_vel + accel * dt
        current_vel = np.clip(current_vel, 0, target_vel)

        left_vel, right_vel = constraints.get_wheel_speeds(current_vel, angular_vel)
        left_accel = (left_vel - old_left_vel) / dt
        right_accel = (right_vel - old_right_vel) / dt
        # print(f"Wheel acceleration: {left_accel:.4f}, {right_accel:.4f}")
        old_left_vel = left_vel
        old_right_vel = right_vel
        
        # Update position
        delta_pos = current_vel * dt + 0.5 * accel * dt * dt
        current_pos += delta_pos
        
        # Store results
        times.append(current_time)
        positions.append(current_pos)
        linear_vels.append(current_vel)
        angular_vels.append(angular_vel)
        accelerations.append(accel)
        headings.append(heading)
        coords.append(coord)
        
        current_time += dt
    
    # Print final state
    print(f"\nFinal state:")
    print(f"Final position: {current_pos:.6f}")
    print(f"Position error: {current_pos - total_length:.6f}")
    print(f"Final velocity: {current_vel:.6f}")
    print(f"Target end velocity: {velocities[-1]:.6f}")
    print(f"Velocity error: {current_vel - velocities[-1]:.6f}")
    print(f"Final time: {current_time:.6f}")
    
    return times, positions, linear_vels, accelerations, headings, angular_vels, nodes_map, coords
    
def get_wheel_trajectory(
    linear_vels: List[float],
    angular_vels: List[float],
    track_width: float
) -> Tuple[List[float], List[float]]:
    """
    Converts linear and angular velocities into left and right wheel velocities
    """
    left_vels = []
    right_vels = []
    
    for linear_vel, angular_vel in zip(linear_vels, angular_vels):
        left_vel = linear_vel - (angular_vel * track_width / 2)
        right_vel = linear_vel + (angular_vel * track_width / 2)
        left_vels.append(left_vel)
        right_vels.append(right_vel)
    
    return left_vels, right_vels
