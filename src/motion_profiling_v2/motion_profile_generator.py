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
                        
        # Calculate maximum speed based on friction (to prevent slipping)
        max_friction_speed = math.sqrt(self.friction_coef * (1 / abs(curvature)) * 9.81 * 39.3701)
        
        return min(max_turn_speed, self.max_vel)

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
        print(curvature)
        velocities.append(0)  # Initialize velocities
        current_dist += delta_dist
    
    # Add final point
    velocities.append(end_vel)
    t = spline_manager.distance_to_time(total_dist)
    curvatures.append(spline_manager.get_curvature(t))
    
    # Forward pass
    velocities[0] = start_vel
    
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
            max_accel_kin = 2 * constraints.max_acc / (constraints.track_width * abs(curvature) + 2)
            max_accel = min(max_accel_ang, max_accel_kin)
        
        # Calculate next velocity using the provided formula
        velocities[i + 1] = min(max_linear_vel, math.sqrt(current_vel**2 + 2 * max_accel * delta_dist))
        
        # Optional: verify wheel speeds are within bounds
        linear_vel = velocities[i + 1]
        angular_vel = linear_vel * curvature
        left_vel, right_vel = constraints.get_wheel_speeds(linear_vel, angular_vel)
        velocities[i + 1] = min(velocities[i + 1], 
                               abs(constraints.max_vel / (1 + (constraints.track_width * abs(curvature) / 2))))
    
    # Backward pass
    velocities[-1] = end_vel
    
    for i in range(len(velocities) - 1, 0, -1):
        current_vel = velocities[i]
        curvature = curvatures[i]
        
        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_decel = constraints.max_dec
        else:
            # Using the provided formulas again but for deceleration
            max_vel_ang = max_angular_vel / abs(curvature)
            max_vel_kin = 2 * constraints.max_vel / (constraints.track_width * abs(curvature) + 2)
            max_curve_vel = constraints.max_speed_at_curvature(curvature)
            max_linear_vel = min(max_vel_ang, max_vel_kin, max_curve_vel)
            
            max_decel_ang = max_angular_accel / abs(curvature)
            max_decel_kin = 2 * constraints.max_dec / (constraints.track_width * abs(curvature) + 2)
            max_decel = min(max_decel_ang, max_decel_kin)
        
        # Calculate maximum achievable velocity considering deceleration
        max_achievable_vel = math.sqrt(current_vel**2 + 2 * max_decel * delta_dist)
        
        # Take minimum of current velocity and new constraints
        velocities[i - 1] = min(velocities[i - 1], max_achievable_vel, max_linear_vel)
        
        # Optional: verify wheel speeds are within bounds
        linear_vel = velocities[i - 1]
        angular_vel = linear_vel * curvature
        left_vel, right_vel = constraints.get_wheel_speeds(linear_vel, angular_vel)
        velocities[i - 1] = min(velocities[i - 1], 
                               abs(constraints.max_vel / (1 + (constraints.track_width * abs(curvature) / 2))))

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
    
    # Print initial conditions
    print(f"\nInitial conditions:")
    print(f"Total path length: {total_length:.6f}")
    print(f"Initial velocity: {current_vel:.6f}")
    print(f"Target end velocity: {velocities[-1]:.6f}")
    print(f"Number of velocity points: {len(velocities)}")
    prev_t = 0
    while current_pos < (total_length):
        t = spline_manager.distance_to_time(current_pos)
        if (t%1 < prev_t%1 and t < spline_manager.distance_to_time(total_length)):
            nodes_map.append(len(times))

        prev_t = t
        curvature = spline_manager.get_curvature(t)
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
        
        # Apply acceleration limits
        accel = np.clip(accel, -constraints.max_dec, constraints.max_acc)
        
        # Update velocity and position
        current_vel = current_vel + accel * dt
        current_vel = np.clip(current_vel, 0, target_vel)
        
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
