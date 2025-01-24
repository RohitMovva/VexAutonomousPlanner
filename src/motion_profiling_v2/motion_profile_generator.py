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

    def limit_velocity_by_ang_accel(self, prev_ang_vel: float, angular_accel: float, max_angular_accel: float, delta_theta: float, curvature: float) -> float:
        # print(f"Comp: {angular_accel}, {max_angular_accel}")
        print(f"Inputs: {angular_accel}, {max_angular_accel}, {prev_ang_vel}, {delta_theta}, {curvature}")
        # if (abs(angular_accel) < max_angular_accel):
        #     return self.max_vel
        # return self.max_vel
        
        # print("Limiting velocity by angular acceleration")
        # print(f"Inputs: {prev_ang_vel}, {angular_accel}, {max_angular_accel}, {delta_theta}, {curvature}")
        lin_vel = math.sqrt(prev_ang_vel**2 + 2 * max_angular_accel * abs(delta_theta)) / abs(curvature)
        # if (angular_accel < 0 and prev_ang_vel**2 - 2 * max_angular_accel * abs(delta_theta) > 0):
        #     lin_vel = math.sqrt(prev_ang_vel**2 - 2 * max_angular_accel * abs(delta_theta)) / abs(curvature)
        # print(f"Prev ang vel: {prev_ang_vel}, end ang vel {lin_vel*curvature}")
        return lin_vel

    def max_accels_at_turn(self, angular_accel: float):
        """Calculate maximum linear acceleration based on angular acceleration"""
        # print(f"Inputs: {self.max_acc}, {angular_accel}, {self.track_width}")
        left_lin_ac = self.max_acc + angular_accel * self.track_width / 2
        right_lin_ac = self.max_acc - angular_accel * self.track_width / 2
        # print(f"Outputs: {left_lin_ac}, {right_lin_ac}")
        # print()
        if (abs(left_lin_ac) < abs(right_lin_ac)):
            return left_lin_ac
        else:
            return right_lin_ac

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
    headings = []
    
    # Calculate total path length and gather curvatures
    total_dist = spline_manager.get_total_arc_length()
    current_dist = 0
    
    while current_dist < total_dist:
        t = spline_manager.distance_to_time(current_dist)
        curvature = spline_manager.get_curvature(t)
        heading = spline_manager.get_heading(t)

        curvatures.append(curvature)
        headings.append(heading)

        velocities.append(0)  # Initialize velocities
        current_dist += delta_dist
    
    # Add final point
    velocities.append(end_vel)
    t = spline_manager.distance_to_time(total_dist)
    curvatures.append(spline_manager.get_curvature(t))
    headings.append(spline_manager.get_heading(t))
    
    # Forward pass
    velocities[0] = start_vel
    
    print("\nForward Pass:")
    print("-------------")

    prev_ang_vel = 0
    
    for i in range(len(velocities) - 1):
        current_vel = velocities[i]
        curvature = curvatures[i]
        ang_vel = velocities[i] * abs(curvature)
        max_ang_acc_vel = constraints.max_vel
        print(f" Dist: {i*delta_dist:.6f}")
        
        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_accel = constraints.max_acc
        else:
            # Using the provided formulas
            delta_theta = headings[i+1] - headings[i]
            accel_ang = (ang_vel**2 - prev_ang_vel**2) / (2*delta_theta) * np.sign(abs(curvatures[i+1]) - abs(curvatures[i]))
            
            max_vel_ang = max_angular_vel / abs(curvature)
            max_vel_kin = 2 * constraints.max_vel / (constraints.track_width * abs(curvature) + 2)
            max_curve_vel = constraints.max_speed_at_curvature(abs(curvature))
            max_ang_acc_vel = constraints.limit_velocity_by_ang_accel(ang_vel, accel_ang, max_angular_accel, headings[i+1] - headings[i], curvatures[i+1])
            # max_ang_acc_vel = 1e9
            # print(f"Velocity limits: {max_vel_ang:.6f}, {max_vel_kin:.6f}, {max_curve_vel:.6f}, {max_ang_acc_vel:.6f}")
            max_linear_vel = min(max_vel_ang, max_vel_kin, max_curve_vel, max_ang_acc_vel)
            
            max_accel_ang = max_angular_accel / abs(curvature)
            max_accel_kin = 2 * constraints.max_acc / (constraints.track_width * abs(curvature) + 2)

            # print(f"Angular acceleration: {accel_ang:.4f}")
            max_accel_wheel = constraints.max_accels_at_turn(accel_ang)
            if (max_accel_wheel < 0):
                max_accel_wheel = 0

            max_accel = min(max_accel_ang, max_accel_kin, max_accel_wheel)

        # if (max_linear_vel > )
        next_vel = min(max_linear_vel, math.sqrt(current_vel**2 + 2 * max_accel * delta_dist))
        
        velocities[i + 1] = next_vel
        prev_ang_vel = ang_vel
        
        # Final velocity adjustment for track width
        velocities[i + 1] = min(velocities[i + 1], 
                               abs(constraints.max_vel / (1 + (constraints.track_width * abs(curvature) / 2))))

        # if (max_ang_acc_vel != constraints.max_vel):
        #     velocities[i + 1] = max_ang_acc_vel
        print("Fin vel:", velocities[i+1], max_ang_acc_vel, max_accel)

        print()

    # Backward pass
    velocities[-1] = end_vel
    prev_heading = headings[-1]
    prev_ang_vel = 0
    # old_angular_vel = end_vel * curvatures[-1]
    
    print("\nBackward Pass:")
    print("--------------")
    
    for i in range(len(velocities) - 1, 0, -1):
        current_vel = velocities[i]
        curvature = curvatures[i]
        ang_vel = velocities[i] * curvature
        print(f" Dist: {i*delta_dist:.6f}")
        max_ang_acc_vel = constraints.max_vel
        
        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_decel = constraints.max_dec
        else:
            delta_theta = headings[i-1] - headings[i]
            accel_ang = (ang_vel**2 - prev_ang_vel**2) / (2*delta_theta) * np.sign(abs(curvatures[i-1]) - abs(curvatures[i]))

            max_vel_ang = max_angular_vel / abs(curvature)
            max_vel_kin = 2 * constraints.max_vel / (constraints.track_width * abs(curvature) + 2)
            max_curve_vel = constraints.max_speed_at_curvature(curvature)
            max_ang_acc_vel = constraints.limit_velocity_by_ang_accel(ang_vel, accel_ang, max_angular_accel, headings[i-1] - headings[i], curvatures[i-1])
            # print(f"Velocity limits: {max_vel_ang:.6f}, {max_vel_kin:.6f}, {max_curve_vel:.6f}, {max_ang_acc_vel:.6f}")
            max_linear_vel = min(max_vel_ang, max_vel_kin, max_curve_vel, max_ang_acc_vel)
            
            max_decel_ang = max_angular_accel / abs(curvature)
            # print(max_decel_ang)
            max_decel_kin = 2 * constraints.max_dec / (constraints.track_width * abs(curvature) + 2)


            # print(f"Angular acceleration: {accel_ang:.4f}")
            max_accel_wheel = constraints.max_accels_at_turn(accel_ang)
            if (max_accel_wheel < 0):
                max_accel_wheel = 0

            max_decel = min(max_decel_ang, max_decel_kin, max_accel_wheel)
        
        # Calculate maximum achievable velocity considering deceleration
        prev_vel = math.sqrt(current_vel**2 + 2 * max_decel * delta_dist)
        
        # print(f"Components: {prev_vel:.6f}, {velocities[i-1]}, {max_linear_vel}")
        prev_vel = min(prev_vel, velocities[i - 1], max_linear_vel)
            
        velocities[i - 1] = prev_vel

        prev_heading = headings[i-1]
        prev_ang_vel = ang_vel
        
        # Final velocity adjustment for track width
        velocities[i - 1] = min(velocities[i - 1], 
                               abs(constraints.max_vel / (1 + (constraints.track_width * abs(curvature) / 2))))
        print("Fin vel:", velocities[i-1], max_ang_acc_vel)
        print()

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
    prev_target_vel = 0
    prev_curv = 0
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
        # print(f"Angular acceleration: {ang_accel:.4f}")
        old_angular_vel = angular_vel
        # Apply acceleration limits
        accel = np.clip(accel, -constraints.max_dec, constraints.max_acc)
        
        # Update velocity and position
        current_vel = current_vel + accel * dt
        current_vel = np.clip(current_vel, 0, target_vel)

        old_left_vel, old_right_vel = constraints.get_wheel_speeds(prev_target_vel, prev_target_vel * curvature)
        left_vel, right_vel = constraints.get_wheel_speeds(current_vel, angular_vel)
        left_accel = (left_vel - old_left_vel) / dt
        right_accel = (right_vel - old_right_vel) / dt
        print(f"Wheel acceleration: {current_time/dt}, {left_accel:.4f}, {right_accel:.4f}")
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
        prev_target_vel = target_vel
        prev_curv = curvature
        
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
