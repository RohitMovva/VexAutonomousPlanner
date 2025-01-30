import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


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
        max_turn_speed = ((2 * self.max_vel / self.track_width) * self.max_vel) / (
            abs(curvature) * self.max_vel + (2 * self.max_vel / self.track_width)
        )

        return min(max_turn_speed, self.max_vel)

    def limit_velocity_by_ang_accel(
        self, dkappads: float, max_angular_accel: float
    ) -> float:
        """Calculate maximum velocity based on angular acceleration and rate of change of curvature."""
        if abs(dkappads) < 1e-9:  # Avoid division by zero if dkappads is very small
            return self.max_vel
        max_v_sq = max_angular_accel / abs(dkappads)
        if max_v_sq < 0:
            return 0.0
        return min(math.sqrt(max_v_sq), self.max_vel)

    def max_accels_at_turn(self, angular_accel: float):
        """Calculate maximum linear acceleration based on angular acceleration"""
        # if (angular_accel < 0):
        #     return self.max_acc
        left_lin_ac = self.max_acc + angular_accel * self.track_width / 2
        right_lin_ac = self.max_acc - angular_accel * self.track_width / 2
        if abs(left_lin_ac) < abs(right_lin_ac):
            return left_lin_ac
        else:
            return right_lin_ac

    def get_wheel_speeds(
        self, linear_vel: float, angular_vel: float
    ) -> Tuple[float, float]:
        """Calculate differential drive wheel speeds from linear and angular velocity"""
        left_vel = linear_vel - (angular_vel * self.track_width / 2)
        right_vel = linear_vel + (angular_vel * self.track_width / 2)
        return left_vel, right_vel


def forward_backward_pass(
    spline_manager,
    constraints: Constraints,
    delta_dist: float,
    start_vel: float = 0.01,
    end_vel: float = 0.01,
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
    curvature_derivs = []
    headings = []

    # Calculate total path length and gather curvatures
    total_dist = spline_manager.get_total_arc_length()
    current_dist = 0

    prev_t = 0
    node_num = 0
    while current_dist < total_dist:
        t = spline_manager.distance_to_time(current_dist)

        curvature = spline_manager.get_curvature(t)
        heading = spline_manager.get_heading(t)

        headings.append(heading)
        curvatures.append(curvature)

        velocities.append(1e9)  # Initialize velocities
        current_dist += delta_dist

        if (prev_t % 1) > (t % 1) and t < spline_manager.distance_to_time(total_dist):
            node_num += 1
            if (spline_manager.nodes[node_num].stop):
                velocities[-1] = 0.01
        prev_t = t

    for i in range(len(curvatures)):
        if i == 0:
            curvature_derivs.append((curvatures[i + 1] + curvatures[i]) / delta_dist)
        elif i == len(curvatures) - 1:
            curvature_derivs.append((curvatures[i] + curvatures[i - 1]) / delta_dist)
        else:
            curvature_derivs.append(
                (curvatures[i + 1] - curvatures[i - 1]) / (2 * delta_dist)
            )

    # Add final point
    velocities.append(end_vel)
    t = spline_manager.distance_to_time(total_dist)
    headings.append(spline_manager.get_heading(t))
    curvatures.append(spline_manager.get_curvature(t))
    curvature_derivs.append(spline_manager.get_curvature_derivative(t))

    # Forward pass
    velocities[0] = start_vel

    prev_ang_vel = 0
    accel_ang = 0
    for i in range(len(velocities) - 1):
        current_vel = velocities[i]
        curvature = curvatures[i]
        ang_vel = velocities[i] * abs(curvature)
        max_ang_acc_vel = constraints.max_vel
        delta_theta = headings[i + 1] - headings[i]

        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_accel = constraints.max_acc
        else:
            # Using the provided formulas
            delta_theta = headings[i + 1] - headings[i]
            accel_ang = (ang_vel**2 - prev_ang_vel**2) / (2 * abs(delta_theta))
            max_vel_ang = max_angular_vel / abs(curvature)
            max_vel_kin = (
                2 * constraints.max_vel / (constraints.track_width * abs(curvature) + 2)
            )
            max_curve_vel = constraints.max_speed_at_curvature(abs(curvature))
            max_ang_acc_vel = constraints.limit_velocity_by_ang_accel(
                curvature_derivs[i], max_angular_accel
            )
            max_linear_vel = min(
                max_vel_ang, max_vel_kin, max_curve_vel, max_ang_acc_vel
            )

            max_accel_ang = max_angular_accel / abs(curvature)
            max_accel_kin = (
                2 * constraints.max_acc / (constraints.track_width * abs(curvature) + 2)
            )

            max_accel_wheel = constraints.max_accels_at_turn(abs(accel_ang))
            if max_accel_wheel < 0:
                max_accel_wheel = 0

            max_accel = min(max_accel_ang, max_accel_kin, max_accel_wheel)

        # if (max_linear_vel > )
        next_vel = min(
            max_linear_vel, math.sqrt(current_vel**2 + 2 * max_accel * delta_dist)
        )

        velocities[i + 1] = min(velocities[i + 1], next_vel)
        prev_ang_vel = ang_vel

        # Final velocity adjustment for track width
        velocities[i + 1] = min(
            velocities[i + 1],
            abs(
                constraints.max_vel
                / (1 + (constraints.track_width * abs(curvature) / 2))
            ),
        )

    # Backward pass
    velocities[-1] = end_vel
    prev_ang_vel = 0

    for i in range(len(velocities) - 1, 0, -1):
        current_vel = velocities[i]
        curvature = curvatures[i]
        ang_vel = velocities[i] * abs(curvature)
        max_ang_acc_vel = constraints.max_vel

        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_decel = constraints.max_dec
        else:
            delta_theta = headings[i - 1] - headings[i]
            accel_ang = (ang_vel**2 - prev_ang_vel**2) / (2 * abs(delta_theta))

            max_vel_ang = max_angular_vel / abs(curvature)
            max_vel_kin = (
                2 * constraints.max_vel / (constraints.track_width * abs(curvature) + 2)
            )
            max_curve_vel = constraints.max_speed_at_curvature(curvature)
            max_ang_acc_vel = constraints.limit_velocity_by_ang_accel(
                curvature_derivs[i], max_angular_accel
            )
            max_linear_vel = min(
                max_vel_ang, max_vel_kin, max_curve_vel, max_ang_acc_vel
            )

            max_decel_ang = max_angular_accel / abs(curvature)
            max_decel_kin = (
                2 * constraints.max_dec / (constraints.track_width * abs(curvature) + 2)
            )

            max_accel_wheel = constraints.max_accels_at_turn(accel_ang)
            if max_accel_wheel < 0:
                max_accel_wheel = 0
            max_decel = min(max_decel_ang, max_decel_kin, max_accel_wheel)

        # Calculate maximum achievable velocity considering deceleration
        prev_vel = math.sqrt(current_vel**2 + 2 * max_decel * delta_dist)

        prev_vel = min(prev_vel, velocities[i - 1], max_linear_vel)

        velocities[i - 1] = prev_vel

        prev_ang_vel = ang_vel

        # Final velocity adjustment for track width
        velocities[i - 1] = min(
            velocities[i - 1],
            abs(
                constraints.max_vel
                / (1 + (constraints.track_width * abs(curvature) / 2))
            ),
        )

    return velocities


def generate_motion_profile(
    spline_manager, constraints: Constraints, dt: float = 0.025, dd: float = 0.005
) -> Tuple[
    List[float], List[float], List[float], List[float], List[float], List[float]
]:
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
    total_length = spline_manager.get_total_arc_length()
    is_reversed = False
    node_idx = 0

    prev_t = 0
    while current_pos < (total_length):
        # print(f"p: {current_pos}")
        t = spline_manager.distance_to_time(current_pos)
        if t % 1 < prev_t % 1 and t < spline_manager.distance_to_time(total_length):
            nodes_map.append(len(times))
            node_idx += 1
            if (spline_manager.nodes[node_idx].is_reverse_node):
                is_reversed = not is_reversed

        # print(is_reversed)
        # print(f"t: {t}")

        prev_t = t
        curvature = spline_manager.get_curvature(t)
        heading = spline_manager.get_heading(t) + (math.pi if is_reversed else 0)
        while (heading > math.pi):
            heading -= 2 * math.pi

        heading *= -1
        coord = spline_manager.get_point_at_parameter(t)

        # Get interpolated target velocity
        target_vel = np.interp(
            current_pos, [i * dd for i in range(len(velocities))], velocities
        )

        next_target_vel = np.interp(
            current_pos + dd, [i * dd for i in range(len(velocities))], velocities
        )

        target_vel = (target_vel + next_target_vel) / 2

        target_vel = max(target_vel, 0.001)  # Small minimum velocity

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

    return (
        times,
        positions,
        linear_vels,
        accelerations,
        headings,
        angular_vels,
        nodes_map,
        coords,
    )


def get_wheel_trajectory(
    linear_vels: List[float], angular_vels: List[float], track_width: float
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
