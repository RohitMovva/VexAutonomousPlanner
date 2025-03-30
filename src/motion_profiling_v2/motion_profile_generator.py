import logging
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

import motion_profiling_v2.one_dim_mp_generator as one_dim_mp_generator

logger = logging.getLogger(__name__)


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

    velocities = []
    curvatures = []
    curvature_derivs = []
    headings = []

    # Calculate total path length and gather curvatures
    total_dist = spline_manager.get_total_arc_length()
    current_dist = 0

    prev_t = 0
    node_num = 0
    action_idx = 0
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
            if spline_manager.nodes[node_num].stop:
                velocities[-1] = 0.01

        if (
            action_idx < len(spline_manager.action_points)
            and prev_t < spline_manager.action_points[action_idx].t
            and t >= spline_manager.action_points[action_idx].t
        ):
            if spline_manager.action_points[action_idx].stop:
                velocities[-1] = 0.01
            action_idx += 1

        prev_t = t

    # Add final point
    velocities.append(end_vel)
    t = spline_manager.distance_to_time(total_dist)
    headings.append(spline_manager.get_heading(t))
    curvatures.append(spline_manager.get_curvature(t))

    for i in range(len(curvatures)):
        if i == 0:
            curvature_derivs.append((curvatures[i + 1] + curvatures[i]) / delta_dist)
        elif i == len(curvatures) - 1:
            curvature_derivs.append((curvatures[i] + curvatures[i - 1]) / delta_dist)
        else:
            curvature_derivs.append(
                (curvatures[i + 1] - curvatures[i - 1]) / (2 * delta_dist)
            )

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

            max_accel = min(max_accel_ang, max_accel_kin, max_accel_wheel, constraints.max_acc)

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
            max_decel = min(max_decel_ang, max_decel_kin, max_accel_wheel, constraints.max_dec)

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


def motion_profile_angle(
    angle: int, constraints: Constraints, dt: float = 0.01
) -> Tuple[
    List[float], List[float], List[float], List[float], List[float], List[float]
]:
    # angle = math.radians(angle)

    arc_length = abs(angle) * constraints.track_width / 2

    velocities = one_dim_mp_generator.generate_trapezoidal_profile(
        constraints.max_vel, constraints.max_acc, arc_length, dt
    )

    headings = []

    accum_arc_length = 0
    for i in range(len(velocities)):
        current_angle = accum_arc_length / (constraints.track_width / 2)
        headings.append(current_angle * (-1 if angle > 0 else 1))

        accum_arc_length += velocities[i] * dt

    # Differentiate to get angular velocities
    angular_velocities = [0]
    for i in range(1, len(headings)):
        angular_velocities.append((headings[i] - headings[i - 1]) / dt)

    return headings, angular_velocities

def lerp(x, x_array, y_array, cache=None):
    """
    Fast linear interpolation when x_array is sorted.
    Optionally uses a cache to avoid binary search when x is close to previous lookup.
    """
    # Use cached index as starting point if available
    if cache is not None and 'last_idx' in cache:
        idx = cache['last_idx']
        # Check if we're still in the right segment
        if idx < len(x_array) - 1 and x_array[idx] <= x < x_array[idx + 1]:
            # We're in the same segment, use it
            pass
        elif idx > 0 and x_array[idx - 1] <= x < x_array[idx]:
            # We're in the previous segment
            idx -= 1
        else:
            # Find the right segment with binary search
            idx = np.searchsorted(x_array, x, side='right') - 1
    else:
        # Find the right segment with binary search
        idx = np.searchsorted(x_array, x, side='right') - 1
    
    # Boundary checks
    if idx < 0:
        return y_array[0]
    if idx >= len(x_array) - 1:
        return y_array[-1]
    
    # Linear interpolation
    x0, x1 = x_array[idx], x_array[idx + 1]
    y0, y1 = y_array[idx], y_array[idx + 1]
    
    # Store index for next lookup
    if cache is not None:
        cache['last_idx'] = idx
    
    # Linear interpolation formula
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)

def generate_motion_profile(
    spline_manager, constraints: Constraints, dt: float = 0.01, dd: float = 0.005
) -> Tuple[
    List[float], List[float], List[float], List[float], List[float], List[float]
]:
    """
    Generates a complete motion profile including velocities, accelerations, and angular components
    """
    logger.info("Generating motion profile")
    start_time = time.time()

    # Rebuild spline tables
    sm_start_time = time.time()
    spline_manager.rebuild_tables()
    sm_end_time = time.time()
    logger.info(f"Spline table rebuild took {sm_end_time - sm_start_time} seconds")

    # Generate velocity profile
    fb_start_time = time.time()
    velocities = forward_backward_pass(spline_manager, constraints, dd)
    fb_end_time = time.time()

    logger.info(f"Forward-backward pass took {fb_end_time - fb_start_time} seconds")

    # Initialize result arrays
    times = []
    positions = []
    linear_vels = []
    angular_vels = []
    accelerations = []
    headings = []
    nodes_map = [0]
    actions_map = []
    coords = []

    # Initialize tracking variables
    current_time = 0
    current_pos = 0
    current_vel = velocities[0]
    total_length = spline_manager.get_total_arc_length()
    is_reversed = False
    # if spline_manager.nodes[0].is_reverse_node:
    # is_reversed = True
    node_idx = 0
    if spline_manager.nodes[node_idx].is_reverse_node:
        is_reversed = not is_reversed

    if spline_manager.nodes[0].turn != 0:
        angle = np.radians(spline_manager.nodes[0].turn)

        ins_headings, ins_ang_vels = motion_profile_angle(angle, constraints, dt)

        start_heading = headings[-1]
        for i in range(len(ins_headings)):
            while ins_headings[i] + start_heading > math.pi:
                ins_headings[i] -= 2 * math.pi
            while ins_headings[i] + start_heading < -math.pi:
                ins_headings[i] += 2 * math.pi

        positions.extend(0 for _ in ins_headings)
        linear_vels.extend(0 for _ in ins_headings)
        accelerations.extend(0 for _ in ins_headings)
        headings.extend(
            start_heading + ins_headings[i] for i in range(len(ins_headings))
        )
        angular_vels.extend(ins_ang_vels)
        coords.extend(coords[-1] for _ in ins_headings)
        times.extend(current_time + i * dt for i in range(len(ins_headings)))

        current_time += len(ins_headings) * dt

    if spline_manager.nodes[node_idx].wait_time > 0:
        steps = int(spline_manager.nodes[node_idx].wait_time / dt)
        h = -1 * spline_manager.get_heading(0)
        if is_reversed:
            h -= math.pi if is_reversed else 0
        if h > np.pi:
            h -= 2 * np.pi
        if h < -np.pi:
            h += 2 * np.pi
        positions.extend(0 for _ in range(steps))
        linear_vels.extend(0 for _ in range(steps))
        accelerations.extend(0 for _ in range(steps))
        headings.extend(h for _ in range(steps))
        angular_vels.extend(0 for _ in range(steps))
        coords.extend(spline_manager.get_point_at_parameter(0) for _ in range(steps))
        times.extend(current_time + i * dt for i in range(steps))

        current_time += steps * dt

    other_lists_start_time = time.time()
    prev_t = 0
    action_idx = 0
    node_idx = 0
    is_reversed = False

    # Pre-calculate velocity interpolation points for better performance
    velocity_points = [(i * dd, vel) for i, vel in enumerate(velocities)]
    velocity_xs, velocity_ys = zip(*velocity_points)
    print(velocity_xs)
    print(velocity_ys)
    def handle_turn(angle, start_heading, current_time, dt):
        ins_headings, ins_ang_vels = motion_profile_angle(angle, constraints, dt)
        
        # Normalize headings to [-π, π]
        for i in range(len(ins_headings)):
            while ins_headings[i] + start_heading > math.pi:
                ins_headings[i] -= 2 * math.pi
            while ins_headings[i] + start_heading < -math.pi:
                ins_headings[i] += 2 * math.pi

        # Create arrays for turn segment
        turn_len = len(ins_headings)
        positions.extend(positions[-1] for _ in ins_headings)
        linear_vels.extend(0 for _ in ins_headings)
        accelerations.extend(0 for _ in ins_headings)
        headings.extend(start_heading + ins_headings[i] for i in range(turn_len))
        angular_vels.extend(ins_ang_vels)
        coords.extend(coords[-1] for _ in ins_headings)
        times.extend(current_time + i * dt for i in range(turn_len))
        
        return current_time + turn_len * dt

    def handle_wait(wait_time, current_time, dt):
        steps = int(wait_time / dt)
        positions.extend(0 for _ in range(steps))
        linear_vels.extend(0 for _ in range(steps))
        accelerations.extend(0 for _ in range(steps))
        headings.extend(headings[-1] for _ in range(steps))
        angular_vels.extend(0 for _ in range(steps))
        coords.extend(coords[-1] for _ in range(steps))
        times.extend(current_time + i * dt for i in range(steps))
        return current_time + steps * dt
    
    end_param = spline_manager.distance_to_time(total_length)

    while current_pos < total_length:
        t = spline_manager.distance_to_time(current_pos)
        
        # Handle node transitions
        if t % 1 < prev_t % 1 and t < end_param:
            nodes_map.append(len(times))
            node_idx += 1
            current_node = spline_manager.nodes[node_idx]
            
            # Handle turn if present
            if current_node.turn != 0:
                current_time = handle_turn(
                    np.radians(current_node.turn),
                    headings[-1],
                    current_time,
                    dt
                )
            
            # Update reverse state
            if current_node.is_reverse_node:
                is_reversed = not is_reversed
            
            # Handle wait time if present
            if current_node.wait_time > 0:
                current_time = handle_wait(current_node.wait_time, current_time, dt)

        # Handle action points
        if action_idx < len(spline_manager.action_points):
            action_point = spline_manager.action_points[action_idx]
            if prev_t < action_point.t < t:
                actions_map.append(len(times))
                if action_point.wait_time > 0:
                    current_time = handle_wait(action_point.wait_time, current_time, dt)
                action_idx += 1

        prev_t = t
        
        # Get current state
        curvature = spline_manager.get_curvature(t)
        heading = spline_manager.get_heading(t) - (math.pi if is_reversed else 0)
        heading = ((heading + math.pi) % (2 * math.pi)) - math.pi  # Normalize to [-π, π]
        heading *= -1
        
        coord = spline_manager.get_point_at_parameter(t)

        # Optimized velocity interpolation
        target_vel = lerp(current_pos, velocity_xs, velocity_ys)
        next_target_vel = lerp(current_pos + dd, velocity_xs, velocity_ys)
        target_vel = max((target_vel + next_target_vel) / 2, 0.001)  # minimum velocity

        # Calculate acceleration and angular velocity
        accel = np.clip((target_vel - current_vel) / dt, -constraints.max_dec, constraints.max_acc)
        angular_vel = target_vel * curvature * -1

        # Update state
        current_vel = np.clip(current_vel + accel * dt, 0, target_vel)
        delta_pos = current_vel * dt + 0.5 * accel * dt * dt
        if (current_vel <= 0.1):
            delta_pos = 0.1 * dt + 0.5 * accel * dt * dt
        current_pos += delta_pos

        # Store results
        times.append(current_time)
        positions.append(current_pos)
        linear_vels.append(current_vel * (1 if not is_reversed else -1))
        angular_vels.append(angular_vel)
        accelerations.append(accel * (1 if not is_reversed else -1))
        headings.append(heading)
        coords.append(coord)
        current_time += dt

    other_lists_end_time = time.time()
    logger.info(f"Other lists generation took {other_lists_end_time - other_lists_start_time} seconds")

    end_time = time.time()
    logger.info(f"Motion profile generation took {end_time - start_time} seconds")
    logger.info(f"Generated {len(times)} points")
    logger.info(f"headings: {headings}")
    logger.info(f"angular_vels: {angular_vels}")

    return (
        times,
        positions,
        linear_vels,
        accelerations,
        headings,
        angular_vels,
        nodes_map,
        actions_map,
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
