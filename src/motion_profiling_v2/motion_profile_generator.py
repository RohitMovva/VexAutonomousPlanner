import logging
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

import motion_profiling_v2.one_dim_mp_generator as one_dim_mp_generator

logger = logging.getLogger(__name__)

# Stopped points are held at this speed rather than a true zero, so that a sample
# the robot is stopped at still takes a finite time to traverse. forward_backward_pass
# floors stop nodes here, and the distance -> time inversion relies on that.
#
# It also sets how sharp a reversal looks: the exported velocity steps straight from
# -MIN_PROFILE_SPEED to +MIN_PROFILE_SPEED at a reverse node, which reads as a spurious
# 2 * MIN_PROFILE_SPEED / dt of acceleration on top of the real decel. Keep it small
# enough that this stays well under the noise floor - it costs no meaningful time,
# because the samples either side of a stop dominate the distance -> time integral.
MIN_PROFILE_SPEED = 0.001


@dataclass
class Constraints:
    max_vel: float
    max_acc: float
    max_dec: float
    friction_coef: float
    max_jerk: float
    track_width: float

    def max_speed_at_curvature(self, curvature: float) -> float:
        """Calculate maximum safe speed based on curvature"""
        if abs(curvature) < 1e-6:
            return self.max_vel

        # Calculate maximum speed based on track width
        max_turn_speed = ((2 * self.max_vel / self.track_width) * self.max_vel) / (
            abs(curvature) * self.max_vel + (2 * self.max_vel / self.track_width)
        )

        return min(max_turn_speed, self.max_vel)

    def set_max_vel(self, max_vel):
        self.max_vel = max_vel

    def set_max_acc(self, max_acc):
        self.max_acc = max_acc

    def limit_velocity_by_ang_accel(
        self, dkappads: float, max_angular_accel: float
    ) -> float:
        """Calculate maximum velocity based on angular acceleration and rate of change of curvature."""
        if abs(dkappads) < 1e-9:
            return self.max_vel
        max_v_sq = max_angular_accel / abs(dkappads)
        if max_v_sq < 0:
            return 0.0
        return min(math.sqrt(max_v_sq), self.max_vel)

    def max_accel_at_curvature(
        self,
        velocity: float,
        curvature: float,
        dkappads: float,
        accel_limit: float = None,
    ) -> float:
        """Largest linear acceleration that keeps the outer wheel within max_acc.
        """
        base = self.max_acc if accel_limit is None else accel_limit
        half_tw = self.track_width / 2
        limit = (base - velocity**2 * abs(dkappads) * half_tw) / (
            1 + abs(curvature) * half_tw
        )
        return max(limit, 0.0)

    def get_wheel_speeds(
        self, linear_vel: float, angular_vel: float
    ) -> Tuple[float, float]:
        """Calculate differential drive wheel speeds from linear and angular velocity"""
        left_vel = linear_vel - (angular_vel * self.track_width / 2)
        right_vel = linear_vel + (angular_vel * self.track_width / 2)
        return left_vel, right_vel


def _parameter_to_distance(spline_manager, param: float) -> float:
    """Arc length at a global spline parameter - the inverse of distance_to_time."""
    table = spline_manager.lookup_table
    return float(np.interp(param, table.parameters, table.distances))


def forward_backward_pass(
    spline_manager,
    constraints: Constraints,
    delta_dist: float,
    start_vel: float = 0.01,
    end_vel: float = 0.01,
) -> Tuple[List[float], dict, np.ndarray]:
    """
    Performs forward-backward smoothing with proper consideration of all constraints

    Returns the velocity profile, node_samples (node index -> the sample that node sits
    at) and the distance of every sample. The samples are NOT uniformly spaced: each node
    and action point is pinned to a sample of its own, so a stop node's velocity is
    floored at exactly the distance the spline's heading and curvature jump at. Rounding
    a node onto a uniform grid instead puts the profile's zero up to delta_dist away from
    the node it belongs to, and since v ~ sqrt(2 * a * ds) near a stop, that gap is worth
    a real velocity - so the exported profile reverses direction while still moving, and
    the heading picks up a spurious pi over the samples in between.
    """
    # Derive angular constraints from linear constraints and track width
    max_angular_vel = 2 * constraints.max_vel / constraints.track_width
    max_angular_accel = 2 * constraints.max_acc / constraints.track_width

    total_dist = spline_manager.get_total_arc_length()

    original_max_acc = constraints.max_acc
    original_max_dec = constraints.max_dec

    # Nodes sit at integer parameters; action points carry their parameter directly.
    boundaries = [
        (_parameter_to_distance(spline_manager, n), "node", n)
        for n in range(1, len(spline_manager.nodes) - 1)
    ] + [
        (_parameter_to_distance(spline_manager, ap.t), "action", a)
        for a, ap in enumerate(spline_manager.action_points)
    ]
    boundaries.sort()

    dists = np.unique(
        np.concatenate(
            [
                np.arange(0.0, total_dist, delta_dist),
                [b[0] for b in boundaries],
                [total_dist],
            ]
        )
    )
    # Keep the grid strictly increasing, so the distance -> time inversion can never
    # divide by a zero-length interval when a boundary lands on top of a grid point.
    dists = dists[np.concatenate([[True], np.diff(dists) > 1e-9])]

    starts = {}
    for d, kind, idx in boundaries:
        starts.setdefault(int(np.abs(dists - d).argmin()), []).append((kind, idx))

    velocities = []
    curvatures = []

    max_velocity = constraints.max_vel
    if spline_manager.nodes[0].max_velocity > 0:
        max_velocity = spline_manager.nodes[0].max_velocity

    # A node's max_acceleration override caps both ramps, but where it is unset the two
    # ramps fall back to their own global limits. Resolving the deceleration limit
    # against max_acc instead of max_dec - as this used to - silently discards max_dec
    # entirely, since every boundary overwrote it with the acceleration value.
    max_accels = [
        spline_manager.nodes[0].max_acceleration
        if spline_manager.nodes[0].max_acceleration > 0
        else constraints.max_acc
    ]
    max_decs = [
        spline_manager.nodes[0].max_acceleration
        if spline_manager.nodes[0].max_acceleration > 0
        else constraints.max_dec
    ]

    boundary_map = {0: 0}
    node_samples = {}

    for i, dist in enumerate(dists):
        t = spline_manager.distance_to_time(dist)
        curvatures.append(spline_manager.get_curvature(t))
        velocities.append(max_velocity)

        for kind, idx in starts.get(i, ()):
            point = (
                spline_manager.nodes[idx]
                if kind == "node"
                else spline_manager.action_points[idx]
            )
            if kind == "node":
                node_samples[idx] = i

            if point.stop:
                velocities[-1] = MIN_PROFILE_SPEED

            max_velocity = (
                point.max_velocity if point.max_velocity > 0 else constraints.max_vel
            )
            max_accels.append(
                point.max_acceleration
                if point.max_acceleration > 0
                else constraints.max_acc
            )
            max_decs.append(
                point.max_acceleration
                if point.max_acceleration > 0
                else constraints.max_dec
            )
            boundary_map[i] = len(max_accels) - 1

    logger.info(f"Max Accels: {max_accels}")
    logger.info(f"Boundary Map: {boundary_map}")
    max_accels.append(constraints.max_acc)  # Ensure last point has max_acc
    max_decs.append(constraints.max_dec)

    # np.gradient is second-order accurate on the non-uniform grid.
    curvature_derivs = np.gradient(np.array(curvatures), dists)
    # Spacing between successive samples, used in v^2 = v0^2 + 2 * a * ds.
    deltas = np.diff(dists)

    # Forward pass
    velocities[0] = start_vel

    for i in range(len(velocities) - 1):
        if i in boundary_map:
            constraints.max_acc = max_accels[boundary_map[i]]

        current_vel = velocities[i]
        curvature = curvatures[i]
        max_ang_acc_vel = constraints.max_vel

        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_accel = constraints.max_acc
        else:
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
            max_accel_wheel = constraints.max_accel_at_curvature(
                current_vel, curvature, curvature_derivs[i]
            )

            max_accel = min(max_accel_ang, max_accel_wheel, constraints.max_acc)

        next_vel = min(
            max_linear_vel, math.sqrt(current_vel**2 + 2 * max_accel * deltas[i])
        )

        velocities[i + 1] = min(velocities[i + 1], next_vel)

        velocities[i + 1] = min(
            velocities[i + 1],
            abs(
                constraints.max_vel
                / (1 + (constraints.track_width * abs(curvature) / 2))
            ),
        )

    # Backward pass
    velocities[-1] = end_vel

    for i in range(len(velocities) - 1, 0, -1):
        if i in boundary_map and i > 0:
            constraints.max_dec = max_decs[boundary_map[i] - 1]

        current_vel = velocities[i]
        curvature = curvatures[i]
        max_ang_acc_vel = constraints.max_vel

        if abs(curvature) < 1e-6:
            max_linear_vel = constraints.max_vel
            max_decel = constraints.max_dec
        else:
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

            max_decel_ang = max_angular_accel / abs(curvature)
            max_decel_wheel = constraints.max_accel_at_curvature(
                current_vel, curvature, curvature_derivs[i], constraints.max_dec
            )

            max_decel = min(max_decel_ang, max_decel_wheel, constraints.max_dec)

        # Calculate maximum achievable velocity considering deceleration
        prev_vel = math.sqrt(current_vel**2 + 2 * max_decel * deltas[i - 1])

        prev_vel = min(prev_vel, velocities[i - 1], max_linear_vel)

        velocities[i - 1] = prev_vel

        velocities[i - 1] = min(
            velocities[i - 1],
            abs(
                constraints.max_vel
                / (1 + (constraints.track_width * abs(curvature) / 2))
            ),
        )

    constraints.max_acc = original_max_acc
    constraints.max_dec = original_max_dec

    return velocities, node_samples, dists


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
    if cache is not None and "last_idx" in cache:
        idx = cache["last_idx"]
        # Check if we're still in the right segment
        if idx < len(x_array) - 1 and x_array[idx] <= x < x_array[idx + 1]:
            # We're in the same segment, use it
            pass
        elif idx > 0 and x_array[idx - 1] <= x < x_array[idx]:
            # We're in the previous segment
            idx -= 1
        else:
            # Find the right segment with binary search
            idx = np.searchsorted(x_array, x, side="right") - 1
    else:
        # Find the right segment with binary search
        idx = np.searchsorted(x_array, x, side="right") - 1

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
        cache["last_idx"] = idx

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

    for i, node in enumerate(spline_manager.nodes):
        if (node.is_reverse_node or node.turn != 0) and not node.stop:
            logger.warning(
                f"Node {i} reverses/turns but has its stop overridden off; the "
                f"profile will step discontinuously through it."
            )

    # Rebuild spline tables
    sm_start_time = time.time()
    spline_manager.rebuild_tables()
    sm_end_time = time.time()
    logger.info(f"Spline table rebuild took {sm_end_time - sm_start_time} seconds")

    # Generate velocity profile
    fb_start_time = time.time()
    velocities, node_samples, sample_dists = forward_backward_pass(
        spline_manager, constraints, dd
    )
    fb_end_time = time.time()

    # Node transitions are driven off distance rather than off the spline parameter
    # wrapping. Each node has a sample of its own, so this fires on exactly the sample
    # whose velocity the pass floored to a stop - the one instant the robot is at rest
    # and can reverse without stepping through a discontinuity.
    node_dists = {n: sample_dists[i] for n, i in node_samples.items()}

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

    # Acceleration is constant between two distance samples, so crossing one takes
    # 2 * ds / (v_i + v_i+1) - finite even when one end is stopped, as long as the other
    # isn't. Accumulating gives the time at each distance sample. Sample spacing is not
    # uniform: forward_backward_pass pins every node to a sample of its own.
    sample_times = [0.0]
    for i in range(len(velocities) - 1):
        ds = sample_dists[i + 1] - sample_dists[i]
        v_sum = max(velocities[i] + velocities[i + 1], 2 * MIN_PROFILE_SPEED)
        sample_times.append(sample_times[-1] + 2 * ds / v_sum)

    profile_duration = sample_times[-1]
    pos_cache, vel_cache = {}, {}

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
        positions.extend(positions[-1] for _ in range(steps))
        linear_vels.extend(0 for _ in range(steps))
        accelerations.extend(0 for _ in range(steps))
        headings.extend(headings[-1] for _ in range(steps))
        angular_vels.extend(0 for _ in range(steps))
        coords.extend(coords[-1] for _ in range(steps))
        times.extend(current_time + i * dt for i in range(steps))
        return current_time + steps * dt

    elapsed = 0.0
    while elapsed < profile_duration:
        current_pos = min(
            lerp(elapsed, sample_times, sample_dists, pos_cache), total_length
        )
        current_vel = lerp(elapsed, sample_times, velocities, vel_cache)

        t = spline_manager.distance_to_time(current_pos)

        # Handle node transitions
        while (
            node_idx + 1 in node_dists and current_pos >= node_dists[node_idx + 1]
        ):
            nodes_map.append(len(times))
            node_idx += 1
            current_node = spline_manager.nodes[node_idx]

            # Handle turn if present
            if current_node.turn != 0:
                current_time = handle_turn(
                    np.radians(current_node.turn), headings[-1], current_time, dt
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
        heading = (
            (heading + math.pi) % (2 * math.pi)
        ) - math.pi  # Normalize to [-π, π]
        heading *= -1

        coord = spline_manager.get_point_at_parameter(t)

        # omega = v * kappa
        angular_vel = current_vel * curvature * -1

        # Store results
        times.append(current_time)
        positions.append(current_pos)
        linear_vels.append(current_vel * (1 if not is_reversed else -1))
        angular_vels.append(angular_vel)
        accelerations.append(0.0)  # filled in below, once the signs are known
        headings.append(heading)
        coords.append(coord)

        elapsed += dt
        current_time += dt

    # Acceleration is the derivative of the velocity that actually gets exported, so it
    # has to be differenced from the signed linear_vels. Differencing the unsigned speed
    # and then applying the direction sign - as this used to - hides the direction change
    # at a reverse node entirely: it reports a bounded accel across a step in velocity
    # that no drivetrain could follow, and the reported accel stops being d(v)/dt.
    # times is uniformly spaced by dt, turn and wait blocks included, so a plain
    # difference is valid across the whole profile.
    for i in range(len(linear_vels) - 1):
        accelerations[i] = (linear_vels[i + 1] - linear_vels[i]) / dt
    if accelerations:
        accelerations[-1] = 0.0

    other_lists_end_time = time.time()
    logger.info(
        f"Other lists generation took {other_lists_end_time - other_lists_start_time} seconds"
    )

    end_time = time.time()
    logger.info(f"Motion profile generation took {end_time - start_time} seconds")
    logger.info(f"Generated {len(times)} points")
    logger.debug(f"headings: {headings}")
    logger.debug(f"angular_vels: {angular_vels}")

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
