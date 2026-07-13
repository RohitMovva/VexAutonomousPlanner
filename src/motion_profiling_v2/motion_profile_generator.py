import logging
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

import motion_profiling_v2.one_dim_mp_generator as one_dim_mp_generator

logger = logging.getLogger(__name__)

# Min speed for any point (have be low)
MIN_PROFILE_SPEED = 0.001


@dataclass
class Constraints:
    max_vel: float
    max_acc: float
    max_dec: float
    friction_coef: float
    max_jerk: float
    track_width: float

    # Quicker dt model to take advantage of high accel for AC motors at lower speeds (still capped at some num to prevent wheel slip)
    stall_accel: float = None  # ft/s^2 available at a standstill
    free_speed: float = None  # ft/s at which motor torque reaches zero
    traction_accel: float = None  # ft/s^2 the wheels can transmit (mu * g)

    def has_drivetrain_model(self) -> bool:
        return self.stall_accel is not None and bool(self.free_speed)

    def _apply_ceilings(self, accel: float, ceiling: float, cap: float) -> float:
        """Traction and the configured ceiling always bind; a per-node override replaces
        the ceiling for that node."""
        limit = min(accel, cap if cap is not None else ceiling)
        if self.traction_accel is not None:
            limit = min(limit, self.traction_accel)
        return max(limit, 0.0)

    def accel_limit(self, velocity: float, cap: float = None) -> float:
        """Linear acceleration to command at this speed.

        The torque-speed line says what the motors *can* do; max_acc is a ceiling on what
        we are willing to ask for, and traction is what the tyres can transmit. All three
        bind. cap is a per-node max_acceleration override and replaces max_acc.
        """
        available = float("inf")
        if self.has_drivetrain_model():
            available = self.stall_accel * (
                1.0 - min(abs(velocity), self.free_speed) / self.free_speed
            )
        return self._apply_ceilings(available, self.max_acc, cap)

    def decel_limit(self, velocity: float, cap: float = None) -> float:
        """Braking limit, which is near-flat with speed.

        Driving reverse voltage into a motor that is still spinning forwards puts the
        back-EMF in series with the supply, so the controller pins itself to its current
        limit and holds roughly stall torque the whole way down. Torque is therefore
        never what limits braking - max_dec and traction are. That matters when the
        encoders are also the odometry: braking from speed is the likeliest moment to
        break traction, and a locked wheel corrupts the pose estimate permanently, so
        max_dec is the knob to keep braking inside the friction budget.
        """
        available = self.stall_accel if self.has_drivetrain_model() else float("inf")
        return self._apply_ceilings(available, self.max_dec, cap)

    def angular_accel_limit(self, velocity: float, cap: float = None) -> float:
        """The wheels' force budget spends on angular acceleration too, so this tracks
        whatever linear acceleration is available at this speed."""
        return 2 * self.accel_limit(velocity, cap) / self.track_width

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

    def limit_velocity_by_ang_accel(self, dkappads: float, cap: float = None) -> float:
        """Fastest the robot may travel and still have the angular acceleration to follow
        the curvature ramp ahead of it.

        Steering through a curvature that changes at dkappa/ds costs an angular
        acceleration of v^2 * dkappa/ds, so the limit is the v where that equals what the
        wheels can deliver. With a flat limit A that is just sqrt(A / |dkappa/ds|). With
        the torque-speed model, A itself falls with v - A(v) = (2/tw) * stall * (1 - v/vf)
        - so v solves the quadratic k*v^2 + (A0/vf)*v - A0 = 0 instead. Traction gives a
        second, flat bound; the answer is whichever binds first.
        """
        if abs(dkappads) < 1e-9:
            return self.max_vel

        k = abs(dkappads)
        tw = self.track_width

        def flat(accel):
            """v where v^2 * k exactly spends a constant angular-accel budget."""
            return math.sqrt((2 * accel / tw) / k)

        if cap is not None:
            bounds = [flat(cap)]
        else:
            bounds = [flat(self.max_acc)]
            if self.has_drivetrain_model():
                a0 = 2 * self.stall_accel / tw
                b = a0 / self.free_speed
                bounds.append((-b + math.sqrt(b * b + 4 * k * a0)) / (2 * k))

        if self.traction_accel is not None:
            bounds.append(flat(self.traction_accel))

        return min(min(bounds), self.max_vel)

    def max_accel_at_curvature(
        self,
        velocity: float,
        curvature: float,
        dkappads: float,
        accel_limit: float,
    ) -> float:
        """Largest linear acceleration that keeps the outer wheel within accel_limit.

        A wheel accelerates at a +- alpha * track_width / 2, and since omega = v * kappa
        the angular acceleration is alpha = kappa * a + v^2 * dkappa/ds. Substituting and
        solving the outer wheel's bound provides whats below
        """
        half_tw = self.track_width / 2
        limit = (accel_limit - velocity**2 * abs(dkappads) * half_tw) / (
            1 + abs(curvature) * half_tw
        )
        return max(limit, 0.0)

    def max_accel_across_interval(
        self,
        start_vel: float,
        curvature: float,
        dkappads: float,
        ds: float,
        accel_limit: float,
    ) -> float:
        """Largest acceleration that can be *held across* an interval of length ds and
        still satisfy the outer wheel's bound at the FAR end of it.

        max_accel_at_curvature only checks the end you are standing on. But the same
        acceleration is still in force when the robot arrives at the other end, where
        dkappa/ds may be far worse - and if nothing checks it there, the profile
        accelerates hard into a curvature ramp that has no room for it.

        The far end's velocity is not known independently: v1^2 = v0^2 + 2*a*ds.
        Substituting that into

            a * (1 + |k|*tw/2)  +  v1^2 * |k'| * tw/2  <=  A

        leaves an inequality that is *linear* in a, so it solves exactly:

            a  <=  (A - v0^2 * |k'| * tw/2) / (1 + |k|*tw/2 + ds*|k'|*tw)

        Solving it exactly matters. Bounding v1 above instead and reusing
        max_accel_at_curvature charges the robot for speed it has not reached yet, which
        at a standstill (v0 = 0, where the steering debit must vanish) drives the limit to
        zero and strands the profile at the stop it is trying to leave.
        """
        half_tw = self.track_width / 2
        denom = 1 + abs(curvature) * half_tw + ds * abs(dkappads) * self.track_width
        limit = (accel_limit - start_vel**2 * abs(dkappads) * half_tw) / denom
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
    at) and the distance of every sample.
    """
    total_dist = spline_manager.get_total_arc_length()

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

    # Optional node custom max_acceleration override
    accel_caps = [
        spline_manager.nodes[0].max_acceleration
        if spline_manager.nodes[0].max_acceleration > 0
        else None
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
            accel_caps.append(
                point.max_acceleration if point.max_acceleration > 0 else None
            )
            boundary_map[i] = len(accel_caps) - 1

    logger.info(f"Accel caps: {accel_caps}")
    logger.info(f"Boundary Map: {boundary_map}")

    # np.gradient is second-order accurate on the non-uniform grid.
    curvature_derivs = np.gradient(np.array(curvatures), dists)
    # Spacing between successive samples, used in v^2 = v0^2 + 2 * a * ds.
    deltas = np.diff(dists)

    # Custom accel bounds mapped into actual list to make logic easier in both passes
    caps = []
    cap = accel_caps[0]
    for i in range(len(dists)):
        if i in boundary_map:
            cap = accel_caps[boundary_map[i]]
        caps.append(cap)

    # The speed cap before the passes
    for i in range(len(velocities)):
        velocities[i] = min(
            velocities[i],
            constraints.max_speed_at_curvature(abs(curvatures[i])),
            constraints.limit_velocity_by_ang_accel(curvature_derivs[i], caps[i]),
        )

    def ramp_limit(near, far, ds, avail):
        """Acceleration allowed over the interval between samples `near` and `far`.

        `near` is the end the sweep is standing on, whose velocity is known; `far` is the
        end it is solving for. The wheel budget has to hold at both, so take whichever
        binds harder. Curvature constraints are asked for unconditionally - neither
        divides by kappa (they bind on dkappa/ds), and skipping them where kappa ~ 0 skips
        them at precisely the inflection points, where |dkappa/ds| peaks.
        """
        v = velocities[near]
        return min(
            constraints.max_accel_at_curvature(
                v, curvatures[near], curvature_derivs[near], avail
            ),
            constraints.max_accel_across_interval(
                v, curvatures[far], curvature_derivs[far], ds, avail
            ),
            avail,
        )

    # Forward pass. The sweeps now do nothing but the recursion - no clamping.
    velocities[0] = start_vel

    for i in range(len(velocities) - 1):
        current_vel = velocities[i]

        # What the drivetrain has to spend at this speed. Under the torque-speed model
        # this is large at low speeds and shrinks as the robot winds up.
        avail_accel = constraints.accel_limit(current_vel, caps[i])
        max_accel = ramp_limit(i, i + 1, deltas[i], avail_accel)

        velocities[i + 1] = min(
            velocities[i + 1],
            math.sqrt(current_vel**2 + 2 * max_accel * deltas[i]),
        )

    # Backward pass
    velocities[-1] = end_vel

    for i in range(len(velocities) - 1, 0, -1):
        current_vel = velocities[i]

        avail_decel = constraints.decel_limit(current_vel, caps[i])
        max_decel = ramp_limit(i, i - 1, deltas[i - 1], avail_decel)

        velocities[i - 1] = min(
            velocities[i - 1],
            math.sqrt(current_vel**2 + 2 * max_decel * deltas[i - 1]),
        )

    return velocities, node_samples, dists


def motion_profile_angle(
    angle: int, constraints: Constraints, dt: float = 0.01
) -> Tuple[
    List[float], List[float], List[float], List[float], List[float], List[float]
]:
    
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

    # Exact distance of each node
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

    # Sum path time using dist/velocity
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

    # Calculate accel from velocities
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
