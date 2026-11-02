import logging
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

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

    # ft of *effective* half-track that wheel scrub reduces the outer wheel's velo by
    steering_speed_penalty: float = None

    # Yaw budget. max_speed_at_curvature charges the OUTER wheel's speed, which is what
    # binds on a gentle arc. On a corner whose radius approaches the half-track the inner
    # wheel stops and the outer wheels have to drag the chassis around a pivot against
    # scrub: there the motors hit their current limit long before their speed limit, so
    # the speed budget calls the corner feasible when it is not (OverrideAutonomous log
    # 09122026-9: outer side pinned at 127 for 0.25 s at 2.4 A, delivering 22 of a
    # commanded 35 in/s, heading lag 10.7 deg). These cap the yaw rate and yaw
    # acceleration the profile may ask for. Both are measured off logs, not derived.
    # None disables the cap.
    max_angular_velocity: float = None  # rad/s
    max_angular_acceleration: float = None  # rad/s^2

    # Per-side overrides of max_angular_velocity, keyed by which side is the OUTER wheel
    # (the one that pays for the yaw). The two sides need not be equal: on the 10-19
    # truelocalsawpalt runs (OverrideAutonomous logs 10192026-1012..1021) the right side
    # was saturated on 73% of ticks at the 1.8 rad/s cap when it was outer, the left on
    # 3%, because port 12 drags. None -> max_angular_velocity for that side.
    max_angular_velocity_left_outer: float = None  # rad/s
    max_angular_velocity_right_outer: float = None  # rad/s

    # In-place turns (turn nodes). A spin drives all four motors against a symmetric
    # load, unlike the one-sided pivot the caps above were measured on, so it has its
    # own. Measured at full voltage (OverrideAutonomous logs 09222026-164 and
    # 09242026-289): about 4.3 rad/s, reached at about 7 rad/s^2. Without these the spin
    # ran on the flat max_vel / max_acc, 6-7.4 rad/s, and the robot left every turn
    # 40-70 deg behind. None -> the flat limits. Re-measured 2026-10-09 on the suite's
    # spin_max route (boots 1001, 1002, 1004): about 8 rad/s at full voltage, spin-up
    # 14-16 rad/s^2; config.yaml now caps spins at 7 / 12 for margin.
    max_turn_velocity: float = None  # rad/s
    max_turn_acceleration: float = None  # rad/s^2

    # Spin-through turns (blend_spin_through). The spin starts while the robot is still
    # braking and the outer wheel has to reverse through the cusp, so the in-place
    # limits above, measured from rest, overstate it. First flight (OverrideAutonomous
    # card boots 319/323, 2026-10-30, skills.txt) on 7 / 12: both sides pinned at 127
    # for 33-82% of every flip, outer wheel at the firmware's 45 in/s clamp, wheels
    # slipping at the cusp, heading 15-65 deg behind. The chassis peaked near 5 rad/s
    # reached at about 10 rad/s^2, so these sit below that with feedback headroom.
    # None -> max_turn_velocity / max_turn_acceleration.
    spin_through_velocity: float = None  # rad/s
    spin_through_acceleration: float = None  # rad/s^2

    def spin_through_rate_cap(self) -> float:
        """Yaw rate a spin-through turn may reach at standstill (rad/s)."""
        return (self.spin_through_velocity or self.max_turn_velocity
                or self.max_vel / (self.track_width / 2))

    def spin_through_accel_cap(self) -> float:
        """Yaw acceleration a spin-through turn may use at zero linear accel (rad/s^2)."""
        return (self.spin_through_acceleration or self.max_turn_acceleration
                or self.max_acc / (self.track_width / 2))

    def has_drivetrain_model(self) -> bool:
        return self.stall_accel is not None and bool(self.free_speed)

    def outer_wheel_speed(self, velocity: float, curvature: float) -> float:
        """How fast the OUTER wheel is actually spinning.

        Affects back EMF + torque motor has based on AC motor torque-speed curve
        """
        return abs(velocity) * (1.0 + abs(curvature) * self.track_width / 2.0)

    def wheel_speed_lever(self) -> float:
        """Half-track the outer wheel's speed budget is charged against.

        The outer wheel turns at v * (1 + |k| * tw/2). But it also has to
        pay counteract wheel scrub so essentially widen track width to counteract this with tighter turning
        """
        lever = self.track_width / 2.0
        if self.steering_speed_penalty:
            lever += self.steering_speed_penalty
        return lever

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

    @staticmethod
    def outer_side(curvature: float, reversed_: bool) -> Optional[str]:
        """Which wheel is outer ("left" / "right") on a sample of signed path curvature,
        driving forwards or in reverse. None on a straight.

        The profile emits v = +-|v| and w = -|v| * kappa, and get_wheel_speeds puts the
        right wheel at v + w * tw/2, so the right wheel is outer exactly when v * w > 0:
        kappa < 0 forwards, kappa > 0 in reverse.
        """
        if abs(curvature) < 1e-9:
            return None
        return "right" if (curvature < 0) != reversed_ else "left"

    def yaw_rate_cap(self, outer_side: Optional[str] = None) -> Optional[float]:
        """The yaw-rate cap for a turn whose outer wheel is outer_side. With no side
        given (unknown direction), the strictest configured cap, so it never overstates."""
        per_side = {
            "left": self.max_angular_velocity_left_outer,
            "right": self.max_angular_velocity_right_outer,
        }
        if outer_side is None:
            caps = [c for c in (self.max_angular_velocity, *per_side.values()) if c]
            return min(caps) if caps else None
        return per_side[outer_side] or self.max_angular_velocity

    def max_speed_at_curvature(
        self, curvature: float, outer_side: Optional[str] = None
    ) -> float:
        """Fastest the chassis may travel with the outer wheel inside its speed budget.

        The outer wheel runs at v * (1 + |k| * lever), so holding it at max_vel gives

            v <= max_vel / (1 + |k| * lever)

        which is what the old three-line expression computed, algebraically identical but
        for a straight line where it is exactly max_vel either way. The lever is where the
        steering penalty enters (see wheel_speed_lever): with no penalty this reproduces
        the previous profile bit-for-bit.

        The yaw-rate cap (max_speed_by_yaw_rate) is taken alongside it; whichever binds
        first wins.
        """
        speed_bound = self.max_vel / (1.0 + abs(curvature) * self.wheel_speed_lever())
        return min(speed_bound, self.max_speed_by_yaw_rate(curvature, outer_side))

    def max_speed_by_yaw_rate(
        self, curvature: float, outer_side: Optional[str] = None
    ) -> float:
        """Fastest the chassis may travel without omega = v * kappa exceeding the yaw cap
        for this outer side (see yaw_rate_cap).

        Unbounded without a cap or on a straight, where there is no yaw to limit.
        """
        cap = self.yaw_rate_cap(outer_side)
        if not cap or abs(curvature) < 1e-9:
            return float("inf")
        return cap / abs(curvature)

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

        if self.max_angular_acceleration:
            # v^2 * dkappa/ds on its own must not spend the whole yaw budget.
            bounds.append(math.sqrt(self.max_angular_acceleration / k))

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

    def max_accel_by_yaw_accel(
        self, start_vel: float, curvature: float, dkappads: float, ds: float = 0.0
    ) -> float:
        """Largest linear acceleration that keeps alpha = kappa * a + v^2 * dkappa/ds
        inside the yaw-acceleration cap.

        Same shape as the two outer-wheel bounds above, but on the chassis's yaw rather
        than a wheel's speed. With ds = 0 it is the bound at the sample itself. With
        ds > 0 it is the bound at the FAR end of an interval, v1^2 = v0^2 + 2*a*ds
        substituted in so it stays linear in a:

            a <= (alpha_max - v0^2 * |k'|) / (|k| + 2 * ds * |k'|)

        Unbounded without a cap, and where the path is straight and not bending, since
        there is no yaw to accelerate.
        """
        if not self.max_angular_acceleration:
            return float("inf")
        denom = abs(curvature) + 2.0 * ds * abs(dkappads)
        if denom < 1e-9:
            return float("inf")
        limit = (self.max_angular_acceleration - start_vel**2 * abs(dkappads)) / denom
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
    end_vel: Optional[float] = 0.01,
) -> Tuple[List[float], dict, np.ndarray]:
    """
    Performs forward-backward smoothing with proper consideration of all constraints

    end_vel is the speed forced at the final sample; the default brings the path to
    rest. Passing None means the path does not end at a stop - the endpoint is left
    at whatever the constraints allow it to reach, capped by the last node's
    max_velocity override if it has one. Nothing brakes into the end of the path in
    that case, so the caller owns whatever happens after the trajectory runs out.

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

    # A boundary inserted next to a grid point leaves a sliver interval, and
    # np.gradient over it turns the tiny curvature step at a node into a huge
    # dkappa/ds that limit_velocity_by_ang_accel then slows the robot for. So the
    # boundary replaces any grid point within half a step of it instead.
    grid = np.arange(0.0, total_dist, delta_dist)
    marks = np.array(sorted([b[0] for b in boundaries] + [total_dist]))
    right = np.clip(np.searchsorted(marks, grid), 0, len(marks) - 1)
    left = np.clip(right - 1, 0, len(marks) - 1)
    nearest = np.minimum(np.abs(marks[right] - grid), np.abs(grid - marks[left]))
    grid = grid[(nearest >= delta_dist / 2) | (grid == 0.0)]

    dists = np.unique(
        np.concatenate(
            [
                grid,
                [b[0] for b in boundaries],
                [total_dist],
            ]
        )
    )
    # Keep the grid strictly increasing, so the distance -> time inversion can never
    # divide by a zero-length interval when a boundary lands on top of a grid point.
    dists = dists[np.concatenate([[True], np.diff(dists) > 1e-9])]

    # Two stops closer than a grid step can be left as neighbouring samples with the
    # grid point between them taken out above. Both pinned to MIN_PROFILE_SPEED, the
    # gap is then crossed at that speed: a stop action point 0.1 in before a stop node
    # took 8.2 s in skills (2026-10-23). A sample midway lets the sweeps speed up and
    # brake back down across the gap.
    stop_dists = sorted(
        [0.0, total_dist]
        + [
            d
            for d, kind, idx in boundaries
            if (
                spline_manager.nodes[idx]
                if kind == "node"
                else spline_manager.action_points[idx]
            ).stop
        ]
    )
    midpoints = []
    for a, b in zip(stop_dists, stop_dists[1:]):
        if b - a <= 1e-9:
            continue
        lo = int(np.searchsorted(dists, a + 1e-9, side="left"))
        hi = int(np.searchsorted(dists, b - 1e-9, side="right"))
        if lo >= hi:
            midpoints.append((a + b) / 2)
    if midpoints:
        dists = np.unique(np.concatenate([dists, midpoints]))

    starts = {}
    for d, kind, idx in boundaries:
        starts.setdefault(int(np.abs(dists - d).argmin()), []).append((kind, idx))

    velocities = []
    # Batched: one interp over the lookup table and one over the precomputed
    # curvature table, instead of two scalar lookups per sample.
    curvatures = spline_manager.get_curvatures(spline_manager.distances_to_times(dists))

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
    # Driving direction per sample, for the per-side yaw cap. Toggles at a reverse node
    # the same way generate_motion_profile's is_reversed does.
    reversed_now = bool(spline_manager.nodes[0].is_reverse_node)
    reversed_at = []

    for i in range(len(dists)):
        velocities.append(max_velocity)

        for kind, idx in starts.get(i, ()):
            point = (
                spline_manager.nodes[idx]
                if kind == "node"
                else spline_manager.action_points[idx]
            )
            if kind == "node":
                node_samples[idx] = i
                if point.is_reverse_node:
                    reversed_now = not reversed_now

            if point.stop:
                velocities[-1] = MIN_PROFILE_SPEED

            max_velocity = (
                point.max_velocity if point.max_velocity > 0 else constraints.max_vel
            )
            accel_caps.append(
                point.max_acceleration if point.max_acceleration > 0 else None
            )
            boundary_map[i] = len(accel_caps) - 1

        reversed_at.append(reversed_now)

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
            constraints.max_speed_at_curvature(
                curvatures[i], Constraints.outer_side(curvatures[i], reversed_at[i])
            ),
            constraints.limit_velocity_by_ang_accel(curvature_derivs[i], caps[i]),
        )

    # The last node never gets a boundary entry - nothing follows it, so its
    # max_velocity would have no samples to govern. When the path does not end at a
    # stop it governs exactly one: the exit speed. Applied before the sweeps so the
    # forward pass cannot overshoot it and the backward pass brakes into it.
    if end_vel is None and spline_manager.nodes[-1].max_velocity > 0:
        velocities[-1] = min(velocities[-1], spline_manager.nodes[-1].max_velocity)

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
            # The yaw budget has to hold at both ends too.
            constraints.max_accel_by_yaw_accel(
                v, curvatures[near], curvature_derivs[near]
            ),
            constraints.max_accel_by_yaw_accel(
                v, curvatures[far], curvature_derivs[far], ds
            ),
            avail,
        )

    # Forward pass. The sweeps now do nothing but the recursion - no clamping.
    velocities[0] = start_vel

    for i in range(len(velocities) - 1):
        current_vel = velocities[i]

        # What the drivetrain has to spend at this speed. Under the torque-speed model
        # this is large at low speeds and shrinks as the robot speeds up.
        #
        # Evaluated at the OUTER WHEEL's speed account for individual speed of
        # drivebase sides
        avail_accel = constraints.accel_limit(
            constraints.outer_wheel_speed(current_vel, curvatures[i]), caps[i]
        )
        max_accel = ramp_limit(i, i + 1, deltas[i], avail_accel)

        velocities[i + 1] = min(
            velocities[i + 1],
            math.sqrt(current_vel**2 + 2 * max_accel * deltas[i]),
        )

    # Backward pass. With end_vel None the endpoint keeps whatever the forward pass
    # reached, so the sweep has nothing to brake into and the profile ends at speed.
    if end_vel is not None:
        velocities[-1] = end_vel

    for i in range(len(velocities) - 1, 0, -1):
        current_vel = velocities[i]

        # decel limit is just to prevent wheel slip since with back EMF robot can 
        # decel very quickly
        avail_decel = constraints.decel_limit(current_vel, caps[i])
        max_decel = ramp_limit(i, i - 1, deltas[i - 1], avail_decel)

        velocities[i - 1] = min(
            velocities[i - 1],
            math.sqrt(current_vel**2 + 2 * max_decel * deltas[i - 1]),
        )

    return velocities, node_samples, dists


def motion_profile_angle(
    angle: int, constraints: Constraints, dt: float = 0.01,
    max_vel: float = 0.0, max_acc: float = 0.0,
) -> Tuple[
    List[float], List[float], List[float], List[float], List[float], List[float]
]:
    """In-place turn profile. max_vel / max_acc are the turn node's own max_velocity /
    max_acceleration overrides (wheel ft/s, ft/s^2; 0 = none): like a node override on
    the path, each one replaces the limit it overrides -- here the spin's caps -- so a
    node can spin slower or faster than the config allows (the spin-rate test routes)."""
    half_track = constraints.track_width / 2
    arc_length = abs(angle) * half_track

    # Wheel speed and acceleration of the spin: the flat limits, under the spin's own
    # caps where the config has them (not the path's yaw caps; see Constraints).
    wheel_vel, wheel_acc = constraints.max_vel, constraints.max_acc
    if constraints.max_turn_velocity:
        wheel_vel = min(wheel_vel, constraints.max_turn_velocity * half_track)
    if constraints.max_turn_acceleration:
        wheel_acc = min(wheel_acc, constraints.max_turn_acceleration * half_track)
    if max_vel > 0:
        wheel_vel = max_vel
    if max_acc > 0:
        wheel_acc = max_acc
    velocities = one_dim_mp_generator.generate_trapezoidal_profile(
        wheel_vel, wheel_acc, arc_length, dt
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


def sample_profile_state(elapsed, sample_times, sample_dists, velocities):
    """Sample the constant-acceleration intervals used by the distance sweeps.

    Their duration is 2*ds/(v0+v1). Velocity is linear in time, but distance
    is quadratic; interpolating distance linearly gives the wrong pose for
    the velocity supplied to the robot's tracking controller.
    """
    if elapsed <= sample_times[0]:
        return sample_dists[0], velocities[0]
    if elapsed >= sample_times[-1]:
        return sample_dists[-1], velocities[-1]
    i = int(np.searchsorted(sample_times, elapsed, side="right")) - 1
    duration = sample_times[i + 1] - sample_times[i]
    tau = elapsed - sample_times[i]
    acceleration = (velocities[i + 1] - velocities[i]) / duration
    velocity = velocities[i] + acceleration * tau
    position = sample_dists[i] + velocities[i] * tau + 0.5 * acceleration * tau**2
    return position, velocity


def blends_turn(node, node_index: int, node_count: int) -> bool:
    """Whether a turn node's spin overlaps the ramps either side of it (spin_through)
    instead of running as a stop, spin in place, go.

    Needs a segment on both sides to overlap. A node with a wait keeps the in-place
    spin: the robot is meant to sit still there, and a lift wait holds the
    trajectory clock on the robot, which would freeze it mid-spin."""
    if not getattr(node, "spin_through", False) or node.turn == 0:
        return False
    if not 0 < node_index < node_count - 1 or node.wait_time > 0:
        return False
    actions = node.get_action_map() if hasattr(node, "get_action_map") else node.actions
    return not actions.get("wait_lift", 0.0)


def _smoothstep(x):
    """3x^2 - 2x^3 on [0, 1] and its derivative; flat at both ends."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x), 6.0 * x * (1.0 - x)


def _spin_capacity(w_lim, a_lim, lo, hi, dt):
    """Fastest spin that fits samples lo..hi-1, starting and ending at rest, under a
    per-sample yaw-rate limit w_lim and yaw-accel limit a_lim. Returns the rate per
    sample (zero outside the window)."""
    n = len(w_lim)
    fwd = np.zeros(n)
    for k in range(lo + 1, hi):
        fwd[k] = min(w_lim[k], fwd[k - 1] + a_lim[k] * dt)
    bwd = np.zeros(n)
    for k in range(hi - 2, lo - 1, -1):
        bwd[k] = min(w_lim[k], bwd[k + 1] + a_lim[k] * dt)
    rate = np.minimum(fwd, bwd)
    rate[:lo] = 0.0
    rate[hi:] = 0.0
    return rate


# Fractions of the flat accel/decel ceilings tried for the ramps into and out of a
# spin-through node. Braking less hard leaves the spin acceleration to work with, and
# rolling slower near the cusp leaves the spin yaw rate. Tight curved flips still sat
# still at the cusp at 0.35 (skills5, 2026-10-31), so the ladder goes well below it.
SPIN_THROUGH_RAMP_FRACTIONS = (1.0, 0.8, 0.65, 0.5, 0.35, 0.25, 0.18, 0.12, 0.08)


def _retime_ramps(s, v, h, c, m, s_node, fraction, constraints, dt):
    """Resample a blend region with the ramps into and out of its node capped at
    `fraction` of max_dec / max_acc.

    s, v, h, c are the region's distance, signed speed, heading (unwrapped) and
    coords; the node sits between samples m-1 and m at distance s_node. Only the
    stretch next to the node where the slower ramp binds is resampled. Returns the
    new arrays and the index of the first sample after the node."""
    n = len(s)

    def ramp(dist, accel):
        return np.sqrt(2.0 * fraction * accel * np.maximum(dist, 0.0))

    # The resampled stretch: from the last sample before the node where the slower
    # braking curve does not bind, to the first one after it where launching does not.
    k_in = m - 1
    while k_in > 0 and ramp(s_node - s[k_in], constraints.max_dec) < abs(v[k_in]):
        k_in -= 1
    k_out = m
    while k_out < n - 1 and ramp(s[k_out] - s_node, constraints.max_acc) < abs(v[k_out]):
        k_out += 1
    if k_in >= m - 1 and k_out <= m:
        return s, v, h, c, m

    def side(lo, hi, before):
        """Distance grid, speed and elapsed time across one side of the node."""
        ss = np.concatenate([s[lo:hi], [s_node]]) if before else np.concatenate([[s_node], s[lo:hi]])
        vs = np.concatenate([np.abs(v[lo:hi]), [0.0]]) if before else np.concatenate([[0.0], np.abs(v[lo:hi])])
        ss = np.maximum.accumulate(ss)
        grid = np.linspace(ss[0], ss[-1], max(50, int((ss[-1] - ss[0]) / 0.001)))
        accel = constraints.max_dec if before else constraints.max_acc
        to_node = (s_node - grid) if before else (grid - s_node)
        speed = np.minimum(np.interp(grid, ss, vs), ramp(to_node, accel))
        if before:
            speed[0] = abs(v[lo])
        else:
            speed[-1] = abs(v[hi - 1])
        steps = 2.0 * np.diff(grid) / np.maximum(speed[1:] + speed[:-1], 1e-6)
        return grid, speed, np.concatenate([[0.0], np.cumsum(steps)])

    g_in, u_in, t_in = side(k_in, m, True)
    g_out, u_out, t_out = side(m, k_out + 1, False)
    total = t_in[-1] + t_out[-1]
    count = max(1, int(round(total / dt)))
    scale = total / (count * dt)  # stretches the stretch onto whole samples

    sign_in = np.sign(v[k_in:m].sum()) or 1.0
    sign_out = np.sign(v[m : k_out + 1].sum()) or 1.0
    new_s, new_v, new_h, new_c = [], [], [], []
    for k in range(1, count):
        tau = k * dt * scale
        if tau < t_in[-1]:
            dist = float(np.interp(tau, t_in, g_in))
            speed = sign_in * float(np.interp(dist, g_in, u_in))
            idx = slice(k_in, m)
        else:
            dist = float(np.interp(tau - t_in[-1], t_out, g_out))
            speed = sign_out * float(np.interp(dist, g_out, u_out))
            idx = slice(m, k_out + 1)
        new_s.append(dist)
        new_v.append(speed / scale)
        new_h.append(np.interp(dist, s[idx], h[idx]))
        new_c.append([np.interp(dist, s[idx], c[idx, 0]), np.interp(dist, s[idx], c[idx, 1])])
    new_m = k_in + 1 + int(np.searchsorted(
        [k * dt * scale for k in range(1, count)], t_in[-1], side="left"
    ))

    def join(a, middle):
        middle = np.asarray(middle, dtype=float).reshape((-1,) + a.shape[1:])
        return np.concatenate([a[: k_in + 1], middle, a[k_out:]])

    return join(s, new_s), join(v, new_v), join(h, new_h), join(c, new_c), new_m


def _uniform_slowdown(v, w, h, c, s, factor, dt):
    """Run a region `factor` times slower along the same path, resampled on dt."""
    n = len(v)
    count = max(2, int(round((n - 1) * factor)) + 1)
    old_t = np.arange(n) * dt
    new_t = np.linspace(0.0, old_t[-1], count)

    def at(values):
        return np.interp(new_t, old_t, values)

    return (
        at(v) / factor, at(w) / factor, at(h),
        np.column_stack([at(c[:, 0]), at(c[:, 1])]), at(s), new_t,
    )


def blend_spin_through(
    node, lists: dict, first: int, split: int, last: int, s_node: float, delta: float,
    constraints: Constraints, dt: float,
):
    """Overlap a turn node's spin with the brake into it and the launch out of it.

    lists holds the profile's per-sample lists. Samples first..split-1 drive into the
    node (from the previous node's sample), split..last-1 drive out to the next node;
    the node is s_node along the path. The in-place spin was never inserted; delta is
    the heading change it would have made (the profile's sign convention). Lists are
    edited in place. Returns a function mapping an old sample index at or after
    `first` to its new one.

    The spin may use whatever yaw budget the drive leaves: rate up to
    spin_through_velocity * (1 - |v| / max_vel) and accel up to
    spin_through_acceleration * (1 - |a| / max_acc), each less what the path is already
    turning at. That straight line between the measured straight-line and in-place
    limits is an assumption, not a measurement. Braking at full decel leaves the spin
    no acceleration, so the ramps next to the node are also tried at gentler fractions
    (SPIN_THROUGH_RAMP_FRACTIONS). If the spin still does not fit, the robot dwells at
    the node for the shortest time that lets it. Each fraction is carried through the
    pose shift and slowdown below, and the one with the shortest final region is kept.

    A robot that turns while moving leaves the spline, so the pose is integrated
    through the window. The mismatch this leaves at the next node is then spread back
    along the distance travelled, flat at both neighbouring nodes and at the cusp, and
    heading, speed and yaw rate are recomputed from that shifted path. The result
    stays something a differential drive can follow and still ends exactly on the
    next node. Should that push a wheel past max_vel, the whole region is slowed
    uniformly until it does not.
    """
    half_tw = constraints.track_width / 2
    turn_vel = constraints.spin_through_rate_cap()
    turn_acc = constraints.spin_through_accel_cap()
    # Per-node overrides are wheel speeds, as in motion_profile_angle.
    if node.max_velocity > 0:
        turn_vel = node.max_velocity / half_tw
    if node.max_acceleration > 0:
        turn_acc = node.max_acceleration / half_tw
    max_lin_acc = max(constraints.max_acc, constraints.max_dec)

    s0 = np.asarray(lists["positions"][first:last], dtype=float)
    v0 = np.asarray(lists["linear_vels"][first:last], dtype=float)
    w0 = np.asarray(lists["angular_vels"][first:last], dtype=float)
    h0 = np.asarray(lists["headings"][first:last], dtype=float)
    c0 = np.array([np.asarray(c, dtype=float).ravel()[:2] for c in lists["coords"][first:last]])
    m0 = split - first
    # Unwrapped per side: the spline's heading jumps by the turn across the node.
    h0 = np.concatenate([np.unwrap(h0[:m0]), np.unwrap(h0[m0:])])

    def extend(values, m, dwell, fill):
        return np.concatenate([values[:m], np.full(dwell, fill), values[m:]])

    def capacity(v, w, m, dwell, lo=0, hi=None):
        v = extend(v, m, dwell, 0.0)
        w = extend(w, m, dwell, 0.0)
        acc = np.gradient(v, dt) if len(v) > 1 else np.zeros(len(v))
        yaw_acc = np.gradient(w, dt) if len(w) > 1 else np.zeros(len(w))
        w_lim = np.maximum(turn_vel * (1.0 - np.abs(v) / constraints.max_vel) - np.abs(w), 0.0)
        a_lim = np.maximum(turn_acc * (1.0 - np.abs(acc) / max_lin_acc) - np.abs(yaw_acc), 0.0)
        hi = len(w_lim) if hi is None else hi
        return _spin_capacity(w_lim, a_lim, lo, hi, dt)

    target = abs(delta)

    def shortest_dwell(v, w, m):
        _, spin_rates = motion_profile_angle(
            delta, constraints, dt, node.max_velocity, node.max_acceleration
        )
        lo_d, hi_d = 0, len(spin_rates) + 2
        while capacity(v, w, m, hi_d).sum() * dt < target:
            hi_d *= 2
        while lo_d < hi_d:
            mid = (lo_d + hi_d) // 2
            if capacity(v, w, m, mid).sum() * dt >= target:
                hi_d = mid
            else:
                lo_d = mid + 1
        return lo_d

    def path_rates(h, m):
        """Yaw rate of the path itself, from heading, per side of the node."""
        rates = np.zeros(len(h))
        for part in (slice(0, m), slice(m, len(h))):
            if part.stop - part.start > 1:
                rates[part] = np.gradient(h[part], dt)
        return rates

    def finish(s, v, w, h, c, m, dwell):
        """Place the spin, integrate and shift the pose, and slow the region down if
        that pushed it out of budget. Returns the final region and its node split."""
        # Pull the window in from both ends while the spin still fits, so the turning
        # happens where the robot is slowest and strays least from the spline.
        n = len(v) + dwell
        lo, hi = 0, n
        moved = True
        while moved:
            moved = False
            if lo < m and capacity(v, w, m, dwell, lo + 1, hi).sum() * dt >= target:
                lo += 1
                moved = True
            if hi > m + dwell + 1 and capacity(v, w, m, dwell, lo, hi - 1).sum() * dt >= target:
                hi -= 1
                moved = True
        spin = capacity(v, w, m, dwell, lo, hi)
        spin *= target / (spin.sum() * dt)
        spin *= np.sign(delta)
        # Heading the spin has added by each sample (it acts over the interval after it).
        turned = np.concatenate([[0.0], np.cumsum(spin)[:-1] * dt])

        s = extend(s, m, dwell, s_node)
        v = extend(v, m, dwell, 0.0)
        w = extend(w, m, dwell, 0.0) + spin
        h_path = extend(h, m, dwell, h[m - 1])
        c = np.concatenate([c[:m], np.repeat(c[m - 1 : m], dwell, axis=0), c[m:]])
        out_side = np.arange(n) >= m + dwell
        # Before the node the body is ahead of the spline by what has turned; after it,
        # the spline already carries the whole turn, so the body lags it by the rest.
        offset = turned - np.where(out_side, delta, 0.0)

        # Integrate the pose: each step covers the spline's displacement, but along the
        # body's actual heading. Profile headings are the negated canvas angle.
        pos = np.zeros_like(c)
        pos[0] = c[0]
        for k in range(1, n):
            if out_side[k] == out_side[k - 1]:
                off = 0.5 * (offset[k] + offset[k - 1])
            else:
                off = offset[k]
            step = c[k] - c[k - 1]
            cs, sn = math.cos(-off), math.sin(-off)
            pos[k] = pos[k - 1] + (cs * step[0] - sn * step[1], sn * step[0] + cs * step[1])

        gap = pos[-1] - c[-1]
        travelled = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))])
        s_cusp = travelled[m + dwell] if m + dwell < n else travelled[-1]
        s_total = travelled[-1]
        shift = np.zeros_like(pos)
        slope = np.zeros_like(pos)
        if s_total > 1e-9:
            share_in = gap * (s_cusp / s_total)
            share_out = gap * ((s_total - s_cusp) / s_total)
            if s_cusp > 1e-9:
                f, df = _smoothstep(travelled / s_cusp)
                before = ~out_side
                shift[before] = np.outer(f[before], share_in)
                slope[before] = np.outer(df[before] / s_cusp, share_in)
            if s_total - s_cusp > 1e-9:
                f, df = _smoothstep((travelled - s_cusp) / (s_total - s_cusp))
                shift[out_side] = share_in + np.outer(f[out_side], share_out)
                slope[out_side] = np.outer(df[out_side] / (s_total - s_cusp), share_out)
        pos = pos - shift

        # Moving along the shifted path: velocity v * u - |v| * slope, so the body points
        # along u - sign(v) * slope and the speed scales by that vector's length.
        heading = h_path + offset
        body = np.column_stack([np.cos(-heading), np.sin(-heading)])
        body = body - np.sign(v)[:, None] * slope
        stretch = np.linalg.norm(body, axis=1)
        corrected = -np.arctan2(body[:, 1], body[:, 0])
        tweak = (corrected - heading + math.pi) % (2 * math.pi) - math.pi
        # The two sides were unwrapped apart and can sit 2 pi apart across the node, which
        # the slowdown below would interpolate straight through.
        heading = np.unwrap(heading + tweak)
        w = w + (np.gradient(tweak, dt) if n > 1 else 0.0)
        v = v * stretch

        # The shift bends and speeds up the path a little; keep every wheel in budget, and
        # the acceleration within what the region asked for before (slowing down by f
        # divides acceleration by f^2). The spin itself stays inside its envelope
        # |v| / max_vel + |w| / turn_vel <= 1, which the shift can also push past.
        outer = np.max(np.abs(v) + np.abs(w) * half_tw)
        accel_was = max(np.max(np.abs(np.diff(v0))) / dt if len(v0) > 1 else 0.0, max_lin_acc)
        accel_now = np.max(np.abs(np.diff(v))) / dt if n > 1 else 0.0
        spinning = spin != 0.0
        envelope = (
            np.max(np.abs(v[spinning]) / constraints.max_vel + np.abs(w[spinning]) / turn_vel)
            if spinning.any() else 0.0
        )
        factor = max(outer / constraints.max_vel, math.sqrt(accel_now / accel_was), envelope, 1.0)
        new_split = first + m
        if factor > 1.0:
            v, w, heading, pos, s, new_t = _uniform_slowdown(v, w, heading, pos, s, factor, dt)
            new_split = first + int(np.searchsorted(new_t, m * dt * factor, side="left"))
            n = len(v)
        heading = (heading + math.pi) % (2 * math.pi) - math.pi
        return n, v, w, heading, pos, s, new_split

    # Each ramp fraction is judged on the region it finally produces: a candidate
    # that is shortest before the pose shift can need the largest uniform slowdown
    # after it, and picking on the pre-shift length made raising a limit slow
    # skills5 by up to 0.65 s (2026-10-31).
    best = None
    for fraction in SPIN_THROUGH_RAMP_FRACTIONS:
        s, v, h, c, m = _retime_ramps(s0, v0, h0, c0, m0, s_node, fraction, constraints, dt)
        w = w0 if fraction == 1.0 and len(v) == len(v0) else path_rates(h, m)
        dwell = shortest_dwell(v, w, m)
        result = finish(s, v, w, h, c, m, dwell)
        if best is None or result[0] < best[0][0]:
            best = (result, h, m)
    (n, v, w, heading, pos, s, new_split), h, m = best

    jump = (h[m] - h[m - 1] - delta + math.pi) % (2 * math.pi) - math.pi
    if abs(jump) > 0.05:
        logger.warning(
            f"spin-through node: path heading changes by {delta + jump:.3f} rad across "
            f"the node, not the turn's {delta:.3f}"
        )

    t0 = lists["times"][first]
    added = n - (last - first)
    lists["times"][first:last] = [t0 + k * dt for k in range(n)]
    for k in range(first + n, len(lists["times"])):
        lists["times"][k] += added * dt
    lists["positions"][first:last] = s.tolist()
    lists["linear_vels"][first:last] = v.tolist()
    lists["angular_vels"][first:last] = w.tolist()
    lists["headings"][first:last] = heading.tolist()
    lists["accelerations"][first:last] = [0.0] * n
    lists["coords"][first:last] = list(pos)

    new_s = s

    def remap(index: int) -> int:
        if index >= last:
            return index + added
        if index == split:
            return new_split
        # Same distance along the path, on the same side of the node.
        if index < split:
            k = int(np.searchsorted(new_s[: new_split - first], s0[index - first] - 1e-9))
        else:
            k = (new_split - first) + int(
                np.searchsorted(new_s[new_split - first :], s0[index - first] - 1e-9)
            )
        return first + min(k, n - 1)

    return remap


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
        if (
            node.is_reverse_node or node.turn != 0 or node.wait_time > 0
        ) and not node.stop:
            logger.warning(
                f"Node {i} reverses/turns/waits but has its stop overridden off; the "
                f"profile will step discontinuously through it."
            )

    # Rebuild spline tables
    sm_start_time = time.time()
    spline_manager.rebuild_tables()
    sm_end_time = time.time()
    logger.info(f"Spline table rebuild took {sm_end_time - sm_start_time} seconds")

    # Generate velocity profile. The path ends at rest unless the final node's stop
    # was overridden off - Node.auto_stop() counts the end node, so this is opt-in
    # per route and the default is unchanged.
    stops_at_end = spline_manager.nodes[-1].stop
    if not stops_at_end:
        logger.info("End node stop overridden off; profile will end at speed")

    fb_start_time = time.time()
    velocities, node_samples, sample_dists = forward_backward_pass(
        spline_manager, constraints, dd, end_vel=0.01 if stops_at_end else None
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
    # (node index, sample index of the node) for spin-through turns
    blends = []
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

        first = spline_manager.nodes[0]
        ins_headings, ins_ang_vels = motion_profile_angle(
            angle, constraints, dt, first.max_velocity, first.max_acceleration
        )

        # No samples exist yet, so the starting pose comes from the path itself
        start_heading = -1 * spline_manager.get_heading(0)
        if is_reversed:
            start_heading -= math.pi
        start_heading = ((start_heading + math.pi) % (2 * math.pi)) - math.pi
        start_coord = spline_manager.get_point_at_parameter(0)

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
        coords.extend(start_coord for _ in ins_headings)
        times.extend(current_time + i * dt for i in range(len(ins_headings)))

        current_time += len(ins_headings) * dt

    if spline_manager.nodes[node_idx].wait_time > 0:
        steps = int(spline_manager.nodes[node_idx].wait_time / dt)
        if headings:
            # A first-node turn already ran don't overwrite the heading
            h = headings[-1]
        else:
            h = -1 * spline_manager.get_heading(0)
            if is_reversed:
                h -= math.pi
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

    sample_times = np.asarray(sample_times)

    profile_duration = sample_times[-1]

    def handle_turn(angle, start_heading, current_time, dt, node):
        ins_headings, ins_ang_vels = motion_profile_angle(
            angle, constraints, dt, node.max_velocity, node.max_acceleration
        )

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
        current_pos, current_vel = sample_profile_state(
            elapsed, sample_times, sample_dists, velocities
        )
        current_pos = min(current_pos, total_length)

        t = spline_manager.distance_to_time(current_pos)

        # Handle node transitions
        while (
            node_idx + 1 in node_dists and current_pos >= node_dists[node_idx + 1]
        ):
            nodes_map.append(len(times))
            node_idx += 1
            current_node = spline_manager.nodes[node_idx]

            # Handle turn if present. A spin-through turn is laid over the ramps
            # once the whole path is sampled; here it only marks where.
            if blends_turn(current_node, node_idx, len(spline_manager.nodes)):
                blends.append((node_idx, len(times)))
            elif current_node.turn != 0:
                current_time = handle_turn(
                    np.radians(current_node.turn), headings[-1], current_time, dt,
                    current_node,
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

    # Ensure that the last nodes/action points at very end of path get their actions executed
    while node_idx + 1 < len(spline_manager.nodes):
        nodes_map.append(len(times))
        node_idx += 1
        current_node = spline_manager.nodes[node_idx]

        if current_node.turn != 0 and times:
            current_time = handle_turn(
                np.radians(current_node.turn), headings[-1], current_time, dt,
                current_node,
            )

        if current_node.is_reverse_node:
            is_reversed = not is_reversed

        if current_node.wait_time > 0 and times:
            current_time = handle_wait(current_node.wait_time, current_time, dt)

    # Spin-through turns, last first so inserting dwell samples leaves the sample
    # indices of the earlier ones alone. Each blends across the samples from the
    # previous node to the next one.
    lists = {
        "times": times, "positions": positions, "linear_vels": linear_vels,
        "accelerations": accelerations, "headings": headings,
        "angular_vels": angular_vels, "coords": coords,
    }
    for node_i, split in reversed(blends):
        first = nodes_map[node_i - 1]
        last = min(nodes_map[node_i + 1], len(times))
        node = spline_manager.nodes[node_i]
        delta = -np.radians(node.turn)
        if not first < split < last:
            # Nothing to overlap on one side: spin in place at the node instead.
            logger.warning(f"Node {node_i}: no segment to spin through; turning in place")
            ins_headings, ins_rates = motion_profile_angle(
                np.radians(node.turn), constraints, dt, node.max_velocity, node.max_acceleration
            )
            base = headings[split - 1]
            dwell = len(ins_headings)
            for name, fill in (
                ("positions", [positions[split - 1]] * dwell),
                ("linear_vels", [0.0] * dwell),
                ("accelerations", [0.0] * dwell),
                ("headings", [((base + h + math.pi) % (2 * math.pi)) - math.pi for h in ins_headings]),
                ("angular_vels", list(ins_rates)),
                ("coords", [coords[split - 1]] * dwell),
                ("times", [times[split - 1] + (k + 1) * dt for k in range(dwell)]),
            ):
                lists[name][split:split] = fill
            for k in range(split + dwell, len(times)):
                times[k] += dwell * dt
            for k in range(node_i + 1, len(nodes_map)):
                nodes_map[k] += dwell
            for k in range(len(actions_map)):
                if actions_map[k] >= split:
                    actions_map[k] += dwell
        else:
            remap = blend_spin_through(
                node, lists, first, split, last, node_dists[node_i], delta,
                constraints, dt,
            )
            for k in range(node_i, len(nodes_map)):
                nodes_map[k] = remap(nodes_map[k])
            for k in range(len(actions_map)):
                if actions_map[k] >= first:
                    actions_map[k] = remap(actions_map[k])

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


@dataclass
class FeasibilityReport:
    """What a generated trajectory asks of the chassis.

    A corner the profiler lets through is only as feasible as its constraints are
    honest, so this reports the quantities the yaw budget exists to bound, and warns
    when the trajectory is outside what the config says the robot can do. Units are
    the profiler's: feet, seconds, radians.
    """

    duration: float
    peak_speed: float
    peak_yaw_rate: float
    # Finite-differenced on the 10 ms grid, so a spline split with a curvature step
    # shows up here as a spike the robot never has to produce. Informational only.
    peak_yaw_accel: float
    # Over samples that are actually moving; in-place turns are excluded. inf if the
    # path never turns.
    min_turn_radius: float
    peak_outer_wheel_speed: float
    samples_at_yaw_cap: int
    warnings: List[str]

    def summary(self) -> str:
        radius = (
            "straight"
            if math.isinf(self.min_turn_radius)
            else f"{self.min_turn_radius:.2f} ft"
        )
        return (
            f"duration {self.duration:.2f} s, peak speed {self.peak_speed:.2f} ft/s, "
            f"peak yaw {self.peak_yaw_rate:.2f} rad/s, peak yaw accel "
            f"{self.peak_yaw_accel:.1f} rad/s^2, min radius {radius}, "
            f"peak outer wheel {self.peak_outer_wheel_speed:.2f} ft/s, "
            f"{self.samples_at_yaw_cap} samples at the yaw cap"
        )


def spin_through_samples(nodes, nodes_map, sample_count: int) -> np.ndarray:
    """Which samples of a generated profile belong to a spin-through turn's region
    (previous node to next node), for check_trajectory_feasibility."""
    mask = np.zeros(sample_count, dtype=bool)
    for i, node in enumerate(nodes):
        if blends_turn(node, i, len(nodes)) and i + 1 < len(nodes_map):
            mask[nodes_map[i - 1] : nodes_map[i + 1]] = True
    return mask


def check_trajectory_feasibility(
    times: List[float],
    linear_vels: List[float],
    angular_vels: List[float],
    constraints: Constraints,
    moving_speed: float = 0.1,
    spin_through: Optional[np.ndarray] = None,
) -> FeasibilityReport:
    """Audit a generated trajectory against the constraints it was built under.

    moving_speed (ft/s) is the floor below which a sample counts as an in-place turn
    or a stop and is left out of the turning-radius figure. spin_through marks the
    samples of spin-through regions (spin_through_samples).
    """
    t = np.asarray(times, dtype=float)
    v = np.asarray(linear_vels, dtype=float)
    w = np.asarray(angular_vels, dtype=float)
    if len(t) == 0:
        return FeasibilityReport(0.0, 0.0, 0.0, 0.0, float("inf"), 0.0, 0, [])

    abs_v, abs_w = np.abs(v), np.abs(w)
    half_tw = constraints.track_width / 2

    yaw_accel = 0.0
    if len(t) > 1:
        dt = np.diff(t)
        ok = dt > 1e-9
        if ok.any():
            yaw_accel = float(np.max(np.abs(np.diff(w)[ok] / dt[ok])))

    moving = abs_v >= moving_speed
    # A spin-through turn spins while still creeping (blend_spin_through). Inside its
    # regions, samples within the spin's envelope |v| / max_vel + |w| /
    # spin_through_velocity <= 1 are held to the spin's limits, not the path's.
    if spin_through is not None and len(spin_through) == len(v):
        turn_cap = constraints.spin_through_rate_cap()
        inside = abs_v / constraints.max_vel + abs_w / turn_cap <= 1.02
        moving = moving & ~(np.asarray(spin_through, dtype=bool) & inside)
    turning = moving & (abs_w > 1e-6)
    min_radius = float(np.min(abs_v[turning] / abs_w[turning])) if turning.any() else float("inf")

    # The path's yaw cap binds while driving; an in-place turn has its own
    # (max_turn_velocity). Each sample is held to the cap for its outer side: the right
    # wheel is outer when v * w > 0 (get_wheel_speeds).
    side_caps = {
        side: constraints.yaw_rate_cap(side) or float("inf") for side in ("left", "right")
    }
    cap_per_sample = np.where(v * w > 0, side_caps["right"], side_caps["left"])
    capped = np.isfinite(cap_per_sample)
    at_cap = int(np.count_nonzero(moving & capped & (abs_w >= 0.98 * cap_per_sample)))

    warnings: List[str] = []
    peak_w = float(abs_w.max())
    peak_driving_w = float(abs_w[moving].max()) if moving.any() else 0.0
    for side in ("left", "right"):
        cap = side_caps[side]
        on_side = moving & ((v * w > 0) == (side == "right"))
        peak_side = float(abs_w[on_side].max()) if on_side.any() else 0.0
        if np.isfinite(cap) and peak_side > 1.02 * cap:
            warnings.append(
                f"peak yaw rate {peak_side:.2f} rad/s exceeds max_angular_velocity "
                f"{cap:.2f} ({side} wheel outer)"
            )
    spin_cap = constraints.max_turn_velocity
    peak_spin_w = float(abs_w[~moving].max()) if (~moving).any() else 0.0
    if spin_cap and peak_spin_w > 1.02 * spin_cap:
        warnings.append(
            f"peak turn-in-place rate {peak_spin_w:.2f} rad/s exceeds max_turn_velocity {spin_cap:.2f}"
        )
    if min_radius < half_tw:
        warnings.append(
            f"minimum turning radius {min_radius:.2f} ft is inside the half-track "
            f"({half_tw:.2f} ft): the inner wheel reverses and the outer wheels drag the "
            f"chassis around a pivot"
        )
    peak_outer = float(np.max(abs_v + abs_w * half_tw))
    if peak_outer > 1.02 * constraints.max_vel:
        warnings.append(
            f"peak outer wheel speed {peak_outer:.2f} ft/s exceeds max_velocity "
            f"{constraints.max_vel:.2f}"
        )

    return FeasibilityReport(
        duration=float(t[-1] - t[0]),
        peak_speed=float(abs_v.max()),
        peak_yaw_rate=peak_w,
        peak_yaw_accel=yaw_accel,
        min_turn_radius=min_radius,
        peak_outer_wheel_speed=peak_outer,
        samples_at_yaw_cap=at_cap,
        warnings=warnings,
    )
