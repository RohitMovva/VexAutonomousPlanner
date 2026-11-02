"""The yaw budget: corners the wheel-speed bound calls feasible but the chassis
cannot turn (OverrideAutonomous log 09122026-9) must be slowed by the profiler."""
import math

import numpy as np
import pytest

from motion_profiling_v2 import motion_profile_generator as mpg
from utilities import headless_profile as hp

TW = 1.0  # ft, the firmware's 12.0 in
YAW_CAP = 1.5  # rad/s
YAW_ACCEL_CAP = 5.0  # rad/s^2


def make_constraints(**overrides):
    base = dict(
        max_vel=3.5, max_acc=12.0, max_dec=12.0, friction_coef=0.8, max_jerk=16.0,
        track_width=TW, stall_accel=7.5, free_speed=4.8, traction_accel=12.0,
        steering_speed_penalty=0.4,
    )
    base.update(overrides)
    return mpg.Constraints(**base)


# ------------------------------------------------------------------ unit bounds

def test_yaw_rate_cap_binds_on_tight_curvature_only():
    uncapped = make_constraints()
    capped = make_constraints(max_angular_velocity=YAW_CAP)
    # Gentle arc: the wheel-speed bound binds and the cap changes nothing.
    assert capped.max_speed_at_curvature(0.2) == uncapped.max_speed_at_curvature(0.2)
    # Pivot-tight corner (radius 0.5 ft): omega = v * k must stay under the cap.
    k = 2.0
    assert capped.max_speed_at_curvature(k) == pytest.approx(YAW_CAP / k)
    assert capped.max_speed_at_curvature(k) < uncapped.max_speed_at_curvature(k)
    # Straight: unbounded, so max_vel is what remains.
    assert capped.max_speed_at_curvature(0.0) == uncapped.max_vel
    assert math.isinf(capped.max_speed_by_yaw_rate(0.0))


def test_yaw_accel_cap_limits_velocity_through_a_curvature_ramp():
    uncapped = make_constraints()
    capped = make_constraints(max_angular_acceleration=YAW_ACCEL_CAP)
    dkappads = 20.0
    assert capped.limit_velocity_by_ang_accel(dkappads) <= uncapped.limit_velocity_by_ang_accel(dkappads)
    assert capped.limit_velocity_by_ang_accel(dkappads) <= math.sqrt(YAW_ACCEL_CAP / dkappads)
    assert capped.limit_velocity_by_ang_accel(0.0) == capped.max_vel


def test_max_accel_by_yaw_accel_forms():
    c = make_constraints(max_angular_acceleration=YAW_ACCEL_CAP)
    # No bend at all: nothing to accelerate.
    assert math.isinf(c.max_accel_by_yaw_accel(2.0, 0.0, 0.0))
    # At the sample: a <= (alpha - v^2 k') / k.
    v, k, kp = 1.0, 2.0, 1.0
    assert c.max_accel_by_yaw_accel(v, k, kp) == pytest.approx((YAW_ACCEL_CAP - v * v * kp) / k)
    # Across an interval the far end charges the extra speed: strictly tighter.
    assert c.max_accel_by_yaw_accel(v, k, kp, ds=0.1) < c.max_accel_by_yaw_accel(v, k, kp)
    # Never negative.
    assert c.max_accel_by_yaw_accel(10.0, k, kp) == 0.0
    # Off: unbounded.
    assert math.isinf(make_constraints().max_accel_by_yaw_accel(v, k, kp))


def test_in_place_turns_are_not_under_the_yaw_caps():
    """A spin in place loads all four motors symmetrically; the path's yaw caps were
    measured on a one-sided pivot and do not apply (a spin has its own, below)."""
    capped = make_constraints(max_angular_velocity=YAW_CAP, max_angular_acceleration=YAW_ACCEL_CAP)
    _, capped_w = mpg.motion_profile_angle(math.pi / 2, capped, dt=0.01)
    _, plain_w = mpg.motion_profile_angle(math.pi / 2, make_constraints(), dt=0.01)
    assert capped_w == pytest.approx(plain_w)


TURN_CAP = 4.3  # rad/s, logs 09222026-164 and 09242026-289 at full voltage
TURN_ACCEL_CAP = 7.0  # rad/s^2


def test_in_place_turns_have_their_own_caps():
    """Uncapped, a 180 deg spin planned at 7 rad/s in 0.7 s; the robot does ~4.3 rad/s
    and left every turn 40-70 deg behind."""
    plain_h, plain_w = mpg.motion_profile_angle(math.pi, make_constraints(), dt=0.01)
    c = make_constraints(max_turn_velocity=TURN_CAP, max_turn_acceleration=TURN_ACCEL_CAP)
    h, w = mpg.motion_profile_angle(math.pi, c, dt=0.01)
    assert max(abs(x) for x in plain_w) > 6.0
    assert max(abs(x) for x in w) <= TURN_CAP + 1e-6
    assert np.abs(np.diff(w) / 0.01).max() <= TURN_ACCEL_CAP + 0.5
    assert abs(h[-1]) == pytest.approx(math.pi, abs=0.05)
    # Trapezoid at 4.3 rad/s and 7 rad/s^2: pi / 4.3 + 4.3 / 7 = 1.35 s.
    assert len(h) * 0.01 == pytest.approx(math.pi / TURN_CAP + TURN_CAP / TURN_ACCEL_CAP, abs=0.05)


def test_turn_node_overrides_replace_the_spin_caps():
    """A turn node's max_velocity / max_acceleration (wheel ft/s, ft/s^2) set its spin,
    slower or faster than the config caps: the spin-rate test routes rely on both."""
    c = make_constraints(max_turn_velocity=TURN_CAP, max_turn_acceleration=TURN_ACCEL_CAP)
    half = c.track_width / 2
    _, slow = mpg.motion_profile_angle(4 * math.pi, c, 0.01, 1.0 * half, 3.0 * half)
    assert max(abs(x) for x in slow) == pytest.approx(1.0, abs=0.01)
    _, fast = mpg.motion_profile_angle(4 * math.pi, c, 0.01, 6.0 * half, 12.0 * half)
    assert max(abs(x) for x in fast) == pytest.approx(6.0, abs=0.05)
    assert np.abs(np.diff(fast) / 0.01).max() == pytest.approx(12.0, abs=0.5)
    # No override: the config caps, as before
    _, plain = mpg.motion_profile_angle(4 * math.pi, c, 0.01)
    assert max(abs(x) for x in plain) == pytest.approx(TURN_CAP, abs=0.01)


def test_feasibility_judges_spins_and_driving_by_their_own_caps():
    c = make_constraints(max_angular_velocity=YAW_CAP, max_turn_velocity=TURN_CAP)
    t = [0.0, 0.01, 0.02, 0.03]
    # A 4 rad/s spin (v = 0) is fine for a 4.3 rad/s turn cap though it tops the path cap.
    report = mpg.check_trajectory_feasibility(t, [0, 0, 0, 0], [0, 4.0, 4.0, 0], c)
    assert not report.warnings
    assert report.samples_at_yaw_cap == 0
    # Faster than the turn cap: warned as a spin.
    report = mpg.check_trajectory_feasibility(t, [0, 0, 0, 0], [0, 5.0, 5.0, 0], c)
    assert [w for w in report.warnings if "max_turn_velocity" in w]
    assert not [w for w in report.warnings if "max_angular_velocity" in w]
    # 2 rad/s while driving is over the path cap.
    report = mpg.check_trajectory_feasibility(t, [0, 2.0, 2.0, 0], [0, 2.0, 2.0, 0], c)
    assert [w for w in report.warnings if "max_angular_velocity" in w]


# ------------------------------------------------------------------ end to end

def corner_path():
    """An L with a tight elbow: the shape of turn_test's corner."""
    nodes = [
        hp.HeadlessNode(0.0, 0.0, is_start_node=True),
        hp.HeadlessNode(2.0, 0.0),
        hp.HeadlessNode(2.0, 2.0, is_end_node=True),
    ]
    return hp.build_spline_manager(nodes)


def peak_yaw_on_distance_grid(manager, constraints):
    velocities, _, dists = mpg.forward_backward_pass(manager, constraints, 0.005)
    curvatures = manager.get_curvatures(manager.distances_to_times(dists))
    return float(np.max(np.abs(np.asarray(velocities) * np.asarray(curvatures))))


def test_corner_is_slowed_to_the_yaw_cap():
    manager = corner_path()
    # The test is only meaningful if the corner would otherwise exceed the cap.
    assert peak_yaw_on_distance_grid(manager, make_constraints()) > YAW_CAP
    capped = make_constraints(max_angular_velocity=YAW_CAP)
    assert peak_yaw_on_distance_grid(manager, capped) <= YAW_CAP * (1 + 1e-6)

    profiled = hp.profile_route(manager, capped)
    # The exported 10 ms trajectory carries omega = v * kappa; interpolation between
    # grid samples can overshoot by a hair, never by more than a couple of percent.
    assert profiled.report.peak_yaw_rate <= YAW_CAP * 1.02
    assert not any("yaw rate" in w for w in profiled.report.warnings)


def test_straight_route_is_untouched_by_the_caps():
    nodes = [hp.HeadlessNode(0.0, 0.0, is_start_node=True), hp.HeadlessNode(4.0, 0.0, is_end_node=True)]
    manager = hp.build_spline_manager(nodes)
    plain = hp.profile_route(manager, make_constraints())
    capped = hp.profile_route(
        manager, make_constraints(max_angular_velocity=YAW_CAP, max_angular_acceleration=YAW_ACCEL_CAP)
    )
    assert capped.linear_vels == pytest.approx(plain.linear_vels)
    assert math.isinf(capped.report.min_turn_radius)


# ------------------------------------------------------------------ report

def test_feasibility_report_flags_a_pivot_and_a_yaw_overrun():
    c = make_constraints(max_angular_velocity=YAW_CAP)
    times = [i * 0.01 for i in range(5)]
    # 1.2 ft/s at 2.4 rad/s: radius 0.5 ft = the half-track, so a pivot; over the cap.
    report = mpg.check_trajectory_feasibility(times, [1.2] * 5, [2.4] * 5, c)
    assert report.peak_yaw_rate == pytest.approx(2.4)
    assert report.min_turn_radius == pytest.approx(0.5)
    assert report.samples_at_yaw_cap == 5
    assert any("yaw rate" in w for w in report.warnings)
    assert any("inside the half-track" in w for w in report.warnings) is False  # exactly at, not inside

    report = mpg.check_trajectory_feasibility(times, [1.0] * 5, [2.4] * 5, c)
    assert any("inside the half-track" in w for w in report.warnings)
    assert "min radius 0.42 ft" in report.summary()


def test_feasibility_report_ignores_in_place_turns_for_radius():
    c = make_constraints(max_angular_velocity=YAW_CAP)
    times = [i * 0.01 for i in range(4)]
    report = mpg.check_trajectory_feasibility(times, [0.0] * 4, [1.0] * 4, c)
    assert math.isinf(report.min_turn_radius)
    assert report.warnings == []
    assert mpg.check_trajectory_feasibility([], [], [], c).duration == 0.0


# ------------------------------------------------------------------ per-side caps
# The outer wheel pays for the yaw, and the two sides need not have the same headroom
# (OverrideAutonomous logs 10192026-1012..1021: right outer saturated 73% at the cap,
# left outer 3%). Per-side overrides replace max_angular_velocity for that side.

def test_yaw_rate_cap_per_side_and_fallbacks():
    c = make_constraints(max_angular_velocity=1.8, max_angular_velocity_left_outer=2.5)
    assert c.yaw_rate_cap("left") == 2.5
    assert c.yaw_rate_cap("right") == 1.8
    # Direction unknown: the strictest, so nothing is ever overstated.
    assert c.yaw_rate_cap(None) == 1.8
    assert make_constraints(max_angular_velocity_right_outer=1.2).yaw_rate_cap(None) == 1.2
    assert make_constraints().yaw_rate_cap("left") is None
    k = 2.0
    assert c.max_speed_at_curvature(k, "left") > c.max_speed_at_curvature(k, "right")
    assert c.max_speed_at_curvature(k) == c.max_speed_at_curvature(k, "right")


def test_outer_side_follows_curvature_sign_and_direction():
    assert mpg.Constraints.outer_side(-1.0, False) == "right"
    assert mpg.Constraints.outer_side(1.0, False) == "left"
    assert mpg.Constraints.outer_side(1.0, True) == "right"
    assert mpg.Constraints.outer_side(-1.0, True) == "left"
    assert mpg.Constraints.outer_side(0.0, False) is None


def corner_outer_side(reverse: bool) -> str:
    """Which wheel the exported trajectory makes outer through corner_path's elbow,
    judged the way the firmware's wheel split does (right = v + w * tw/2)."""
    nodes = [
        hp.HeadlessNode(0.0, 0.0, is_start_node=True, is_reverse_node=reverse),
        hp.HeadlessNode(2.0, 0.0),
        hp.HeadlessNode(2.0, 2.0, is_end_node=True),
    ]
    manager = hp.build_spline_manager(nodes)
    p = hp.profile_route(manager, make_constraints(max_angular_velocity=YAW_CAP))
    v, w = np.asarray(p.linear_vels), np.asarray(p.angular_vels)
    peak = int(np.argmax(np.abs(w)))
    return ("right" if v[peak] * w[peak] > 0 else "left"), manager


@pytest.mark.parametrize("reverse", [False, True])
def test_per_side_cap_binds_only_turns_with_that_outer_side(reverse):
    side, manager = corner_outer_side(reverse)
    other = "left" if side == "right" else "right"
    sym = hp.profile_route(manager, make_constraints(max_angular_velocity=YAW_CAP))

    # Raising the cap for the side that is outer here speeds the corner up...
    faster = hp.profile_route(manager, make_constraints(
        max_angular_velocity=YAW_CAP, **{f"max_angular_velocity_{side}_outer": 2 * YAW_CAP}))
    assert faster.times[-1] < sym.times[-1] - 0.02
    assert faster.report.peak_yaw_rate > YAW_CAP * 1.05
    assert faster.report.peak_yaw_rate <= 2 * YAW_CAP * 1.02
    assert not any("yaw rate" in w for w in faster.report.warnings)

    # ...while raising the other side's changes nothing.
    untouched = hp.profile_route(manager, make_constraints(
        max_angular_velocity=YAW_CAP, **{f"max_angular_velocity_{other}_outer": 2 * YAW_CAP}))
    assert untouched.linear_vels == pytest.approx(sym.linear_vels)


def test_reversing_through_the_same_corner_swaps_the_outer_side():
    assert corner_outer_side(False)[0] != corner_outer_side(True)[0]


def test_feasibility_judges_each_sample_against_its_outer_sides_cap():
    c = make_constraints(max_angular_velocity=1.8, max_angular_velocity_left_outer=2.5)
    t = [0.0, 0.01, 0.02, 0.03]
    # v * w < 0: left outer, 2.2 rad/s is inside its 2.5 cap.
    report = mpg.check_trajectory_feasibility(t, [0, 2.0, 2.0, 0], [0, -2.2, -2.2, 0], c)
    assert not [w for w in report.warnings if "max_angular_velocity" in w]
    # v * w > 0: right outer, 2.2 is over its 1.8.
    report = mpg.check_trajectory_feasibility(t, [0, 2.0, 2.0, 0], [0, 2.2, 2.2, 0], c)
    assert [w for w in report.warnings if "max_angular_velocity" in w and "right" in w]
