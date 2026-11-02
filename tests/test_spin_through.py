"""Spin-through turns: the spin overlaps the brake into a turn node and the launch out.

The case they exist for is straight_cycle's: back into the goal, drive toward the
matchloader, flip 180 deg on the way and back into it, then the same home. A reverse
node with a 180 deg turn keeps the path going the same way while the body flips. As a
stop, spin in place, go, that flip costs a full stop and a 1 s spin; overlapped, the
robot passes through zero speed already spinning (2026-10-26 model: ~0.6 s a leg).
"""
import copy
import math

import numpy as np
import pytest

from motion_profiling_v2 import motion_profile_generator as mpg
from utilities import headless_profile as hp
from utilities import route_serializer as rs

# config.yaml's motion section as of 2026-10-26
MOTION = {
    "max_velocity": 3.5,
    "max_acceleration": 12.0,
    "max_deceleration": 12.0,
    "max_jerk": 16.0,
    "stall_accel": 12.0,
    "free_speed": 4.8,
    "traction_accel": 12.0,
    "steering_speed_penalty": 0.28,
    "max_angular_velocity": 1.8,
    "max_angular_acceleration": 7.0,
    "max_turn_velocity": 7.0,
    "max_turn_acceleration": 12.0,
}
TRACK_WIDTH_IN = 11.35
STOPS = (0, 2, 4)  # goal, matchloader, goal


def _cycle(spin_through=True, turn=180.0, **flip):
    """Goal at x=0 to a matchloader 3 ft away in -x and back, flipping halfway each way."""
    nodes = [
        hp.HeadlessNode(0.0, 0.0, is_start_node=True),
        hp.HeadlessNode(-1.5, 0.0, is_reverse_node=True, turn=turn, spin_through=spin_through, **flip),
        hp.HeadlessNode(-3.0, 0.0, is_reverse_node=True),
        hp.HeadlessNode(-1.5, 0.0, is_reverse_node=True, turn=turn, spin_through=spin_through, **flip),
        hp.HeadlessNode(0.0, 0.0, is_end_node=True),
    ]
    for i, direction in ((0, (-1.0, 0.0)), (2, (-1.0, 0.0)), (4, (1.0, 0.0))):
        nodes[i].tangent = np.array(direction)
        nodes[i].curvature = np.zeros(2)
        nodes[i].incoming_magnitude = nodes[i].outgoing_magnitude = 1.5
    return nodes


def _profile(nodes):
    manager = hp.build_spline_manager(nodes, [])
    return hp.profile_route(manager, hp.constraints_from_config(MOTION, TRACK_WIDTH_IN))


def _arrays(result):
    xy = np.array([np.asarray(c, dtype=float).ravel()[:2] for c in result.coords])
    return (
        np.asarray(result.times),
        xy,
        np.unwrap(np.asarray(result.headings)),
        np.asarray(result.linear_vels),
        np.asarray(result.angular_vels),
    )


def test_spin_through_beats_stopping_to_spin():
    in_place = _profile(_cycle(spin_through=False)).times[-1]
    blended = _profile(_cycle()).times[-1]
    assert blended < in_place - 0.5, (blended, in_place)


def test_spin_through_is_drivable():
    result = _profile(_cycle())
    t, xy, h, v, w = _arrays(result)
    dt = np.diff(t)
    assert np.allclose(dt, 0.01, atol=1e-9)

    # A differential drive only moves along its heading: no sideways speed, and the
    # heading changes by exactly what the yaw rate says.
    mid = -0.5 * (h[1:] + h[:-1])
    step = np.diff(xy, axis=0)
    sideways = np.abs(np.cos(mid) * step[:, 1] - np.sin(mid) * step[:, 0]) / dt
    assert sideways.max() < 0.05
    assert np.abs(np.diff(h) - 0.5 * (w[1:] + w[:-1]) * dt).max() < 0.01

    c = hp.constraints_from_config(MOTION, TRACK_WIDTH_IN)
    outer = np.abs(v) + np.abs(w) * c.track_width / 2
    assert outer.max() <= 1.01 * c.max_vel
    spinning = np.abs(w) > c.max_angular_velocity
    envelope = np.abs(v) / c.max_vel + np.abs(w) / c.max_turn_velocity
    assert envelope[spinning].max() <= 1.02
    assert result.report.warnings == []


def test_spin_through_still_hits_the_stops_square():
    blended = _profile(_cycle())
    in_place = _profile(_cycle(spin_through=False))
    _, xy, h, _, _ = _arrays(blended)
    _, _, h_ref, _, _ = _arrays(in_place)
    nodes = _cycle()
    for i in STOPS:
        k = min(blended.nodes_map[i], len(xy) - 1)
        k_ref = min(in_place.nodes_map[i], len(h_ref) - 1)
        assert np.linalg.norm(xy[k] - (nodes[i].x, nodes[i].y)) * 12 < 0.05
        error = (h[k] - h_ref[k_ref] + math.pi) % (2 * math.pi) - math.pi
        assert abs(error) < math.radians(1.0)


def test_spin_through_turns_while_moving():
    _, _, _, v, w = _arrays(_profile(_cycle()))
    c = hp.constraints_from_config(MOTION, TRACK_WIDTH_IN)
    # Spinning well past what any path corner allows while still rolling at 6+ in/s.
    assert np.any((np.abs(w) > 1.5 * c.max_angular_velocity) & (np.abs(v) > 0.5))


def test_maps_stay_in_order():
    result = _profile(_cycle())
    assert list(result.nodes_map) == sorted(result.nodes_map)
    assert result.nodes_map[-1] <= len(result.times)


@pytest.mark.parametrize(
    "flip",
    [{"wait_time": 0.5}, {"actions": {"wait_lift": 1.0}}],
    ids=["wait", "lift wait"],
)
def test_a_node_that_waits_spins_in_place(flip):
    blended = _profile(_cycle(**flip)).times[-1]
    in_place = _profile(_cycle(spin_through=False, **flip)).times[-1]
    assert blended == pytest.approx(in_place)


def test_spin_through_without_a_turn_changes_nothing():
    assert _profile(_cycle(turn=0.0)).times[-1] == pytest.approx(
        _profile(_cycle(spin_through=False, turn=0.0)).times[-1]
    )


def test_spin_through_round_trips_through_the_route_file():
    route = rs.Route(nodes=[rs.NodeRecord(x=0, y=0, is_start=True),
                            rs.NodeRecord(x=10, y=0, turn=180, spin_through=True),
                            rs.NodeRecord(x=20, y=0, is_end=True)])
    text = rs.dumps_route(route, [])
    parsed = rs.parse_route(text, [])
    assert [n.spin_through for n in parsed.nodes] == [False, True, False]
    assert not parsed.warnings
    plain = copy.deepcopy(route)
    plain.nodes[1].spin_through = False
    assert "spin_through" not in rs.dumps_route(plain, [])


def test_spin_through_caps_replace_the_in_place_ones():
    """spin_through_velocity / _acceleration bound the flip, not max_turn_*: the in-place
    limits are measured from rest and saturated both sides on the first flight
    (firmware card boots 319/323, 2026-10-30)."""
    derated = {**MOTION, "spin_through_velocity": 4.0, "spin_through_acceleration": 7.0}
    manager = hp.build_spline_manager(_cycle(), [])
    c = hp.constraints_from_config(derated, TRACK_WIDTH_IN)
    result = hp.profile_route(manager, c)
    _, _, _, v, w = _arrays(result)
    spinning = np.abs(w) > c.max_angular_velocity
    envelope = np.abs(v) / c.max_vel + np.abs(w) / c.spin_through_velocity
    assert envelope[spinning].max() <= 1.02
    assert np.abs(w).max() <= 1.02 * c.spin_through_velocity
    assert result.report.warnings == []
    # Slower than the in-place caps allow, still quicker than stopping to spin.
    assert result.times[-1] > _profile(_cycle()).times[-1]
    assert result.times[-1] < _profile(_cycle(spin_through=False)).times[-1]


# config.yaml's motion section as of 2026-10-31: derated spin-through caps.
MOTION_1031 = {**MOTION, "spin_through_velocity": 4.0, "spin_through_acceleration": 8.0}

# routes/curved_cycle_spin.txt (2026-10-26): goal -> loader -> goal with two curved
# flips, the shape of skills5's cycles. Routes are gitignored, so it lives here.
CURVED_CYCLE = """{
  "format": "vap-route", "version": 2, "actions": [],
  "nodes": [
    {"x":-47.465,"y":31.114,"start":true,"tangent":[0,1],"tangent_derivative":[0,0],"incoming_magnitude":1.427104,"outgoing_magnitude":1.1916},
    {"x":-51.06,"y":45.947,"reverse":true,"turn":151.9,"spin_through":true,"tangent":[-0.621944,0.783062],"tangent_derivative":[0,0],"incoming_magnitude":0.9753,"outgoing_magnitude":1.0432},
    {"x":-64.086,"y":59.743,"reverse":true,"tangent":[-0.981627,0.190809],"tangent_derivative":[-0.195395,-0.460172],"incoming_magnitude":1.8994,"outgoing_magnitude":1.5604},
    {"x":-52.443,"y":48.736,"reverse":true,"turn":174.2,"spin_through":true,"tangent":[0.34373,-0.939069],"tangent_derivative":[0,0],"incoming_magnitude":1.6589,"outgoing_magnitude":0.6044},
    {"x":-47.522,"y":31.064,"end":true,"tangent":[0,-1],"tangent_derivative":[-0.0625,0],"incoming_magnitude":1.4954,"outgoing_magnitude":1.454115}
  ],
  "action_points": []
}"""


def _curved_cycle():
    return hp.nodes_from_route(rs.parse_route(CURVED_CYCLE, []))[0]


ROUTES = {"straight": _cycle, "curved": _curved_cycle}


def _duration(make_nodes, motion):
    manager = hp.build_spline_manager(make_nodes(), [])
    return hp.profile_route(manager, hp.constraints_from_config(motion, TRACK_WIDTH_IN)).times[-1]


@pytest.mark.parametrize("motion", [MOTION, MOTION_1031], ids=["in-place caps", "spin caps"])
@pytest.mark.parametrize("route", sorted(ROUTES))
def test_blend_keeps_the_quickest_ramp_fraction(route, motion, monkeypatch):
    """Every ramp fraction is judged on the region it finally produces, after the pose
    shift and any uniform slowdown. Judging on the pre-shift length picked candidates
    the slowdown then stretched, so the full ladder came out slower than one of its own
    rungs alone (straight 4.20 vs 4.16 s, curved 4.28 vs 4.25 s, 2026-10-31)."""
    ladder = mpg.SPIN_THROUGH_RAMP_FRACTIONS
    full = _duration(ROUTES[route], motion)
    for fraction in ladder:
        monkeypatch.setattr(mpg, "SPIN_THROUGH_RAMP_FRACTIONS", (fraction,))
        assert full <= _duration(ROUTES[route], motion) + 1e-9, fraction


# Each raise is above MOTION_1031's value.
RAISED = {
    "max_acceleration": 14.0,
    "max_deceleration": 14.0,
    "stall_accel": 16.0,
    "traction_accel": 16.0,
    "max_velocity": 4.0,
    "max_angular_velocity": 2.6,
    "max_angular_acceleration": 12.0,
    "spin_through_velocity": 5.0,
    "spin_through_acceleration": 10.0,
}

# The blend only gets the yaw budget the first-pass profile left at the cusp, and a
# route planned faster into a flip leaves less of it, so a raised limit can still
# cost a little on a flip (worst here: curved, max_angular_velocity, +0.12 s). Going
# below this needs the first pass to reserve budget near spin-through nodes.
RAISE_TOLERANCE_S = 0.15


@pytest.mark.parametrize("limit", sorted(RAISED))
@pytest.mark.parametrize("route", sorted(ROUTES))
def test_raising_a_limit_does_not_slow_a_route(route, limit):
    """A looser constraint should never plan a slower route. Before 2026-10-31 it did:
    stall_accel 12 -> 16 slowed skills5 by 0.65 s, all on legs touching a flip."""
    base = _duration(ROUTES[route], MOTION_1031)
    raised = _duration(ROUTES[route], {**MOTION_1031, limit: RAISED[limit]})
    assert raised <= base + RAISE_TOLERANCE_S, (base, raised)


@pytest.mark.xfail(strict=False, reason=(
    "two-pass blend: with the in-place turn caps (7/12) the curved flips lose ~0.4 s "
    "to stall_accel 16; needs the first pass to reserve yaw budget at spin nodes"))
def test_raising_stall_accel_with_in_place_caps():
    base = _duration(_curved_cycle, MOTION)
    raised = _duration(_curved_cycle, {**MOTION, "stall_accel": 16.0})
    assert raised <= base + RAISE_TOLERANCE_S, (base, raised)
