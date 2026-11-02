"""The path micro-optimizer (utilities/path_optimizer.py, tools/optimize_route.py).

What has to hold for its answers to mean anything: writing every node's effective
settings back in rebuilds the identical spline, the stretches between stops add up to
the route's time, locked settings never move, and the route it returns is the one it
timed.
"""

import copy
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from motion_profiling_v2 import motion_profile_generator as mpg
from utilities import headless_profile as hp
from utilities import path_optimizer as po
from utilities import route_serializer as rs

REPO = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((REPO / "config.yaml").read_text())
ACTIONS = CONFIG["robot"]["actions"]
CONSTRAINTS = hp.constraints_from_config(CONFIG["motion"], CONFIG["robot"]["track_width"])
SUITE = sorted((REPO / "suite" / "routes").glob("*.txt"))


def tricky_route() -> rs.Route:
    """A reverse start node, a turn, and a zero out-handle on a reverse node that does
    not stop (the spline falls back to the chord there, which no explicit tangent can
    reproduce)."""
    return rs.Route(
        nodes=[
            rs.NodeRecord(x=-50, y=0, is_start=True, is_reverse=True, tangent=[1, 0],
                          incoming_magnitude=0, outgoing_magnitude=2),
            rs.NodeRecord(x=-20, y=10),
            rs.NodeRecord(x=0, y=30, turn=45),
            rs.NodeRecord(x=20, y=10, is_reverse=True, stop_mode=2, tangent=[1, -1],
                          incoming_magnitude=2, outgoing_magnitude=0),
            rs.NodeRecord(x=30, y=40, actions={"intake": 1.0}),
            rs.NodeRecord(x=10, y=50, is_end=True),
        ]
    )


def routes():
    out = [(p.stem, hp.load_route_file(p, ACTIONS)) for p in SUITE]
    return out + [("tricky", tricky_route())]


def build(route):
    nodes, aps = hp.nodes_from_route(route)
    return nodes, aps, hp.build_spline_manager(nodes, aps)


def segments(manager):
    return np.concatenate([np.asarray(s.segments, float) for s in manager.splines])


@pytest.mark.parametrize("name, route", routes(), ids=lambda v: v if isinstance(v, str) else "")
def test_materialized_settings_rebuild_the_same_spline(name, route):
    nodes, aps, manager = build(route)
    shapes = po.materialize(manager)
    frozen = po.frozen_nodes(nodes, aps, shapes)
    rebuilt = hp.build_spline_manager(po.apply_shapes(nodes, shapes, frozen), aps)
    assert np.abs(segments(rebuilt) - segments(manager)).max() < 1e-9


def test_zero_handle_mid_route_is_frozen_not_rewritten():
    nodes, aps, manager = build(tricky_route())
    frozen = po.frozen_nodes(nodes, aps, po.materialize(manager))
    assert 3 in frozen
    assert {0, 1, 2, 5}.isdisjoint(frozen)


@pytest.mark.parametrize("name, route", routes(), ids=lambda v: v if isinstance(v, str) else "")
def test_stretches_add_up_to_the_route(name, route):
    nodes, aps, manager = build(route)
    shapes = po.materialize(manager)
    frozen = po.frozen_nodes(nodes, aps, shapes)
    sides = po.knot_sides(manager)
    parts = [po._evaluate_stretch(nodes, shapes, s, CONSTRAINTS, frozen, sides) for s in po.stretches_of(nodes, aps)]
    vel, _, d = mpg.forward_backward_pass(manager, CONSTRAINTS, po.DD, end_vel=0.01 if nodes[-1].stop else None)
    v = np.maximum(np.asarray(vel), 1e-3)
    whole = float(np.sum(2 * np.diff(d) / (v[1:] + v[:-1])))
    # The distance grid restarts at each stop, so the split is close but not exact.
    assert sum(p.time for p in parts) == pytest.approx(whole, rel=0.01)


def test_default_locks():
    free = po.free_settings
    assert not free(rs.NodeRecord(is_start=True))[po.HEADING]
    assert not free(rs.NodeRecord(is_end=True))[po.HEADING]
    assert not free(rs.NodeRecord(actions={"lift_state": 1.0}))[po.HEADING]
    assert all(free(rs.NodeRecord()).values())
    assert free(rs.NodeRecord(is_start=True))[po.HANDLES]
    # The route's own field wins over the defaults, both ways.
    assert free(rs.NodeRecord(actions={"intake": 1.0}, optimize={"heading": True}))[po.HEADING]
    assert not free(rs.NodeRecord(optimize={"curvature": False}))[po.CURVATURE]


def test_optimize_field_round_trips_and_is_omitted_when_empty():
    route = rs.Route(nodes=[rs.NodeRecord(optimize={"heading": False}), rs.NodeRecord()])
    text = rs.dumps_route(route, ACTIONS)
    assert text.count('"optimize"') == 1
    parsed = rs.parse_route(text, ACTIONS)
    assert parsed.nodes[0].optimize == {"heading": False}
    assert parsed.nodes[1].optimize == {}
    assert not parsed.warnings
    with pytest.raises(rs.RouteParseError):
        rs.parse_route(text.replace('{"heading":false}', "true"), ACTIONS)


@pytest.fixture(scope="module")
def optimized():
    route = hp.load_route_file(REPO / "suite" / "routes" / "chain_4seg.txt", ACTIONS)
    route.nodes[2].optimize = {po.HEADING: False}
    options = po.OptimizerOptions(max_sweeps=4)
    return route, po.optimize_route(route, ACTIONS, CONSTRAINTS, options)


def test_optimizer_never_slows_the_route(optimized):
    _, result = optimized
    assert result.after_time <= result.before_time + 1e-9
    # Something must actually change, or the tests below check nothing.
    assert result.changes
    assert any(c.heading_deg for c in result.changes), "no free heading moved; the lock test is vacuous"


def test_optimizer_returns_the_route_it_timed(optimized):
    _, result = optimized
    nodes, aps, _ = build(result.route)
    assert hp.profile_route(hp.build_spline_manager(nodes, aps), CONSTRAINTS).times[-1] == pytest.approx(
        result.after_time
    )


def test_positions_and_locked_headings_do_not_move(optimized):
    route, result = optimized
    before = po.materialize(build(route)[2])
    after = po.materialize(build(result.route)[2])
    for rec_before, rec_after in zip(route.nodes, result.route.nodes):
        assert (rec_before.x, rec_before.y) == (rec_after.x, rec_after.y)
        assert rec_before.actions == rec_after.actions
    for i, rec in enumerate(route.nodes):
        if not po.free_settings(rec)[po.HEADING]:
            (x0, y0), (x1, y1) = before[i].direction, after[i].direction
            assert abs(x0 * y1 - y0 * x1) < 1e-9 and x0 * x1 + y0 * y1 > 0, f"node {i + 1}"


def test_only_changed_nodes_and_their_neighbours_are_rewritten(optimized):
    route, result = optimized
    changed = {c.node for c in result.changes}
    allowed = changed | {j for i in changed for j in (i - 1, i + 1)}
    for i, (a, b) in enumerate(zip(route.nodes, result.route.nodes)):
        if i not in allowed:
            assert a == b, f"node {i + 1} rewritten without changing"


def test_path_stays_in_the_corridor(optimized):
    route, result = optimized
    ts = np.linspace(0, len(route.nodes) - 1, 400)
    original = build(route)[2].get_points_at_parameters(ts)
    new = build(result.route)[2].get_points_at_parameters(ts)
    dist = np.sqrt(((new[:, None] - original[None]) ** 2).sum(-1)) * 12
    # Both ways: near the old path, and still covering all of it.
    assert max(dist.min(1).max(), dist.min(0).max()) <= po.OptimizerOptions().corridor_in + 0.1


def test_heading_holds_at_action_points_with_resets():
    # The chicane is where the optimizer reshapes the most (it eases the S-bend's
    # crossover), so a reset placed on it is the hard case for the guard.
    route = hp.load_route_file(REPO / "suite" / "routes" / "chicane_R12.txt", ACTIONS)
    route.action_points = [
        rs.ActionPointRecord(t=0.6, actions={"front_reset": 1.0}),
        rs.ActionPointRecord(t=1.4, actions={"intake": 1.0}),
    ]
    options = po.OptimizerOptions(max_sweeps=6)
    result = po.optimize_route(route, ACTIONS, CONSTRAINTS, options)
    assert result.saved > 0
    moved = dict(result.action_point_headings)
    assert abs(moved[0]) <= options.action_point_heading_deg + 0.05


def test_settings_switched_off_are_not_touched():
    route = hp.load_route_file(REPO / "suite" / "routes" / "chain_4seg.txt", ACTIONS)
    options = po.OptimizerOptions(max_sweeps=2, settings=(po.HANDLES,))
    result = po.optimize_route(route, ACTIONS, CONSTRAINTS, options)
    for change in result.changes:
        assert change.heading_deg == 0 and change.curvature_delta == 0


def test_cli_writes_a_copy_and_leaves_the_original(tmp_path):
    src = REPO / "suite" / "routes" / "vee_reverse.txt"
    route_file = tmp_path / "vee.txt"
    route_file.write_text(src.read_text())
    tool = [sys.executable, str(REPO / "tools" / "optimize_route.py")]

    shown = subprocess.run(tool + [str(route_file), "--show-locks"], capture_output=True, text=True, check=True)
    # Curvature is locked at the start and at the reverse node (both set off from rest).
    assert "start handles\n" in shown.stdout
    # The reverse node may also become a spin-through turn (it carries no action).
    assert "2       heading,handles,spin-through\n" in shown.stdout
    moved = subprocess.run(tool + [str(route_file), "--show-locks", "--free", "2:position", "--position-radius", "4"],
                           capture_output=True, text=True, check=True)
    assert "heading +-90 deg, handles, position 4 in, spin-through" in moved.stdout

    run = subprocess.run(tool + [str(route_file), "--write", "--settings", "handles"],
                         capture_output=True, text=True, check=True)
    assert "->" in run.stdout
    assert route_file.read_text() == src.read_text()
    out = tmp_path / "vee_opt.txt"
    if "written" in run.stdout:
        rs.parse_route(out.read_text(), ACTIONS)
        again = subprocess.run(tool + [str(route_file), "--write", "--settings", "handles"],
                               capture_output=True, text=True, check=True)
        assert "NOT written" in again.stdout


# ---------------------------------------------------------------------------
# Per-node amounts and corridor (the planner's Optimize page edits these)
# ---------------------------------------------------------------------------


def test_optimize_amounts_round_trip_and_reject_nonsense():
    limits = {"heading": 10.0, "handles": 1.5, "curvature": False, "corridor": 1.25}
    route = rs.Route(nodes=[rs.NodeRecord(optimize=limits), rs.NodeRecord()])
    parsed = rs.parse_route(rs.dumps_route(route, ACTIONS), ACTIONS)
    assert parsed.nodes[0].optimize == limits
    text = rs.dumps_route(route, ACTIONS)
    for bad in ('"heading":-1', '"heading":"lots"'):
        with pytest.raises(rs.RouteParseError):
            rs.parse_route(text.replace('"heading":10', bad), ACTIONS)


def test_node_limits():
    options = po.OptimizerOptions(heading_range_deg=20, handle_scale=(0.5, 2.0), corridor_in=3)
    plain = po.node_limits(rs.NodeRecord(), options)
    assert (plain.heading_deg, plain.handle_scale, plain.curvature, plain.corridor_in) == (20, (0.5, 2.0), math.inf, 3)
    # Defaults still lock the heading at start, end and action nodes; true frees it to
    # the run's range, a number to that many degrees.
    assert po.node_limits(rs.NodeRecord(is_start=True), options).heading_deg == 0
    assert po.node_limits(rs.NodeRecord(is_start=True, optimize={"heading": True}), options).heading_deg == 20
    custom = po.node_limits(
        rs.NodeRecord(actions={"intake": 1.0}, optimize={"heading": 5, "handles": 1.25, "curvature": 0.5, "corridor": 1}),
        options,
    )
    assert (custom.heading_deg, custom.handle_scale, custom.curvature, custom.corridor_in) == (5, (0.8, 1.25), 0.5, 1)
    # Zero degrees / x1 / 0 1/ft are locks.
    locked = po.node_limits(rs.NodeRecord(optimize={"heading": 0, "handles": 1, "curvature": 0}), options)
    assert not any(locked.free().values())


@pytest.fixture(scope="module")
def limited():
    """chain_4seg with tight amounts on two nodes and a narrow corridor on one."""
    route = hp.load_route_file(REPO / "suite" / "routes" / "chain_4seg.txt", ACTIONS)
    route.nodes[1].optimize = {po.HEADING: 3.0, po.HANDLES: 1.2, po.CURVATURE: 0.3}
    route.nodes[2].optimize = {po.CORRIDOR: 0.5}
    return route, po.optimize_route(route, ACTIONS, CONSTRAINTS, po.OptimizerOptions(max_sweeps=4))


def test_per_node_amounts_are_respected(limited):
    route, result = limited
    assert result.changes
    by_node = {c.node: c for c in result.changes}
    assert 1 in by_node, "the limited node did not move; the limits are untested"
    c = by_node[1]
    assert abs(c.heading_deg) <= 3.0 + 1e-6
    assert 1 / 1.2 - 1e-9 <= c.in_scale <= 1.2 + 1e-9
    assert 1 / 1.2 - 1e-9 <= c.out_scale <= 1.2 + 1e-9
    assert abs(c.curvature_delta) <= 0.3 + 1e-9


def test_per_node_corridor_holds_near_its_node(limited):
    route, result = limited
    # Around node 3 (index 2) the corridor narrows to 0.5 in; half a segment either
    # side it is still under the interpolated bound, which is at most 1.75 in.
    ts = np.linspace(1.5, 2.5, 200)
    original = build(route)[2].get_points_at_parameters(np.linspace(0, len(route.nodes) - 1, 800))
    new = build(result.route)[2].get_points_at_parameters(ts)
    near = np.sqrt(((new[:, None] - original[None]) ** 2).sum(-1)).min(1) * 12
    bound = np.interp(np.abs(ts - 2), [0, 1], [0.5, 3.0])
    assert (near <= bound + 0.1).all()


def test_should_stop_returns_the_route_unchanged_and_valid():
    route = hp.load_route_file(REPO / "suite" / "routes" / "chain_4seg.txt", ACTIONS)
    result = po.optimize_route(route, ACTIONS, CONSTRAINTS, should_stop=lambda: True)
    assert result.stopped_early and not result.changes
    assert result.after_time == pytest.approx(result.before_time)


def test_worker_process_runs_the_optimizer():
    from utilities import optimizer_worker as ow

    text = (REPO / "suite" / "routes" / "vee_reverse.txt").read_text()
    job = ow.OptimizerJob(text, ACTIONS, CONSTRAINTS, po.OptimizerOptions(max_sweeps=1))
    job.start()
    messages = []
    deadline = time.monotonic() + 120
    while not job.finished and time.monotonic() < deadline:
        messages += job.poll()
        time.sleep(0.05)
    job.kill()
    kinds = [kind for kind, _ in messages]
    assert kinds[-1] == ow.DONE, messages[-1]
    result = messages[-1][1]
    assert result.after_time <= result.before_time + 1e-9


# ---------------------------------------------------------------------------
# Default curvature locks, node positions and keep-out zones
# ---------------------------------------------------------------------------


def test_curvature_is_locked_where_the_robot_sets_off_from_rest():
    free = lambda **kw: po.free_settings(rs.NodeRecord(**kw))[po.CURVATURE]  # noqa: E731
    for kw in ({"is_start": True}, {"is_end": True}, {"is_reverse": True}, {"turn": 90},
               {"stop_mode": 1}, {"actions": {"wait_lift": 1.0}}):
        assert not free(**kw), kw
    assert free()
    assert free(is_reverse=True, stop_mode=2)  # drives through
    assert free(actions={"intake": 1.0})
    assert free(is_reverse=True, optimize={"curvature": True})


def test_position_limits():
    options = po.OptimizerOptions(position_radius_in=5, heading_range_deg=30, position_heading_deg=90)
    assert po.node_limits(rs.NodeRecord(), options).position_in == 0
    assert po.node_limits(rs.NodeRecord(optimize={"position": True}), options).position_in == 5
    moving = po.node_limits(rs.NodeRecord(optimize={"position": 2.5}), options)
    assert (moving.position_in, moving.heading_deg) == (2.5, 90)
    # A heading the route sets itself, or one locked by default, stays as it is.
    assert po.node_limits(rs.NodeRecord(optimize={"position": 3, "heading": 10}), options).heading_deg == 10
    assert po.node_limits(rs.NodeRecord(actions={"intake": 1.0}, optimize={"position": 3}), options).heading_deg == 0
    assert po.node_limits(rs.NodeRecord(optimize={"position": False}), options).position_in == 0


def test_keep_out_round_trips_and_rejects_nonsense():
    route = rs.Route(
        nodes=[rs.NodeRecord(is_start=True), rs.NodeRecord(x=10, is_end=True)],
        keep_out=[rs.KeepOutRecord("circle", [1, 2, 3]), rs.KeepOutRecord("rect", [4, 5, 6, 7])],
    )
    text = rs.dumps_route(route, ACTIONS)
    parsed = rs.parse_route(text, ACTIONS)
    assert parsed.keep_out == route.keep_out
    assert rs.parse_route(rs.dumps_route(route, ACTIONS, compact=True), ACTIONS).keep_out == route.keep_out
    # Corners in any order are stored min-first.
    swapped = rs.parse_route(text.replace("[4,5,6,7]", "[6,7,4,5]"), ACTIONS)
    assert swapped.keep_out[1].values == [4, 5, 6, 7]
    # Absent when there are none.
    assert "keep_out" not in rs.dumps_route(rs.Route(nodes=route.nodes), ACTIONS)
    for bad in ('{"circle":[1,2,0]}', '{"circle":[1,2]}', '{"blob":[1,2,3]}', '{"rect":[4,5,4,7]}', '[1,2,3]'):
        with pytest.raises(rs.RouteParseError):
            rs.parse_route(text.replace('{"circle":[1,2,3]}', bad), ACTIONS)


def test_keep_out_depth():
    footprint = (16.0, 12.0)  # 8 in either way along the heading, 6 across
    east = np.array([[1.0, 0.0]])
    circle = [rs.KeepOutRecord("circle", [20.0, 0.0, 5.0])]
    # Nose at x = 8 in, circle edge at 15: 7 in clear.
    assert po.keep_out_depth(circle, [[0.0, 0.0]], east, footprint)[0] == 0
    # Nose at 18: 3 in in. Turned north its side reaches 16: 1 in in.
    assert po.keep_out_depth(circle, [[10.0, 0.0]], east, footprint)[0] == pytest.approx(3.0)
    assert po.keep_out_depth(circle, [[10.0, 0.0]], [[0.0, 1.0]], footprint)[0] == pytest.approx(1.0)
    rect = [rs.KeepOutRecord("rect", [10.0, -2.0, 30.0, 2.0])]
    assert po.keep_out_depth(rect, [[0.0, 0.0]], east, footprint)[0] == 0
    assert po.keep_out_depth(rect, [[5.0, 0.0]], east, footprint)[0] == pytest.approx(3.0)
    # Turned 45 deg the corner reaches 4.9 in past x = 10, but the shortest way out is
    # along the robot's side: 6 + 12 sin 45 - 15 sin 45 = 3.88 in.
    diag = np.array([[math.sqrt(0.5), math.sqrt(0.5)]])
    assert po.keep_out_depth(rect, [[5.0, 0.0]], diag, footprint)[0] == pytest.approx(3.88, abs=0.01)
    # The deepest sample counts.
    both = po.keep_out_depth(rect, [[0.0, 0.0], [5.0, 0.0]], np.vstack([east, east]), footprint)
    assert both[0] == pytest.approx(3.0)


def cycle_route() -> rs.Route:
    """straight_cycle (2026-09-26): out to a matchload corner and back, turning round
    with a reverse node each way. The intake (node 3) must keep its heading."""
    return rs.Route(nodes=[
        rs.NodeRecord(x=-30.524, y=47.076, is_start=True, tangent=[-1, 0], tangent_derivative=[0, 0.03125],
                      incoming_magnitude=2.78323, outgoing_magnitude=1.390776),
        rs.NodeRecord(x=-48.257, y=40.805, is_reverse=True, tangent=[-0.35869, -0.933457],
                      tangent_derivative=[0, 0], incoming_magnitude=1.567431, outgoing_magnitude=1.567431,
                      optimize={"position": 6.0}),
        rs.NodeRecord(x=-64.688, y=57.534, is_reverse=True, tangent=[-0.978148, 0.207911],
                      tangent_derivative=[0, 0], incoming_magnitude=3.57845, outgoing_magnitude=3.581986,
                      optimize={"heading": False}),
        rs.NodeRecord(x=-48.308, y=40.754, is_reverse=True, tangent=[-0.358635, -0.933478],
                      tangent_derivative=[0, 0], incoming_magnitude=1.572147, outgoing_magnitude=1.572147,
                      optimize={"position": 6.0}),
        rs.NodeRecord(x=-30.541826, y=47.100658, is_end=True, tangent=[1, 0], tangent_derivative=[0, 0.25],
                      incoming_magnitude=1.677597, outgoing_magnitude=2.061868),
    ])


# Shape only: these tests are about how the reverse nodes move and what holds them back.
# Letting the run make them spin-through turns (test_spin_through_* below) changes where
# they go entirely.
CYCLE_OPTIONS = po.OptimizerOptions(
    max_sweeps=20, seeds=1, settings=po.SHAPE_SETTINGS + (po.POSITION,)
)


@pytest.fixture(scope="module")
def cycle():
    route = cycle_route()
    return route, po.optimize_route(route, ACTIONS, CONSTRAINTS, CYCLE_OPTIONS)


def test_moving_the_reverse_nodes_saves_time(cycle):
    route, result = cycle
    # By hand, moving the two reverse nodes ~3.5 in took it from 6.25 to 5.59 s.
    assert result.before_time == pytest.approx(6.25, abs=0.02)
    assert result.after_time < result.before_time - 0.4
    moved = {c.node for c in result.changes if math.hypot(c.dx_in, c.dy_in) > 0.5}
    assert moved == {1, 3}


def test_nodes_move_only_within_their_radius(cycle):
    route, result = cycle
    for i, (a, b) in enumerate(zip(route.nodes, result.route.nodes)):
        limit = a.optimize.get("position", 0.0)
        assert math.hypot(b.x - a.x, b.y - a.y) <= limit + 1e-6, f"node {i + 1}"
    # Start, intake and end keep their headings (the start/end by default, the intake by
    # its own lock) and the reverse nodes their curvature (locked by default).
    before = po.materialize(build(route)[2])
    after = po.materialize(build(result.route)[2])
    for i in (0, 2, 4):
        assert before[i].direction == pytest.approx(after[i].direction, abs=1e-9), f"node {i + 1}"
    for i in (0, 1, 2, 3, 4):
        assert before[i].curvature == pytest.approx(after[i].curvature, abs=1e-9), f"node {i + 1}"


def test_optimizer_returns_the_moved_route_it_timed(cycle):
    _, result = cycle
    nodes, aps, manager = build(result.route)
    assert hp.profile_route(manager, CONSTRAINTS).times[-1] == pytest.approx(result.after_time)


def footprint_depth(route, zones):
    nodes, aps, manager = build(route)
    ts = np.linspace(0, len(nodes) - 1, 2000)
    d = manager.get_derivatives_at_parameters(ts)
    return po.keep_out_depth(zones, manager.get_points_at_parameters(ts) * 12,
                             d / np.linalg.norm(d, axis=1)[:, None], CYCLE_OPTIONS.footprint_in)


def test_keep_out_zone_holds_the_moving_nodes_back(cycle):
    _, free_result = cycle
    route = cycle_route()
    # Unhindered, both reverse nodes go about 3.5 in towards -y; a wall of zone just
    # below where the robot already reaches stops that.
    route.keep_out = [rs.KeepOutRecord("rect", [-70, 20, -30, 33.5])]
    result = po.optimize_route(route, ACTIONS, CONSTRAINTS, CYCLE_OPTIONS)
    before, after = footprint_depth(route, route.keep_out), footprint_depth(result.route, route.keep_out)
    assert after[0] <= before[0] + 0.05
    lowest = lambda r: min(r.nodes[1].y, r.nodes[3].y)  # noqa: E731
    assert lowest(result.route) > lowest(free_result.route) + 1.0


# --- spin-through placement ---------------------------------------------------------

SPIN_OPTIONS = po.OptimizerOptions(max_sweeps=8, seeds=1, spin_through_sweeps=6)


@pytest.fixture(scope="module")
def spun():
    route = cycle_route()
    return route, po.optimize_route(route, ACTIONS, CONSTRAINTS, SPIN_OPTIONS)


def test_spin_through_turns_the_cycle_round_faster(cycle, spun):
    _, shape_only = cycle
    route, result = spun
    # The two turn-around reverse nodes become spin-through turns; the matchloader
    # (node 3, heading locked) never does.
    assert [n.spin_through for n in result.route.nodes] == [False, True, False, True, False]
    assert {c.node for c in result.changes if c.spin_through} == {1, 3}
    assert result.after_time < shape_only.after_time - 0.5
    assert result.warnings_after == []
    for i in (0, 2, 4):
        n0, n1 = route.nodes[i], result.route.nodes[i]
        assert (n0.x, n0.y, n0.turn, n0.spin_through) == (n1.x, n1.y, n1.turn, n1.spin_through)


def test_spin_through_route_is_the_one_it_timed(spun):
    _, result = spun
    nodes, aps, manager = build(result.route)
    assert hp.profile_route(manager, CONSTRAINTS).times[-1] == pytest.approx(result.after_time)
    reparsed = rs.parse_route(rs.dumps_route(result.route, ACTIONS), ACTIONS)
    assert [n.spin_through for n in reparsed.nodes] == [n.spin_through for n in result.route.nodes]


def test_a_node_can_refuse_spin_through():
    route = cycle_route()
    route.nodes[1].optimize[po.SPIN_THROUGH] = False
    quick = po.OptimizerOptions(max_sweeps=1, seeds=1, spin_through_sweeps=1)
    result = po.optimize_route(route, ACTIONS, CONSTRAINTS, quick)
    assert not result.route.nodes[1].spin_through
    assert result.route.nodes[1].turn == 0


def test_spin_through_candidates():
    route = cycle_route()
    assert [po.spin_candidate(n) for n in route.nodes] == [False, True, True, True, False]
    # The matchloader's heading is locked, so the run may not spin through it.
    assert [po.node_limits(n).spin_through for n in route.nodes] == [False, True, False, True, False]
    route.nodes[1].actions = {"intake": 1.0}
    route.nodes[3].wait_time = 0.5
    assert not po.spin_candidate(route.nodes[1]) and not po.spin_candidate(route.nodes[3])

