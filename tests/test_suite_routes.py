"""The path-following suite (suite/routes, tools/make_suite.py) must stay loadable,
land in the bands its manifest promises, and export in the firmware's format."""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from utilities import headless_profile as hp

ROOT = Path(__file__).resolve().parents[1]
SUITE = ROOT / "suite"
CONFIG = yaml.safe_load((ROOT / "config.yaml").read_text())
MANIFEST = yaml.safe_load((SUITE / "manifest.yaml").read_text())["routes"]


def profiled(name):
    route = hp.load_route_file(SUITE / "routes" / f"{name}.txt", CONFIG["robot"]["actions"])
    nodes, aps = hp.nodes_from_route(route)
    manager = hp.build_spline_manager(nodes, aps)
    constraints = hp.constraints_from_config(CONFIG["motion"], CONFIG["robot"]["track_width"])
    return nodes, aps, manager, hp.profile_route(manager, constraints)


def test_manifest_matches_route_files():
    files = {p.stem for p in (SUITE / "routes").glob("*.txt")}
    assert files == set(MANIFEST), "regenerate with tools/make_suite.py"


@pytest.mark.parametrize("name", sorted(MANIFEST))
def test_route_profiles_and_holds_its_band(name):
    spec = MANIFEST[name]
    nodes, aps, manager, result = profiled(name)
    report = result.report
    assert report.duration > 0.5
    # spin_max is planned past the robot's limits on purpose (exceeds_caps in the manifest)
    if not spec.get("exceeds_caps"):
        assert not any("exceeds" in w for w in report.warnings), report.warnings

    if "radius_ft" in spec:
        target = spec["radius_ft"]
        manager.rebuild_tables()
        dists = np.linspace(0, manager.get_total_arc_length(), 800)
        kappa = np.abs(manager.get_curvatures(manager.distances_to_times(dists)))
        radius = np.where(kappa > 1e-6, 1.0 / np.maximum(kappa, 1e-9), np.inf)
        # Blends approach the arc from a larger radius, never a smaller one.
        assert radius.min() >= 0.95 * target, f"{name}: min radius {radius.min():.2f} ft below {target}"
        # Routes built from constant-radius arcs must actually SIT on that radius, not
        # just graze it. A route marked holds_radius: false is a spline whose curvature
        # varies continuously (the chicane is an S-bend through a single inflection) --
        # radius_ft is its tightest point, which the bound above and min_turn_radius
        # below both still check, but there is no arc for it to hold.
        if spec.get("holds_radius", True):
            turning = radius < 1.5 * target
            on_arc = turning & (np.abs(radius / target - 1.0) < 0.03)
            share = on_arc.sum() / max(turning.sum(), 1)
            assert share > 0.5, f"{name}: only {100 * share:.0f}% of the turning path sits on the {target} ft arc"
        # The profile actually turns at that radius (not just the geometry).
        assert report.min_turn_radius == pytest.approx(target, rel=0.05)
    else:
        if spec.get("straight_in") and "segments" not in spec and name not in ("vee_reverse", "wall_reset"):
            assert math.isinf(report.min_turn_radius), f"{name} should be straight"

    if name == "crawl_R3":
        assert result.report.peak_speed * 12 == pytest.approx(spec["max_velocity_in_s"], rel=0.05)
    if name in ("vee_reverse", "wall_reset", "chain_4seg"):
        expected = spec.get("segments", 2) - 1
        assert sum(1 for n in nodes if n.is_reverse_node) == expected
    if name == "reverse_arc_R12":
        assert min(result.linear_vels) < -0.5 and max(result.linear_vels) < 0.05
    if name == "lift_while_driving":
        assert len(aps) == 1 and aps[0].actions == {"intake": 1, "lift_state": 1}


@pytest.mark.parametrize("name", ["corner_R6", "lift_while_driving", "wall_reset"])
def test_export_matches_firmware_format(name):
    nodes, aps, manager, result = profiled(name)
    names = CONFIG["robot"]["actions"]
    text = hp.export_trajectory(result, nodes, aps, names)
    rows = [line.split() for line in text.splitlines()]
    samples = [r for r in rows if r[0] == "0"]
    actions = [r for r in rows if r[0] == "1"]
    assert len(samples) == len(result.times)
    assert all(len(r) == 7 for r in samples)
    assert all(len(r) == 1 + len(names) for r in actions)
    assert len(actions) == len(nodes) + len(aps)
    # Canvas y is flipped for the robot and the first sample is the start pose.
    assert float(samples[0][2]) == pytest.approx(nodes[0].x * 12, abs=1e-6)
    assert float(samples[0][3]) == pytest.approx(-nodes[0].y * 12, abs=1e-6)
    if name == "wall_reset":
        reset_rows = [r for r in actions if float(r[1 + names.index("front_reset")]) == 1]
        assert len(reset_rows) == 1


# --------------------------------------------------------- odometry slip tests

import sys  # noqa: E402

sys.path.insert(0, str(ROOT / "tools"))
import make_suite  # noqa: E402

SLIP = sorted(n for n, s in MANIFEST.items() if s.get("slip_test"))


@pytest.mark.parametrize("name", SLIP)
def test_slip_route_holds_see_two_walls(name):
    """Every hold has wall truth on both axes, clear of the -x wall matchloader, and
    the route fires no distance reset (it measures raw odometry)."""
    nodes, aps, manager, result = profiled(name)
    holds = make_suite.holds_of(result)
    assert holds and holds[0][0] == 0.0 and holds[0][1] >= 0.5, "needs a still start"
    assert result.times[-1] - holds[-1][1] < 0.02 and holds[-1][1] - holds[-1][0] >= 0.7, "needs a still end"
    for t0, t1, pose in holds:
        views = make_suite.sensor_views(pose)
        assert {v[1] for v in views.values()} == {"x", "y"}, f"{name} hold at {t0:.2f}s sees {views}"
        for wall, axis, reading, hx, hy in views.values():
            if wall == "-x":
                assert hy >= -47.0, f"{name}: beam hits the -x wall at y={hy:.1f}, by the matchloader"
    for n in nodes:
        assert not any(n.actions.get(k) for k in ("front_reset", "left_reset", "right_reset"))


@pytest.mark.parametrize("name,condition,share", [
    ("slip_turn_steady", "steady", 0.9),
    ("slip_turn_brake", "braking", 0.6),
    ("slip_turn_accel", "accelerating", 0.55),
])
def test_slip_turn_rotates_under_its_condition(name, condition, share):
    """The point of each turn route: its 90 deg happens under the stated speed change."""
    _, _, _, result = profiled(name)
    shares = make_suite.rotation_by_accel(result)
    assert shares["moving_deg"] == pytest.approx(90.0, abs=1.0)
    assert shares[condition] >= share, shares
    other = {"braking": "accelerating", "accelerating": "braking"}.get(condition)
    if other:
        assert shares[other] <= 0.15, shares
    assert result.headings[-1] == pytest.approx(-math.pi / 2, abs=math.radians(1.0))


def test_slip_spin_alternates_in_place_turns():
    nodes, _, _, result = profiled("slip_spin")
    turns = [n.turn for n in nodes if n.turn]
    assert turns == [-90.0, 90.0, -90.0, 90.0, -90.0]
    shares = make_suite.rotation_by_accel(result)
    assert shares["in_place_deg"] == pytest.approx(450.0, abs=2.0)
    assert shares["moving_deg"] < 1.0
