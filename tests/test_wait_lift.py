"""The wait_lift action slot: planner half of the hold-until-the-lift-is-done flag.

The firmware half is Robot::followTrajectory holding on slot 8 (LIFT_WAIT_*) in
OverrideAutonomous/src/hardware/robot.cpp. See docs/action-slots.md.
"""

from pathlib import Path

import pytest
import yaml

from gui.gui_manager import format_path_time
from utilities import action_spec as spec
from utilities import headless_profile as hp
from utilities import route_serializer as rs

REPO = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((REPO / "config.yaml").read_text())
ACTIONS = CONFIG["robot"]["actions"]
ROUTE = REPO / "suite" / "routes" / "arc_R12.txt"
# The firmware only holds while the last sample is slower than this (in/s);
# LIFT_WAIT_MAX_SPEED in robot.cpp.
FIRMWARE_HOLD_MAX_SPEED = 3.0


def test_slot_position_in_config():
    # Appended after roller, so trajectories exported before it read the slot as 0 (no
    # wait). Slots added later go after it.
    assert ACTIONS.index(spec.WAIT_LIFT) == 8


def test_labels_and_codes():
    assert spec.format_value(spec.WAIT_LIFT, 0.0) == "Off"
    assert spec.format_value(spec.WAIT_LIFT, 1.0) == "Wait for lift"
    assert spec.check_point({spec.WAIT_LIFT: 1.0}) == []
    assert any("not a code" in w for w in spec.check_point({spec.WAIT_LIFT: 2.0}))


def test_wait_needs_a_stop():
    assert spec.check_stop({spec.WAIT_LIFT: 1.0}, stops=True) == []
    assert spec.check_stop({}, stops=False) == []
    assert spec.check_stop({spec.WAIT_LIFT: 1.0}, stops=False)


def test_node_with_wait_stops_unless_overridden():
    node = hp.HeadlessNode(0.0, 0.0, actions={spec.WAIT_LIFT: 1.0})
    assert node.stop
    node.stop_mode = hp.STOP_DISABLED
    assert not node.stop
    assert not hp.HeadlessNode(0.0, 0.0).stop


def _profile(wait_node=None):
    route = hp.load_route_file(ROUTE, ACTIONS)
    nodes, action_points = hp.nodes_from_route(route)
    if wait_node is not None:
        nodes[wait_node].actions[spec.WAIT_LIFT] = 1.0
    constraints = hp.constraints_from_config(CONFIG["motion"], CONFIG["robot"]["track_width"])
    profiled = hp.profile_route(hp.build_spline_manager(nodes, action_points), constraints)
    return profiled, nodes, action_points


def test_profile_stops_at_the_wait_and_exports_the_flag():
    base, nodes, _ = _profile()
    mid = len(nodes) // 2
    assert not nodes[mid].stop, "pick a node the route normally drives through"
    profiled, nodes, action_points = _profile(wait_node=mid)

    row = profiled.nodes_map[mid]
    # The firmware holds on the sample before the node's action row.
    assert abs(profiled.linear_vels[row - 1]) * 12 < FIRMWARE_HOLD_MAX_SPEED
    # Stopping costs time, but no fixed wait is added for the lift.
    assert profiled.times[-1] > base.times[-1]

    lines = hp.export_trajectory(profiled, nodes, action_points, ACTIONS).splitlines()
    action_rows = [line.split() for line in lines if line.startswith("1 ")]
    flags = [float(fields[1 + ACTIONS.index(spec.WAIT_LIFT)]) for fields in action_rows]
    assert flags.count(1.0) == 1


def test_route_file_round_trip():
    route = rs.Route(nodes=[rs.NodeRecord(x=0, y=0, actions={spec.WAIT_LIFT: 1.0})])
    parsed = rs.parse_route(rs.dumps_route(route, ACTIONS), ACTIONS)
    assert parsed.nodes[0].actions[spec.WAIT_LIFT] == 1.0


def test_path_time_lists_lift_waits():
    profiled, _, _ = _profile()
    args = (profiled.times, profiled.positions, profiled.linear_vels, profiled.report)
    text, _ = format_path_time(*args)
    assert "lift" not in text
    text, tooltip = format_path_time(*args, lift_waits=2)
    assert text.endswith(" + 2 lift waits")
    assert "Not counted: 2 waits for the lift" in tooltip
    assert format_path_time(*args, lift_waits=1)[0].endswith(" + 1 lift wait")
