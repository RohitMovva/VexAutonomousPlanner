"""Tests for the route file format (utilities/route_serializer.py).

Pure Python - no Qt or numpy needed. Run from the repo root:
    pytest tests/test_route_serializer.py -v

The corpus tests parametrize over every real route file in routes/, so a
parser regression against files the team actually uses shows up by name.
"""

import json
from pathlib import Path

import pytest

from utilities import route_serializer as rs

REPO_ROOT = Path(__file__).resolve().parents[1]
ROUTE_FILES = sorted((REPO_ROOT / "routes").glob("*.txt"))

# The action list config.yaml shipped with while the legacy corpus was
# written; tests must not depend on the live config.
ACTIONS = [
    "intake",
    "little_will",
    "left_reset",
    "right_reset",
    "intake_hold_decay",
    "drivetrain_slip_voltage",
    "rake",
    "front_reset",
    "color_sort_toggle",
]


def make_route():
    return rs.Route(
        nodes=[
            rs.NodeRecord(
                x=10.5, y=-20.25, is_start=True, tangent=[0.5, -0.5],
                actions={"intake": 1.0},
            ),
            rs.NodeRecord(
                x=0.0, y=0.0, is_reverse=True, stop_mode=1, turn=90,
                wait_time=1.5, max_velocity=60.0, locked=True, visible=False,
                tangent_derivative=[0.1, 0.2], incoming_magnitude=2.5,
                outgoing_magnitude=3.5, actions={"rake": 2.0, "old_gone": 7.0},
            ),
            rs.NodeRecord(x=-5.0, y=5.0, is_end=True),
        ],
        action_points=[
            rs.ActionPointRecord(
                x=1.0, y=2.0, t=1.25, stop=True, wait_time=0.5,
                actions={"intake": -1.0},
            ),
        ],
    )


def normalize(route):
    """Comparable snapshot of a route (warnings and version excluded)."""

    def num(v):
        return round(float(v), 6)

    def vec(v):
        return None if v is None else tuple(num(x) for x in v)

    def acts(a):
        return tuple(sorted((k, num(v)) for k, v in a.items() if v))

    return (
        tuple(
            (
                num(n.x), num(n.y), n.is_start, n.is_end, n.is_reverse,
                n.stop_mode, num(n.turn), num(n.wait_time),
                num(n.max_velocity), num(n.max_acceleration),
                vec(n.tangent), vec(n.tangent_derivative),
                None if n.incoming_magnitude is None else num(n.incoming_magnitude),
                None if n.outgoing_magnitude is None else num(n.outgoing_magnitude),
                n.locked, n.visible, acts(n.actions),
            )
            for n in route.nodes
        ),
        tuple(
            (
                num(a.x), num(a.y), num(a.t), a.stop, num(a.wait_time),
                num(a.max_velocity), num(a.max_acceleration), acts(a.actions),
            )
            for a in route.action_points
        ),
    )


class TestCurrentFormat:
    def test_round_trip(self):
        route = make_route()
        text = rs.dumps_route(route, ACTIONS)
        parsed = rs.parse_route(text, ACTIONS)
        assert normalize(parsed) == normalize(route)
        assert parsed.source_version == rs.FORMAT_VERSION

    def test_round_trip_compact(self):
        route = make_route()
        text = rs.dumps_route(route, ACTIONS, compact=True)
        assert "\n" not in text
        assert normalize(rs.parse_route(text, ACTIONS)) == normalize(route)

    def test_serialization_is_stable(self):
        """Undo snapshots compare serialized strings, so save->load->save
        must be byte-identical."""
        text = rs.dumps_route(make_route(), ACTIONS)
        again = rs.dumps_route(rs.parse_route(text, ACTIONS), ACTIONS)
        assert text == again

    def test_defaults_omitted(self):
        text = rs.dumps_route(
            rs.Route(nodes=[rs.NodeRecord(x=1.0, y=2.0)]), ACTIONS
        )
        (node,) = json.loads(text)["nodes"]
        assert node == {"x": 1, "y": 2}

    def test_unknown_action_warns_and_survives(self):
        route = make_route()  # node 1 has action "old_gone"
        text = rs.dumps_route(route, ACTIONS)
        parsed = rs.parse_route(text, ACTIONS)
        assert parsed.nodes[1].actions["old_gone"] == 7.0
        assert any("old_gone" in w for w in parsed.warnings)

    def test_config_action_added_and_removed(self):
        """The headline fix: changing robot.actions must not corrupt routes."""
        text = rs.dumps_route(make_route(), ACTIONS)

        grown = ACTIONS + ["brand_new_action"]
        parsed = rs.parse_route(text, grown)
        assert parsed.nodes[0].actions == {"intake": 1.0}

        shrunk = [a for a in ACTIONS if a != "rake"]
        parsed = rs.parse_route(text, shrunk)
        assert parsed.nodes[1].actions["rake"] == 2.0  # preserved, flagged
        assert any("rake" in w for w in parsed.warnings)
        # ...and still there after a save in the shrunk config
        resaved = rs.dumps_route(parsed, shrunk)
        assert rs.parse_route(resaved, ACTIONS).nodes[1].actions["rake"] == 2.0

    def test_newer_version_warns(self):
        obj = json.loads(rs.dumps_route(make_route(), ACTIONS))
        obj["version"] = rs.FORMAT_VERSION + 1
        obj["nodes"][0]["field_from_the_future"] = 42
        parsed = rs.parse_route(json.dumps(obj), ACTIONS)
        assert any("newer" in w for w in parsed.warnings)
        assert any("field_from_the_future" in w for w in parsed.warnings)
        assert len(parsed.nodes) == 3

    def test_garbage_raises(self):
        with pytest.raises(rs.RouteParseError):
            rs.parse_route("not json at all", ACTIONS)
        with pytest.raises(rs.RouteParseError):
            rs.parse_route('"just a string"', ACTIONS)
        with pytest.raises(rs.RouteParseError):
            rs.parse_route('{"nodes": [{"y": 1}]}', ACTIONS)  # x missing


class TestLegacyFormat:
    HEADER = [10.0, 20.0, 1, 0, 1, 2, 45, 1.5, 60.0, 30.0, [0.5, -0.5], [0.0, 0.0], 2.5, 3.5]

    def legacy_text(self, tail, action_points=None):
        return json.dumps([[self.HEADER + tail], action_points or []])

    def test_header_fields(self):
        parsed = rs.parse_route(self.legacy_text([0.0] * len(ACTIONS)), ACTIONS)
        (n,) = parsed.nodes
        assert (n.x, n.y) == (10.0, 20.0)
        assert n.is_start and not n.is_end and n.is_reverse
        assert n.stop_mode == 2 and n.turn == 45 and n.wait_time == 1.5
        assert n.max_velocity == 60.0 and n.max_acceleration == 30.0
        assert n.tangent == [0.5, -0.5]
        assert n.tangent_derivative is None  # legacy [0,0] means "not set"
        assert (n.incoming_magnitude, n.outgoing_magnitude) == (2.5, 3.5)
        assert parsed.source_version == rs.LEGACY_VERSION

    def test_actions_without_flags(self):
        tail = [1.0] + [0.0] * (len(ACTIONS) - 1)
        parsed = rs.parse_route(self.legacy_text(tail), ACTIONS)
        assert parsed.nodes[0].actions == {"intake": 1.0}
        assert parsed.nodes[0].locked is False
        assert parsed.nodes[0].visible is True

    def test_actions_with_locked_visible_flags(self):
        tail = [0.0, 3.0] + [0.0] * (len(ACTIONS) - 2) + [1, 0]
        parsed = rs.parse_route(self.legacy_text(tail), ACTIONS)
        assert parsed.nodes[0].actions == {"little_will": 3.0}
        assert parsed.nodes[0].locked is True
        assert parsed.nodes[0].visible is False

    def test_fewer_actions_than_config(self):
        """A file saved before actions were appended to config.yaml."""
        parsed = rs.parse_route(self.legacy_text([1.0, 2.0]), ACTIONS)
        assert parsed.nodes[0].actions == {"intake": 1.0, "little_will": 2.0}

    def test_action_points(self):
        text = self.legacy_text(
            [0.0] * len(ACTIONS),
            action_points=[[5.0, 6.0, 1.5, 1, 0.5, 40.0, 20.0, 0.0, 4.0]],
        )
        (a,) = rs.parse_route(text, ACTIONS).action_points
        assert (a.x, a.y, a.t) == (5.0, 6.0, 1.5)
        assert a.stop and a.wait_time == 0.5
        assert a.actions == {"little_will": 4.0}

    def test_bare_nodes_list(self):
        """Oldest shape: no action-point list at all, even with 2 nodes
        (which must not be mistaken for [nodes, action_points])."""
        record = self.HEADER + [0.0] * len(ACTIONS)
        parsed = rs.parse_route(json.dumps([record, record]), ACTIONS)
        assert len(parsed.nodes) == 2
        assert parsed.action_points == []

    def test_short_record_skipped(self):
        text = json.dumps([[self.HEADER, [1.0, 2.0]], []])
        parsed = rs.parse_route(text, ACTIONS)
        assert len(parsed.nodes) == 1
        assert any("too short" in w for w in parsed.warnings)

    def test_legacy_warns_about_migration(self):
        parsed = rs.parse_route(self.legacy_text([0.0] * len(ACTIONS)), ACTIONS)
        assert any("legacy" in w for w in parsed.warnings)


@pytest.mark.skipif(not ROUTE_FILES, reason="no route corpus checked out")
class TestRealCorpus:
    @pytest.mark.parametrize(
        "route_file", ROUTE_FILES, ids=lambda p: p.name
    )
    def test_migration_preserves_route(self, route_file):
        """Every real route file parses, and migrating it to the current
        format loses nothing."""
        text = route_file.read_text()
        if not text.strip():
            pytest.skip("empty placeholder file (created by set_working_file)")
        legacy = rs.parse_route(text, ACTIONS)
        migrated = rs.parse_route(rs.dumps_route(legacy, ACTIONS), ACTIONS)
        assert normalize(migrated) == normalize(legacy)
        # Migrated files are stable: a second round trip is byte-identical.
        once = rs.dumps_route(legacy, ACTIONS)
        twice = rs.dumps_route(rs.parse_route(once, ACTIONS), ACTIONS)
        assert once == twice
