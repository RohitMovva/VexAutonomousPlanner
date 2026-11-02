"""Tests for the action slot spec (utilities/action_spec.py).

Pure Python - no Qt needed. Run from the repo root:
    pytest tests/test_action_spec.py -v

The codes checked here are the firmware's: LIFT_ACTION_* in
OverrideAutonomous/src/hardware/robot.cpp. See docs/action-slots.md.
"""

import pytest

from utilities import action_spec as spec
from utilities import route_serializer as rs

ACTIONS = [
    "intake",
    "lift_state",
    "right_reset",
    "left_reset",
    "front_reset",
    "lift_height",
    "intake_lift",
    "roller",
]


# ------------------------------------------------------------------ labels

@pytest.mark.parametrize(
    "name, value, expected",
    [
        ("lift_state", 0.0, "None"),
        ("lift_state", -1.0, "Home (matchload)"),
        ("lift_state", 1.0, "Score"),
        ("lift_state", 2.0, "Matchload"),
        ("lift_state", 3.0, "Close claw"),
        ("lift_state", 4.0, "Matchload (vertical stack)"),
        ("lift_height", 850.0, "850 deg"),
        ("intake", 2.0, "Outtake"),
        ("front_reset", 1.0, "Reset"),
        ("intake_lift", 0.0, "None"),
        ("intake_lift", 1.0, "Raise (auto-drops)"),
        ("intake_lift", -1.0, "Lower"),
        ("roller", 0.0, "None"),
        ("roller", 1.0, "Enable"),
        ("roller", -1.0, "Disable"),
        ("chainbar", 0.0, "None"),
        ("chainbar", 1.0, "Pickup: stack"),
        ("chainbar", 2.0, "Pickup: pin"),
        ("chainbar", 3.0, "Prime: alliance empty"),
        ("chainbar", 4.0, "Prime: alliance pin"),
        ("chainbar", 5.0, "Prime: neutral pin"),
        ("chainbar", -1.0, "Dormant"),
        ("sticks", 0.0, "None"),
        ("sticks", 1.0, "Up"),
        ("sticks", -1.0, "Down"),
    ],
)
def test_known_values_read_as_names(name, value, expected):
    assert spec.format_value(name, value) == expected


def test_unrecognised_choice_is_flagged_not_hidden():
    assert spec.format_value("lift_state", 7.0) == "7 (unrecognised)"


def test_unknown_action_falls_back_to_a_number():
    assert spec.spec_for("little_will") is spec.DEFAULT_SPEC
    assert spec.format_value("little_will", 1.5) == "1.5"


def test_choice_values_round_trip_through_labels():
    lift = spec.spec_for("lift_state")
    for label, value in lift.choices:
        assert lift.value_for(label) == value


# ------------------------------------------------------------------ per point

def test_height_and_macro_in_one_point_conflict():
    warnings = spec.check_point({"lift_state": spec.LIFT_MATCHLOAD, "lift_height": 900.0})
    assert any("split them across two points" in w for w in warnings)


def test_score_with_a_height_is_score_at_height():
    # Lift::score_at: release at that winch height, tower sensor ignored
    assert spec.check_point({"lift_state": spec.LIFT_SCORE, "lift_height": 300.0}) == []


def test_score_at_height_still_range_checks_the_height():
    warnings = spec.check_point({"lift_state": spec.LIFT_SCORE,
                                 "lift_height": spec.MAX_LIFT_HEIGHT_DEG + 100})
    assert any("outside" in w for w in warnings)


def test_height_above_the_firmware_cap_is_caught():
    warnings = spec.check_point({"lift_height": spec.MAX_LIFT_HEIGHT_DEG + 100})
    assert any("outside" in w for w in warnings)


def test_height_at_home_should_be_a_home_command():
    warnings = spec.check_point({"lift_height": 2.0})
    assert any("already home" in w for w in warnings)


def test_lift_and_intake_together_warn_about_the_pto():
    warnings = spec.check_point({"lift_height": 800.0, "intake": 1.0})
    assert any("PTO" in w for w in warnings)


def test_unknown_intake_lift_code_is_caught():
    warnings = spec.check_point({"intake_lift": 2.0})
    assert any("intake_lift 2 is not a code" in w for w in warnings)


@pytest.mark.parametrize("code", [spec.INTAKE_LIFT_UP, spec.INTAKE_LIFT_DOWN])
def test_intake_lift_codes_are_clean(code):
    assert spec.check_point({"intake_lift": code, "intake": 1.0}) == []


def test_unknown_roller_code_is_caught():
    warnings = spec.check_point({"roller": 3.0})
    assert any("roller 3 is not a code" in w for w in warnings)


def test_roller_force_warns_the_lift_falls():
    warnings = spec.check_point({"roller": spec.ROLLER_FORCE})
    assert not any("is not a code" in w for w in warnings)
    assert any("a raised lift falls" in w for w in warnings)


@pytest.mark.parametrize("lift", [{"lift_state": 1.0}, {"lift_height": 120.0}])
def test_roller_force_with_a_lift_command_warns_it_is_undone(lift):
    warnings = spec.check_point({"roller": spec.ROLLER_FORCE, **lift})
    assert any("takes the PTO straight back" in w for w in warnings)


def test_roller_with_intake_running_warns_it_waits():
    warnings = spec.check_point({"roller": spec.ROLLER_ENABLE, "intake": 1.0})
    assert any("waits until the intake stops" in w for w in warnings)


def test_unknown_chainbar_code_is_caught():
    warnings = spec.check_point({"chainbar": 6.0})
    assert any("chainbar 6 is not a code" in w for w in warnings)


@pytest.mark.parametrize("code", [c for _, c in spec.SPECS["chainbar"].choices if c])
def test_chainbar_codes_are_clean(code):
    assert spec.check_point({"chainbar": code}) == []


def test_disabling_the_roller_alongside_the_intake_is_clean():
    assert spec.check_point({"roller": spec.ROLLER_DISABLE, "intake": 1.0}) == []


def test_a_plain_height_is_clean():
    assert spec.check_point({"lift_height": 800.0}) == []


def test_unknown_lift_code_is_caught():
    warnings = spec.check_point({"lift_state": 7.0})
    assert any("not a code the firmware knows" in w for w in warnings)


# ------------------------------------------------------------------ sequence

def test_score_without_a_prime_is_caught():
    warnings = spec.lint([("node 1", {"lift_state": spec.LIFT_SCORE})])
    assert any("score is ignored" in w for w in warnings)


def test_height_primes_the_lift_for_a_later_score():
    warnings = spec.lint(
        [
            ("node 1", {"lift_height": 900.0}),
            ("node 2", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert warnings == []


def test_matchload_primes_the_lift():
    warnings = spec.lint(
        [
            ("action point 1", {"lift_state": spec.LIFT_MATCHLOAD}),
            ("action point 2", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert warnings == []


def test_stack_matchload_primes_the_lift():
    warnings = spec.lint(
        [
            ("action point 1", {"lift_state": spec.LIFT_MATCHLOAD_STACK}),
            ("action point 2", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert warnings == []


def test_stack_matchload_is_a_known_code():
    assert spec.check_point({"lift_state": spec.LIFT_MATCHLOAD_STACK}) == []


def test_fast_matchload_primes_the_lift():
    warnings = spec.lint(
        [
            ("action point 1", {"lift_state": spec.LIFT_MATCHLOAD_FAST}),
            ("action point 2", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert warnings == []


def test_fast_matchload_is_a_known_code():
    assert spec.check_point({"lift_state": spec.LIFT_MATCHLOAD_FAST}) == []
    lift = spec.SPECS["lift_state"]
    assert lift.value_for("Matchload (lift while closing)") == spec.LIFT_MATCHLOAD_FAST
    assert lift.label_for(spec.LIFT_MATCHLOAD_FAST) == "Matchload (lift while closing)"


def test_closed_matchload_primes_the_lift():
    warnings = spec.lint(
        [
            ("action point 1", {"lift_state": spec.LIFT_MATCHLOAD_CLOSED}),
            ("action point 2", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert warnings == []


def test_closed_matchload_is_a_known_code():
    assert spec.check_point({"lift_state": spec.LIFT_MATCHLOAD_CLOSED}) == []
    lift = spec.SPECS["lift_state"]
    assert lift.value_for("Matchload (claw closed)") == spec.LIFT_MATCHLOAD_CLOSED
    assert lift.label_for(spec.LIFT_MATCHLOAD_CLOSED) == "Matchload (claw closed)"


def test_closed_matchload_with_height_is_a_conflict():
    assert spec.check_point({"lift_state": spec.LIFT_MATCHLOAD_CLOSED, "lift_height": 900.0})


def test_scoring_twice_needs_a_second_prime():
    warnings = spec.lint(
        [
            ("node 1", {"lift_height": 900.0}),
            ("node 2", {"lift_state": spec.LIFT_SCORE}),
            ("node 3", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert len(warnings) == 1
    assert warnings[0].startswith("node 3")


def test_close_claw_at_the_start_is_clean_and_a_raise_still_primes():
    warnings = spec.lint(
        [
            ("node 1", {"lift_state": spec.LIFT_GRAB}),
            ("action point 1", {"lift_height": 200.0}),
            ("node 2", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert warnings == []


def test_close_claw_does_not_prime_the_lift():
    warnings = spec.lint(
        [
            ("node 1", {"lift_state": spec.LIFT_GRAB}),
            ("node 2", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert any("node 2: score is ignored" in w for w in warnings)


def test_close_claw_at_boot_is_honoured_in_matchload():
    # The lift boots in MATCHLOAD, and a grab there holds the preload (Lift::grab).
    warnings = spec.lint([("node 1", {"lift_state": spec.LIFT_GRAB})])
    assert not any("close claw is ignored" in w for w in warnings)


def test_close_claw_after_going_home_is_honoured():
    # Home is the matchload pose now (no LOADING mode), where a grab applies
    warnings = spec.lint(
        [
            ("node 1", {"lift_height": 400.0}),
            ("node 2", {"lift_state": spec.LIFT_HOME}),
            ("node 3", {"lift_state": spec.LIFT_GRAB}),
        ]
    )
    assert not any("close claw is ignored" in w for w in warnings)


def test_close_claw_while_primed_is_ignored():
    warnings = spec.lint(
        [
            ("node 1", {"lift_height": 400.0}),
            ("node 2", {"lift_state": spec.LIFT_GRAB}),
        ]
    )
    assert any("node 2: close claw is ignored while the lift is PRIMED" in w for w in warnings)


def test_a_matchload_cycle_comes_home_to_the_loader():
    # Every score returns to MATCHLOAD, the lift's only rest pose, so the next
    # cycle is a matchload again.
    warnings = spec.lint(
        [
            ("node 1", {"lift_state": spec.LIFT_MATCHLOAD}),
            ("node 2", {"lift_state": spec.LIFT_SCORE}),
            ("node 3", {"lift_state": spec.LIFT_MATCHLOAD}),
            ("node 4", {"lift_state": spec.LIFT_SCORE}),
        ]
    )
    assert warnings == []


# ------------------------------------------------------------------ route file

def test_height_survives_a_route_round_trip():
    route = rs.Route(
        nodes=[
            rs.NodeRecord(x=1.0, y=2.0, is_start=True, actions={"lift_height": 900.0}),
            rs.NodeRecord(x=3.0, y=4.0, is_end=True, actions={"lift_state": spec.LIFT_SCORE}),
        ],
        action_points=[],
    )
    text = rs.dumps_route(route, ACTIONS)
    parsed = rs.parse_route(text, ACTIONS)
    assert parsed.warnings == []
    assert parsed.nodes[0].actions == {"lift_height": 900.0}
    assert parsed.nodes[1].actions == {"lift_state": spec.LIFT_SCORE}


def test_intake_lower_survives_a_route_round_trip():
    # -1 is a command, not "unset": the serializer drops only zeros
    route = rs.Route(
        nodes=[
            rs.NodeRecord(x=1.0, y=2.0, is_start=True, actions={"intake_lift": spec.INTAKE_LIFT_UP}),
            rs.NodeRecord(x=3.0, y=4.0, is_end=True, actions={"intake_lift": spec.INTAKE_LIFT_DOWN}),
        ],
        action_points=[],
    )
    parsed = rs.parse_route(rs.dumps_route(route, ACTIONS), ACTIONS)
    assert parsed.warnings == []
    assert parsed.nodes[0].actions == {"intake_lift": spec.INTAKE_LIFT_UP}
    assert parsed.nodes[1].actions == {"intake_lift": spec.INTAKE_LIFT_DOWN}


def test_unknown_sticks_code_is_caught():
    warnings = spec.check_point({"sticks": 2.0})
    assert any("sticks 2 is not a code" in w for w in warnings)


@pytest.mark.parametrize("code", [c for _, c in spec.SPECS["sticks"].choices if c])
def test_sticks_codes_are_clean(code):
    assert spec.check_point({"sticks": code}) == []
