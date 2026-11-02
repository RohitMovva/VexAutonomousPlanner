"""Editor metadata and sanity checks for the robot action slots.

``config.yaml`` ``robot.actions`` owns the slot *order*, which is the positional
order the firmware reads in ``Robot::processAction``. This module owns what a
known slot *means*: how it is edited, how a value reads in a menu, and which
combinations the firmware will refuse. A slot with no spec here falls back to a
plain number field, so one can be added to the config without touching the GUI.

The slot table itself is the cross-repo contract in ``docs/action-slots.md``.
Pure Python (no Qt), so tools and tests can import it standalone.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

CHOICE = "choice"
NUMBER = "number"

# lift_state codes, matching the LIFT_ACTION_* constants in robot.cpp
LIFT_NONE = 0.0
LIFT_HOME = -1.0
LIFT_SCORE = 1.0
LIFT_MATCHLOAD = 2.0
LIFT_GRAB = 3.0  # close the claw in the matchload pose without raising (Lift::grab)
# was matchload at the vertical-stack jaw width; the firmware runs it as a plain
# matchload now, kept so routes that carry it still run
LIFT_MATCHLOAD_STACK = 4.0
# matchload, but the take raises while the claw closes instead of holding
# Lift::CLAW_CLOSE_MS first; one take only (Lift::matchload(true))
LIFT_MATCHLOAD_FAST = 5.0
# matchload with the claw waiting closed: the take raises as soon as the loader sees
# the piece (Lift::set_matchload_claw_closed). Sticks for the re-arm after a score
# until a 2 / 4 / 5 opens it again
LIFT_MATCHLOAD_CLOSED = 6.0

# intake_lift codes, matching INTAKE_LIFT_* in robot.cpp. 0 is no command: every
# point writes every slot, so "down" cannot be 0 or it would cancel a raise early.
INTAKE_LIFT_NONE = 0.0
INTAKE_LIFT_UP = 1.0
INTAKE_LIFT_DOWN = -1.0

# roller codes, matching ROLLER_* in robot.cpp. The colour it turns to is the
# firmware's RollerMech target (set in main), not part of the route.
ROLLER_NONE = 0.0
ROLLER_ENABLE = 1.0
ROLLER_DISABLE = -1.0
# Enable and take the PTO even off a moving lift: the firmware cancels the lift, which
# loses its winch hold. Lasts until the next lift command or a Disable.
ROLLER_FORCE = 2.0

# chainbar codes, matching CHAINBAR_* in robot.cpp (Chainbar::Pose). 0 is no command:
# the bar holds the last pose. Pickups and dormant open the claw; a prime closes it
# and holds the bar for Chainbar::CLAW_CLOSE_MS before it swings.
CHAINBAR = "chainbar"
CHAINBAR_NONE = 0.0
CHAINBAR_DORMANT = -1.0
CHAINBAR_STACK = 1.0
CHAINBAR_PIN = 2.0
CHAINBAR_ALLIANCE_EMPTY = 3.0
CHAINBAR_ALLIANCE_PIN = 4.0
CHAINBAR_NEUTRAL_PIN = 5.0

# sticks codes, matching STICKS_* in robot.cpp. 0 is no command, so the piston stays
# as the last point (or boot) left it. Down is retracted, which is how the robot boots.
STICKS = "sticks"
STICKS_NONE = 0.0
STICKS_UP = 1.0
STICKS_DOWN = -1.0

# wait_lift: the robot holds at the point until the lift is done (or the firmware's
# LIFT_WAIT_TIMEOUT_MS runs out), then carries on. A score is done once its clamp has
# opened -- the macro only goes home after the robot drives off the tower -- and any
# other lift move once it settles. Only meaningful where the robot is stopped, so it
# makes a node stop by itself (Node.auto_stop).
WAIT_LIFT = "wait_lift"
WAIT_LIFT_OFF = 0.0
WAIT_LIFT_ON = 1.0

# Winch degrees above home. Lift::MAX_HEIGHT_DEG caps the top; a target inside
# Lift::HOME_TOLERANCE_DEG is already home, which is what lift_state = -1 is for.
MAX_LIFT_HEIGHT_DEG = 2000.0
MIN_LIFT_HEIGHT_DEG = 5.0

# Simulated lift modes, used by lint() to tell whether a macro can fire
MODE_PRIMED = "PRIMED"
MODE_MATCHLOAD = "MATCHLOAD"


@dataclass(frozen=True)
class ActionSpec:
    """How one named action is edited and displayed."""

    kind: str = NUMBER
    choices: Tuple[Tuple[str, float], ...] = ()
    minimum: float = -1000.0
    maximum: float = 1000.0
    decimals: int = 2
    suffix: str = ""
    prompt: str = ""

    def label_for(self, value: float) -> str:
        """Menu text for a value: the choice name, or the number and its unit."""
        for label, choice_value in self.choices:
            if _same(value, choice_value):
                return label
        if self.kind == CHOICE:
            return f"{_number(value)} (unrecognised)"
        return f"{_number(value)}{self.suffix}"

    def value_for(self, label: str) -> float:
        for choice_label, value in self.choices:
            if choice_label == label:
                return value
        raise KeyError(label)


DEFAULT_SPEC = ActionSpec()

RESET_SPEC = ActionSpec(
    kind=CHOICE,
    choices=(("Off", 0.0), ("Reset", 1.0)),
    prompt="Reset odometry from this sensor:",
)

SPECS: Dict[str, ActionSpec] = {
    "intake": ActionSpec(
        kind=CHOICE,
        choices=(("Off", 0.0), ("Intake", 1.0), ("Outtake", 2.0)),
        prompt="Intake state:",
    ),
    "lift_state": ActionSpec(
        kind=CHOICE,
        choices=(
            ("None", LIFT_NONE),
            ("Home (matchload)", LIFT_HOME),
            ("Score", LIFT_SCORE),
            ("Matchload", LIFT_MATCHLOAD),
            ("Matchload (vertical stack)", LIFT_MATCHLOAD_STACK),
            ("Matchload (lift while closing)", LIFT_MATCHLOAD_FAST),
            ("Matchload (claw closed)", LIFT_MATCHLOAD_CLOSED),
            ("Close claw", LIFT_GRAB),
        ),
        prompt="Lift macro to start here:",
    ),
    "lift_height": ActionSpec(
        kind=NUMBER,
        minimum=0.0,
        maximum=MAX_LIFT_HEIGHT_DEG,
        decimals=0,
        suffix=" deg",
        prompt="Winch height above home (0 = no command):",
    ),
    "intake_lift": ActionSpec(
        kind=CHOICE,
        choices=(
            ("None", INTAKE_LIFT_NONE),
            ("Raise (auto-drops)", INTAKE_LIFT_UP),
            ("Lower", INTAKE_LIFT_DOWN),
        ),
        prompt="Intake lift (a raised intake drops itself once it sees a piece):",
    ),
    "roller": ActionSpec(
        kind=CHOICE,
        choices=(
            ("None", ROLLER_NONE),
            ("Enable", ROLLER_ENABLE),
            ("Enable + force PTO (cancels lift)", ROLLER_FORCE),
            ("Disable", ROLLER_DISABLE),
        ),
        prompt="Roller mech (turns the roller to the firmware's target colour):",
    ),
    CHAINBAR: ActionSpec(
        kind=CHOICE,
        choices=(
            ("None", CHAINBAR_NONE),
            ("Pickup: stack", CHAINBAR_STACK),
            ("Pickup: pin", CHAINBAR_PIN),
            ("Prime: alliance empty", CHAINBAR_ALLIANCE_EMPTY),
            ("Prime: alliance pin", CHAINBAR_ALLIANCE_PIN),
            ("Prime: neutral pin", CHAINBAR_NEUTRAL_PIN),
            ("Dormant", CHAINBAR_DORMANT),
        ),
        prompt="Chainbar pose (pickups open the claw, primes close it first):",
    ),
    STICKS: ActionSpec(
        kind=CHOICE,
        choices=(
            ("None", STICKS_NONE),
            ("Up", STICKS_UP),
            ("Down", STICKS_DOWN),
        ),
        prompt="Sticks piston (down is retracted, as at boot):",
    ),
    WAIT_LIFT: ActionSpec(
        kind=CHOICE,
        choices=(("Off", WAIT_LIFT_OFF), ("Wait for lift", WAIT_LIFT_ON)),
        prompt="Hold here until the lift is done (a score: once the clamp opens):",
    ),
    "right_reset": RESET_SPEC,
    "left_reset": RESET_SPEC,
    "front_reset": RESET_SPEC,
}


def spec_for(name: str) -> ActionSpec:
    return SPECS.get(name, DEFAULT_SPEC)


def format_value(name: str, value: float) -> str:
    return spec_for(name).label_for(value)


def waits_for_lift(actions: Dict[str, float]) -> bool:
    return bool(actions.get(WAIT_LIFT, 0.0))


def check_stop(actions: Dict[str, float], stops: bool) -> List[str]:
    """Actions that need the robot at rest, on a point where it is still moving."""
    if waits_for_lift(actions) and not stops:
        return [
            "wait for lift is ignored because the robot does not stop here; "
            "turn its stop on"
        ]
    return []


def check_point(actions: Dict[str, float]) -> List[str]:
    """Contradictions inside a single point's action values."""
    warnings: List[str] = []
    lift_state = actions.get("lift_state", 0.0)
    height = actions.get("lift_height", 0.0)

    # A score may carry a height: Lift::score_at lowers to it and releases there,
    # ignoring the tower sensor (for a goal the sensor misreads). Any other macro
    # owns the point and the firmware drops the height.
    if lift_state and height and not _same(lift_state, LIFT_SCORE):
        warnings.append(
            f"lift_height {_number(height)} is ignored because lift_state is "
            f"{format_value('lift_state', lift_state)}; split them across two points"
        )
    if lift_state and not _known_lift_state(lift_state):
        warnings.append(f"lift_state {_number(lift_state)} is not a code the firmware knows")
    if height and not 0 < height <= MAX_LIFT_HEIGHT_DEG:
        warnings.append(
            f"lift_height {_number(height)} is outside 0-{_number(MAX_LIFT_HEIGHT_DEG)} deg"
        )
    elif 0 < height <= MIN_LIFT_HEIGHT_DEG:
        warnings.append(
            f"lift_height {_number(height)} deg is already home; use lift_state = Home instead"
        )
    intake_lift = actions.get("intake_lift", 0.0)
    if intake_lift and not _known_intake_lift(intake_lift):
        warnings.append(f"intake_lift {_number(intake_lift)} is not a code the firmware knows")
    wait_lift = actions.get(WAIT_LIFT, 0.0)
    if wait_lift and not _same(wait_lift, WAIT_LIFT_ON):
        warnings.append(f"wait_lift {_number(wait_lift)} is not a code the firmware knows")
    roller = actions.get("roller", 0.0)
    if roller and not _known_roller(roller):
        warnings.append(f"roller {_number(roller)} is not a code the firmware knows")
    elif (_same(roller, ROLLER_ENABLE) or _same(roller, ROLLER_FORCE)) and actions.get(
        "intake", 0.0
    ):
        warnings.append(
            "the roller shares the PTO with the intake, so it waits until the intake stops"
        )
    if _same(roller, ROLLER_FORCE):
        if lift_state or height:
            warnings.append(
                "the lift command on this point takes the PTO straight back from the forced "
                "roller; put the force on a later point"
            )
        else:
            warnings.append(
                "force cancels any lift move and drops its winch hold: a raised lift falls"
            )
    chainbar = actions.get(CHAINBAR, 0.0)
    if chainbar and not _known_chainbar(chainbar):
        warnings.append(f"chainbar {_number(chainbar)} is not a code the firmware knows")
    sticks = actions.get(STICKS, 0.0)
    if sticks and not _known_sticks(sticks):
        warnings.append(f"sticks {_number(sticks)} is not a code the firmware knows")
    if (lift_state or height) and actions.get("intake", 0.0):
        warnings.append(
            "the lift takes the PTO here, so the intake stops until the lift settles"
        )
    return warnings


def order_points(
    node_actions: Sequence[Dict[str, float]],
    node_samples: Sequence[float],
    point_actions: Sequence[Dict[str, float]],
    point_samples: Sequence[float],
) -> List[Tuple[str, Dict[str, float]]]:
    """Label and order action values the way the exported trajectory runs them.

    The sample lists are the profile indices the export inserts at (nodes_map /
    actions_map); anything without one falls back to its own order.
    """
    ordered = []
    for i, actions in enumerate(node_actions):
        sample = node_samples[i] if i < len(node_samples) else i
        ordered.append((sample, f"node {i + 1}", actions))
    for i, actions in enumerate(point_actions):
        sample = point_samples[i] if i < len(point_samples) else len(node_actions) + i
        ordered.append((sample, f"action point {i + 1}", actions))
    ordered.sort(key=lambda entry: entry[0])
    return [(where, actions) for _, where, actions in ordered]


def lint(points: Sequence[Tuple[str, Dict[str, float]]]) -> List[str]:
    """Check action points in execution order.

    ``points`` is ``(where, actions)`` pairs in the order the firmware will run
    them. Tracks the lift mode the same way ``Lift`` does, so a macro that the
    robot would refuse (scoring with nothing primed) is caught before a match.
    """
    warnings: List[str] = []
    # The firmware boots the lift in MATCHLOAD (Lift::initialize)
    mode = MODE_MATCHLOAD

    for where, actions in points:
        for message in check_point(actions):
            warnings.append(f"{where}: {message}")

        lift_state = actions.get("lift_state", 0.0)
        height = actions.get("lift_height", 0.0)
        if _same(lift_state, LIFT_HOME):
            mode = MODE_MATCHLOAD
        elif _same(lift_state, LIFT_SCORE):
            if mode != MODE_PRIMED:
                warnings.append(
                    f"{where}: score is ignored while the lift is {mode}; prime it first "
                    "(matchload, or a lift_height)"
                )
            else:
                # A score always comes back to the matchload pose
                mode = MODE_MATCHLOAD
        elif (_same(lift_state, LIFT_MATCHLOAD) or _same(lift_state, LIFT_MATCHLOAD_STACK)
              or _same(lift_state, LIFT_MATCHLOAD_FAST)
              or _same(lift_state, LIFT_MATCHLOAD_CLOSED)):
            mode = MODE_PRIMED
        elif _same(lift_state, LIFT_GRAB):
            # Holds the claw closed without changing the pose; a later lift_height
            # primes as normal. Only the matchload rest pose honours it.
            if mode != MODE_MATCHLOAD:
                warnings.append(
                    f"{where}: close claw is ignored while the lift is {mode}; "
                    "it only applies in the matchload rest pose"
                )
        elif height and not lift_state:
            mode = MODE_PRIMED
    return warnings


def _known_lift_state(value: float) -> bool:
    return any(
        _same(value, code)
        for code in (LIFT_HOME, LIFT_SCORE, LIFT_MATCHLOAD, LIFT_GRAB, LIFT_MATCHLOAD_STACK,
                     LIFT_MATCHLOAD_FAST, LIFT_MATCHLOAD_CLOSED)
    )


def _known_intake_lift(value: float) -> bool:
    return any(_same(value, code) for code in (INTAKE_LIFT_UP, INTAKE_LIFT_DOWN))


def _known_roller(value: float) -> bool:
    return any(_same(value, code) for code in (ROLLER_ENABLE, ROLLER_DISABLE, ROLLER_FORCE))


def _known_chainbar(value: float) -> bool:
    return any(
        _same(value, code)
        for code in (CHAINBAR_DORMANT, CHAINBAR_STACK, CHAINBAR_PIN, CHAINBAR_ALLIANCE_EMPTY,
                     CHAINBAR_ALLIANCE_PIN, CHAINBAR_NEUTRAL_PIN)
    )


def _known_sticks(value: float) -> bool:
    return any(_same(value, code) for code in (STICKS_UP, STICKS_DOWN))


def _same(a: float, b: float) -> bool:
    # The firmware matches codes with a 1e-3 epsilon (robot.cpp LIFT_ACTION_EPSILON)
    return abs(a - b) < 1e-3


def _number(value: float) -> str:
    return f"{value:g}"
