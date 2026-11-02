"""Optimize a route's shape for time.

The optimizer adjusts, per node, only the settings the route leaves free:

- ``heading``: the tangent direction (bounded rotation),
- ``handles``: the incoming and outgoing tangent magnitudes,
- ``curvature``: the normal component of the node's geometric second derivative
  (``tangent_derivative``, dT/ds in 1/ft; the spline scales it by each side's
  handle squared, see ``QuinticHermiteSpline.fit``),
- ``position``: where the node is, within a radius. Locked unless the route frees it.

- ``spin_through``: whether a turn-around node (``spin_candidate``) may become a
  spin-through turn, and then its turn. See optimize_route.

Waits, stops and actions are never changed, nor are turns other than a spin-through
node's, so the result is an ordinary route: the trajectory format and the firmware are
untouched.

How it stays fast enough to search:

1. **Materialize.** Every node's effective tangent, handles and curvature are read
   off the fitted spline and written back as explicit values. With every knot
   explicit, a node's settings only shape its two neighbouring segments.
2. **Split at stops.** The robot is at rest at every stop node, so the route's time
   is the sum of independent stretches between stops (plus turns and waits, which
   the shape does not change). A setting only re-times the stretches that contain
   its node.
3. **Time a stretch with the forward-backward pass alone**, integrating distance
   over speed on its distance grid. That is continuous in the shape (the full
   profile's 10 ms sampling is not) and skips the rest of the profiler.

Each node may carry its own limits (route field ``optimize``, see ``node_limits``):
how far its heading may turn, how far its handles may scale, how far its curvature may
move, and how far the path near it may move (the corridor, interpolated along the
spline parameter between neighbouring nodes).

Guards, all relative to the route as it was, per stretch: the path stays within the
corridor of the original and inside the field walls, the tightest turn taken while
moving gets no tighter, and the curvature rate (the 99th-percentile |dk/ds|, which
the profiler barely charges for and RAMSETE struggles with) grows by no more than
``curvature_rate_margin`` (where a node moves: over the route's worst stretch). The robot's footprint gets no deeper into any of the
route's keep-out zones. A stretch containing a node that may move drops the corridor
(moving the node is the point); the keep-out zones and walls bound it instead, and
the search there restarts from ``seeds`` spread-out positions because the time is
far from convex in where a node sits. The final route is re-profiled in full to
report its time and feasibility warnings.

Pure numpy, no Qt. ``tools/optimize_route.py`` is the command-line front end.
"""

from __future__ import annotations

import copy
import math
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from motion_profiling_v2 import motion_profile_generator as mpg
from utilities import action_spec
from utilities import headless_profile as hp
from utilities import route_serializer as rs

HEADING = "heading"
HANDLES = "handles"
CURVATURE = "curvature"
POSITION = "position"
# Whether the search may make a turn-around node a spin-through turn (and choose its
# turn); see spin_candidate.
SPIN_THROUGH = "spin_through"
# What shapes the spline at a node that stays where it is.
SHAPE_SETTINGS = (HEADING, HANDLES, CURVATURE)
SETTINGS = SHAPE_SETTINGS + (POSITION, SPIN_THROUGH)
# Not a setting the search changes: how far the path near a node may move, inches.
CORRIDOR = "corridor"

FIELD_HALF_WIDTH_IN = 72.0
# Action-point actions that only work facing the way the route was drawn: a distance
# reset reads a wall along a fixed direction, and a lift macro or chainbar pose places
# on (or picks up from) a goal. The
# heading at an action point carrying one is held within action_point_heading_deg.
# (Nodes with any action have their heading locked outright; see free_settings.)
HEADING_SENSITIVE_ACTIONS = frozenset(
    {"lift_state", "right_reset", "left_reset", "front_reset", "wait_lift", "chainbar"}
)
# Samples slower than this (ft/s) are not "moving" for the radius and curvature-rate
# guards: an in-place turn or the last millimetres into a stop.
MOVING_SPEED = 0.1
# Distance grid for the forward-backward pass (ft), as the profiler uses.
DD = 0.005


@dataclass
class OptimizerOptions:
    corridor_in: float = 3.0
    heading_range_deg: float = 30.0
    # Each handle may scale by this much either way from where it started.
    handle_scale: Tuple[float, float] = (0.4, 2.5)
    # Allowed growth of each stretch's 99th-percentile |dk/ds| ...
    curvature_rate_margin: float = 1.10
    # ... but never below this (1/ft^2), so a near-straight stretch may still bend.
    curvature_rate_floor: float = 2.0
    # How far the heading may move at an action point carrying a heading-sensitive
    # action (HEADING_SENSITIVE_ACTIONS).
    action_point_heading_deg: float = 2.0
    # Settings the run may touch at all; a node's own locks narrow this further.
    settings: Tuple[str, ...] = SETTINGS
    max_sweeps: int = 40
    robot_size_in: float = 16.0
    # Robot length (along its heading) and width, inches: the footprint kept out of
    # the route's keep-out zones.
    footprint_in: Tuple[float, float] = (15.0, 16.0)
    # What "position": true frees a node to, inches.
    position_radius_in: float = 6.0
    # Heading range of a node that may move and whose heading is otherwise free by
    # default: where a node sits largely decides the direction it wants.
    position_heading_deg: float = 90.0
    # Extra starting layouts for the search when any node may move.
    seeds: int = 6
    # A candidate becomes a spin-through turn only if that saves at least this, s.
    spin_through_min_gain: float = 0.05
    # Sweeps of the local search that tries each spin-through candidate.
    spin_through_sweeps: int = 10


@dataclass
class NodeLimits:
    """How far the search may move one node's settings (0 / None = locked)."""

    heading_deg: float
    handle_scale: Optional[Tuple[float, float]]
    # Largest change of the curvature along the normal, 1/ft; math.inf leaves only the
    # built-in bound (|curvature| up to the larger of its own and 1 / half-track).
    curvature: float
    corridor_in: float
    # How far the node may move, inches (0 = locked).
    position_in: float = 0.0
    # May the search make it a spin-through turn (only matters for spin_candidate)?
    spin_through: bool = False

    def free(self) -> Dict[str, bool]:
        """Which of the shape settings may change (position is ``position_in``)."""
        return {
            HEADING: self.heading_deg > 0,
            HANDLES: self.handle_scale is not None,
            CURVATURE: self.curvature > 0,
        }


@dataclass
class NodeShape:
    """A node's effective tangent and curvature, explicit."""

    direction: np.ndarray  # unit, the incoming direction
    m_in: float
    m_out: float
    curvature: np.ndarray  # geometric dT/ds, 1/ft

    def copy(self) -> "NodeShape":
        return NodeShape(self.direction.copy(), self.m_in, self.m_out, self.curvature.copy())


@dataclass
class KnotSides:
    """The tangent and geometric curvature the fitted spline actually uses on each side
    of a knot (None where the path does not continue)."""

    in_vec: Optional[np.ndarray]
    out_vec: Optional[np.ndarray]
    in_curvature: Optional[np.ndarray]
    out_curvature: Optional[np.ndarray]


@dataclass
class Variable:
    node: int
    kind: str  # "heading", "log_in", "log_out", "kappa"
    lo: float
    hi: float
    step: float


@dataclass
class StretchResult:
    first: int
    last: int
    before: float
    after: float


@dataclass
class NodeChange:
    node: int
    heading_deg: float = 0.0
    in_scale: float = 1.0
    out_scale: float = 1.0
    curvature_delta: float = 0.0  # 1/ft along the normal
    dx_in: float = 0.0
    dy_in: float = 0.0
    # The turn the node now carries as a spin-through turn (None: unchanged), and
    # whether the run made it one.
    turn_deg: Optional[float] = None
    spin_through: bool = False

    def describe(self) -> str:
        parts = []
        if self.spin_through:
            parts.append(f"spin-through turn {self.turn_deg:+.0f} deg")
        elif self.turn_deg is not None:
            parts.append(f"turn {self.turn_deg:+.0f} deg")
        if math.hypot(self.dx_in, self.dy_in) > 1e-6:
            parts.append(
                f"moved {math.hypot(self.dx_in, self.dy_in):.1f} in ({self.dx_in:+.1f}, {self.dy_in:+.1f})"
            )
        if abs(self.heading_deg) > 1e-6:
            parts.append(f"heading {self.heading_deg:+.1f} deg")
        if abs(self.in_scale - 1) > 1e-6:
            parts.append(f"in handle x{self.in_scale:.2f}")
        if abs(self.out_scale - 1) > 1e-6:
            parts.append(f"out handle x{self.out_scale:.2f}")
        if abs(self.curvature_delta) > 1e-6:
            parts.append(f"curvature {self.curvature_delta:+.2f} 1/ft")
        return ", ".join(parts)


@dataclass
class OptimizationResult:
    route: rs.Route
    before_time: float
    after_time: float
    stretches: List[StretchResult]
    changes: List[NodeChange]
    warnings_before: List[str]
    warnings_after: List[str]
    # (action point index, heading change in degrees at its spline parameter)
    action_point_headings: List[Tuple[int, float]]
    variables: int
    evaluations: int
    seconds: float
    # Nodes left exactly as they were because the explicit model cannot reproduce them
    # (or a neighbour, or their stretch); see frozen_nodes.
    untouchable: List[int] = field(default_factory=list)
    # should_stop asked the search to finish before it converged; the result is the
    # best route found up to then.
    stopped_early: bool = False

    @property
    def saved(self) -> float:
        return self.before_time - self.after_time

    def warning_changes(self) -> List[Tuple[str, str]]:
        """("NEW" | "CHANGED" | "GONE", warning) for each feasibility warning that
        differs between the route before and after. Warnings are matched with their
        numbers taken out, so a value shifting is CHANGED rather than NEW."""
        kind = lambda w: re.sub(r"[-+]?\d+(\.\d+)?", "#", w)  # noqa: E731
        before = {kind(w): w for w in self.warnings_before}
        after = {kind(w): w for w in self.warnings_after}
        out = []
        for k, w in after.items():
            if k not in before:
                out.append(("NEW", w))
            elif w != before[k]:
                out.append(("CHANGED", w))
        out += [("GONE", w) for k, w in before.items() if k not in after]
        return out


# ---------------------------------------------------------------------------
# Locks
# ---------------------------------------------------------------------------


def heading_locked_by_default(rec: rs.NodeRecord) -> bool:
    """The start node (where the robot is placed), the end node, and any node that
    carries an action (scoring, matchloads and distance resets all need the robot
    facing a particular way)."""
    return rec.is_start or rec.is_end or any(rec.actions.values())


def stops_at(rec: rs.NodeRecord) -> bool:
    """Whether the robot comes to rest at the node (headless_profile.HeadlessNode.stop)."""
    if rec.stop_mode == hp.STOP_FORCED:
        return True
    if rec.stop_mode == hp.STOP_DISABLED:
        return False
    return rec.is_reverse or rec.turn != 0 or rec.is_end or action_spec.waits_for_lift(rec.actions)


def spin_candidate(rec: rs.NodeRecord) -> bool:
    """A node the search may turn into a spin-through turn: a reversal or turn in the
    middle of the route that the robot only stops at to turn around. A node with an
    action, a wait or a lift wait stays as drawn (the robot is meant to be there, at
    that heading), as do the start and end."""
    if rec.is_start or rec.is_end or not (rec.is_reverse or rec.turn != 0):
        return False
    if rec.wait_time > 0 or any(rec.actions.values()):
        return False
    return rec.stop_mode != hp.STOP_DISABLED


def curvature_locked_by_default(rec: rs.NodeRecord) -> bool:
    """The start and every node the robot stops at. Curvature there makes the robot
    set off from rest already turning, which the drivetrain cannot do: far_fourpin's
    reverse arc drifted 2-3 in off every run until its reversal node's curvature went
    to 0. A lock keeps the value as drawn."""
    return rec.is_start or stops_at(rec)


def node_limits(rec: rs.NodeRecord, options: Optional[OptimizerOptions] = None) -> NodeLimits:
    """How far the optimizer may move a node's settings.

    Defaults come from ``options`` (the run's ranges and corridor), with the heading
    locked where ``heading_locked_by_default`` says. The route's ``optimize`` field
    overrides each: ``false`` locks, ``true`` frees to the run's range, a number frees
    to that much (heading in degrees, handles as a scale factor >= 1 either way,
    curvature in 1/ft, position as a radius in inches), and ``corridor`` sets the
    node's corridor in inches. Curvature is locked by default where
    ``curvature_locked_by_default`` says, position everywhere.
    """
    options = options or OptimizerOptions()
    o = rec.optimize

    def pick(name, default_free, default_amount, locked_amount):
        value = o.get(name, default_free)
        if value is True:
            return default_amount
        if value is False:
            return locked_amount
        return float(value)

    position = pick(POSITION, False, options.position_radius_in, 0.0)
    heading = pick(HEADING, not heading_locked_by_default(rec), options.heading_range_deg, 0.0)
    if position > 0 and HEADING not in o and not heading_locked_by_default(rec):
        heading = max(heading, options.position_heading_deg)
    handles_value = o.get(HANDLES, True)
    if handles_value is True:
        handle_scale = tuple(options.handle_scale)
    elif handles_value is False or float(handles_value) <= 1.0:
        handle_scale = None
    else:
        handle_scale = (1.0 / float(handles_value), float(handles_value))
    curvature = pick(CURVATURE, not curvature_locked_by_default(rec), math.inf, 0.0)
    corridor = o.get(CORRIDOR)
    corridor = options.corridor_in if corridor is None or isinstance(corridor, bool) else float(corridor)
    # A node whose heading the route locks arrives at that heading on purpose; spinning
    # through it would leave the robot mid-turn there.
    spin_through = spin_candidate(rec) and o.get(SPIN_THROUGH, True) is not False and heading > 0
    return NodeLimits(heading, handle_scale, curvature, corridor, position, spin_through)


def free_settings(rec: rs.NodeRecord) -> Dict[str, bool]:
    """Which of a node's settings the optimizer may change (see ``node_limits``)."""
    return node_limits(rec).free()


def describe_limits(rec: rs.NodeRecord, options: Optional[OptimizerOptions] = None) -> str:
    """What a node may change, with the amounts the route sets itself, e.g.
    "heading +-10 deg, handles, curvature, corridor 1.5 in"."""
    limits = node_limits(rec, options)
    free = limits.free()
    numeric = {k for k, v in rec.optimize.items() if not isinstance(v, bool)}
    if limits.heading_deg not in (0, (options or OptimizerOptions()).heading_range_deg):
        numeric.add(HEADING)  # widened for a node that may move
    parts = []
    if free[HEADING]:
        parts.append(f"heading +-{limits.heading_deg:g} deg" if HEADING in numeric else HEADING)
    if free[HANDLES]:
        parts.append(f"handles x{limits.handle_scale[1]:g}" if HANDLES in numeric else HANDLES)
    if free[CURVATURE]:
        parts.append(f"curvature +-{limits.curvature:g}/ft" if CURVATURE in numeric else CURVATURE)
    if limits.position_in > 0:
        parts.append(f"position {limits.position_in:g} in")
        numeric.add(POSITION)
    if limits.spin_through:
        parts.append("spin-through")
    text = ",".join(parts) if not numeric else ", ".join(parts)
    text = text or "locked"
    if CORRIDOR in rec.optimize:
        text += f", corridor {limits.corridor_in:g} in"
    return text


# ---------------------------------------------------------------------------
# Materialize and apply
# ---------------------------------------------------------------------------


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _rotation(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _normal(direction: np.ndarray) -> np.ndarray:
    return np.array([-direction[1], direction[0]])


def knot_sides(manager) -> List[KnotSides]:
    """What the fitted spline uses at each knot, read off its segments."""
    sides = [KnotSides(None, None, None, None) for _ in manager.nodes]
    start = 0
    for spline in manager.splines:
        for j, seg in enumerate(spline.segments):
            seg = np.asarray(seg, float)
            m0, m1 = np.linalg.norm(seg[2]), np.linalg.norm(seg[3])
            sides[start + j].out_vec = seg[2]
            sides[start + j].out_curvature = seg[4] / m0**2 if m0 > 1e-12 else np.zeros(2)
            sides[start + j + 1].in_vec = seg[3]
            sides[start + j + 1].in_curvature = seg[5] / m1**2 if m1 > 1e-12 else np.zeros(2)
        start += len(spline.segments)
    return sides


def materialize(manager) -> List[NodeShape]:
    """Every node's tangent, handles and curvature as the fitted spline uses them.

    Exact for every node whose two sides follow the node model (parallel, flipped at a
    reverse node, rotated at a turn); ``frozen_nodes`` finds the ones that do not.
    """
    shapes = []
    for k in knot_sides(manager):
        ref, curvature = (k.in_vec, k.in_curvature) if k.in_vec is not None else (k.out_vec, k.out_curvature)
        m_in = float(np.linalg.norm(ref))
        m_out = m_in if k.out_vec is None else float(np.linalg.norm(k.out_vec))
        shapes.append(NodeShape(_unit(ref), m_in, m_out, np.asarray(curvature, float)))
    return shapes


def apply_shapes(nodes: Sequence[hp.HeadlessNode], shapes: Sequence[NodeShape], frozen=()):
    """Copies of the nodes with every shape written in as explicit overrides.

    Frozen nodes keep their own settings: an explicit tangent cannot reproduce them.
    """
    out = copy.deepcopy(list(nodes))
    for i, (node, shape) in enumerate(zip(out, shapes)):
        if i in frozen:
            continue
        node.tangent = shape.direction.copy()
        node.incoming_magnitude = shape.m_in
        node.outgoing_magnitude = shape.m_out
        node.curvature = shape.curvature.copy()
    return out


def frozen_nodes(nodes, action_points, shapes) -> set:
    """Nodes the explicit model cannot reproduce, e.g. a zero outgoing handle mid-route
    (the spline falls back to the chord direction, so the two sides are not parallel)."""
    reference = [np.asarray(s.segments, float) for s in hp.build_spline_manager(nodes, action_points).splines]
    frozen: set = set()
    for _ in range(len(nodes)):
        rebuilt = hp.build_spline_manager(apply_shapes(nodes, shapes, frozen), action_points)
        bad = set()
        start = 0
        for ref, spline in zip(reference, rebuilt.splines):
            err = np.abs(ref - np.asarray(spline.segments, float)).max(axis=(1, 2))
            bad |= {start + j + d for j in np.flatnonzero(err > 1e-9) for d in (0, 1)}
            start += len(ref)
        if not bad - frozen:
            return frozen
        frozen |= bad
    return frozen


def _outgoing_vector(node_index: int, node: hp.HeadlessNode, shape: NodeShape) -> np.ndarray:
    """Where the path leaves a node, as build_path turns the node's tangent. The start
    node's reverse and turn flags are never applied (build_path starts at node 1)."""
    direction = shape.direction
    if node_index > 0 and node.turn != 0:
        angle = math.radians(node.turn) + (math.pi if node.is_reverse_node else 0.0)
        direction = _rotation(angle) @ direction
    elif node_index > 0 and node.is_reverse_node:
        direction = -direction
    return direction * shape.m_out


# ---------------------------------------------------------------------------
# Stretches
# ---------------------------------------------------------------------------


@dataclass
class _Stretch:
    first: int
    last: int
    ends_at_rest: bool
    action_points: List[hp.HeadlessActionPoint]


def stretches_of(nodes: Sequence[hp.HeadlessNode], action_points) -> List[_Stretch]:
    """Split a route at its stop nodes; the robot is at rest at each boundary."""
    bounds = [0] + [i for i in range(1, len(nodes) - 1) if nodes[i].stop] + [len(nodes) - 1]
    out = []
    for a, b in zip(bounds, bounds[1:]):
        aps = []
        for ap in action_points:
            if a < ap.t < b:
                local = copy.copy(ap)
                local.t = ap.t - a
                aps.append(local)
        ends_at_rest = b < len(nodes) - 1 or nodes[b].stop
        out.append(_Stretch(a, b, ends_at_rest, aps))
    return out


def _stretch_nodes(nodes, shapes, stretch: _Stretch, frozen, fixed_sides):
    """The stretch as a route of its own that the spline builder shapes identically.

    The ends are rewritten as plain start/end nodes carrying the tangent the full route
    actually leaves the first node along and arrives at the last one along, so reverse
    and turn handling at the boundaries does not come into it.
    """
    a, b = stretch.first, stretch.last
    sub = apply_shapes(nodes[a : b + 1], shapes[a : b + 1], {i - a for i in frozen if a <= i <= b})
    head, tail = sub[0], sub[-1]
    if a in frozen:
        out_vec, out_curvature = fixed_sides[a].out_vec, fixed_sides[a].out_curvature
    else:
        out_vec, out_curvature = _outgoing_vector(a, nodes[a], shapes[a]), shapes[a].curvature
    if b in frozen:
        in_vec, in_curvature = fixed_sides[b].in_vec, fixed_sides[b].in_curvature
    else:
        in_vec, in_curvature = shapes[b].direction * shapes[b].m_in, shapes[b].curvature
    head.tangent, head.curvature = _unit(out_vec), np.asarray(out_curvature, float)
    head.incoming_magnitude = head.outgoing_magnitude = float(np.linalg.norm(out_vec))
    tail.tangent, tail.curvature = _unit(in_vec), np.asarray(in_curvature, float)
    tail.incoming_magnitude = tail.outgoing_magnitude = float(np.linalg.norm(in_vec))
    for node in (head, tail):
        node.turn = 0
        node.is_reverse_node = False
    head.is_start_node, tail.is_end_node = True, True
    return sub


@dataclass
class _StretchEval:
    time: float
    points: np.ndarray  # dense path points, ft
    directions: np.ndarray  # unit path direction at each of the points
    min_radius: float  # ft, while moving
    curvature_rate: float  # 99th percentile |dk/ds|, 1/ft^2, while moving
    action_point_headings: np.ndarray  # rad, one per stretch action point
    # Deepest the footprint gets into each keep-out zone, inches (set by the search).
    keep_out_depth: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Where a spin-through turn sends the robot off the spline: the profile's poses
    # (ft) and body directions, for the walls and keep-out zones. Empty otherwise.
    swing_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    swing_directions: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    # Feasibility warnings of the full profile (only counted for a full evaluation).
    warnings: int = 0


def _evaluate_stretch(
    nodes, shapes, stretch: _Stretch, constraints, frozen=(), fixed_sides=None, full=False
) -> Optional[_StretchEval]:
    """Time and shape of one stretch. ``full`` times it with the whole profiler instead
    of the forward-backward pass: needed where spin-through turns join stretches, as
    only the profiler overlaps their spins with the ramps."""
    sub = _stretch_nodes(nodes, shapes, stretch, frozen, fixed_sides)
    try:
        manager = hp.build_spline_manager(sub, stretch.action_points)
        vel, _, dists = mpg.forward_backward_pass(
            manager, constraints, DD, end_vel=0.01 if stretch.ends_at_rest else None
        )
        profiled = hp.profile_route(manager, constraints) if full else None
    except Exception:
        return None
    v = np.maximum(np.asarray(vel, float), 1e-3)
    d = np.asarray(dists, float)
    if len(d) < 2:
        return None
    t = float(np.sum(2 * np.diff(d) / (v[1:] + v[:-1])))

    k = np.asarray(manager.get_curvatures(manager.distances_to_times(d)), float)
    moving = v >= MOVING_SPEED
    abs_k = np.abs(k)
    if full:
        # Around a spin-through turn the robot pivots slowly on purpose. Samples nearer
        # one than either end of the stretch, and inside the spin's envelope (as
        # check_trajectory_feasibility holds them), are the profiler's to police, not
        # the radius and curvature-rate guards'.
        node_d = mpg.forward_backward_pass(manager, constraints, DD)[1]
        spins = [
            d[node_d[i]] for i in range(1, len(sub) - 1)
            if i in node_d and mpg.blends_turn(sub[i], i, len(sub))
        ]
        if spins:
            near = np.min(np.abs(d[:, None] - np.array(spins)[None, :]), axis=1)
            nearer_spin = near < np.minimum(d - d[0], d[-1] - d)
            turn_cap = constraints.spin_through_rate_cap()
            inside = v / constraints.max_vel + v * abs_k / turn_cap <= 1.0
            moving = moving & ~(nearer_spin & inside)
    turning = moving & (abs_k > 1e-9)
    min_radius = float(np.min(1 / abs_k[turning])) if turning.any() else math.inf
    both = moving[1:] & moving[:-1]
    ds = np.diff(d)
    ok = both & (ds > 1e-6)
    rate = float(np.percentile(np.abs(np.diff(k))[ok] / ds[ok], 99)) if ok.any() else 0.0

    params = np.linspace(0, len(sub) - 1, 60 * (len(sub) - 1) + 1)
    points = manager.get_points_at_parameters(params)
    derivs = np.asarray(manager.get_derivatives_at_parameters(params), float)
    norms = np.linalg.norm(derivs, axis=1)
    directions = np.where(norms[:, None] > 1e-12, derivs / np.maximum(norms, 1e-12)[:, None], [1.0, 0.0])
    headings = np.array(
        [math.atan2(*manager.get_derivative_at_parameter(ap.t)[::-1]) for ap in stretch.action_points]
    )
    ev = _StretchEval(t, points, directions, min_radius, rate, headings)
    if profiled is not None:
        ev.time = float(profiled.times[-1])
        ev.swing_points = np.array([np.asarray(c, float).ravel()[:2] for c in profiled.coords])
        body = -np.asarray(profiled.headings, float)  # profile headings are negated
        ev.swing_directions = np.column_stack([np.cos(body), np.sin(body)])
        ev.warnings = len(profiled.report.warnings)
    return ev


# ---------------------------------------------------------------------------
# Keep-out zones
# ---------------------------------------------------------------------------


def keep_out_depth(zones: Sequence[rs.KeepOutRecord], centers_in, directions, footprint_in) -> np.ndarray:
    """How deep a rectangular footprint gets into each zone, inches (0 = clear).

    ``centers_in`` are robot centres in field inches, ``directions`` the unit heading
    at each (a footprint is symmetric, so driving backwards does not matter) and
    ``footprint_in`` the robot's (length along the heading, width). Circles take the
    distance from the centre to the rectangle; rectangles take the separating-axis
    overlap. Both give the depth of overlap, so a shallower clip is a smaller number.
    """
    if not len(zones) or not len(centers_in):
        return np.zeros(len(zones))
    c = np.asarray(centers_in, float)
    u = np.asarray(directions, float)
    n = np.column_stack([-u[:, 1], u[:, 0]])
    a, b = footprint_in[0] / 2, footprint_in[1] / 2
    out = []
    for zone in zones:
        if zone.shape == "circle":
            cx, cy, r = zone.values
            d = np.array([cx, cy]) - c
            lx, ly = (d * u).sum(1), (d * n).sum(1)
            gap = np.hypot(lx - np.clip(lx, -a, a), ly - np.clip(ly, -b, b))
            inside = np.minimum(a - np.abs(lx), b - np.abs(ly))
            depth = np.where(gap > 1e-12, r - gap, r + inside)
        else:
            x0, y0, x1, y1 = zone.values
            hx, hy = (x1 - x0) / 2, (y1 - y0) / 2
            d = np.array([(x0 + x1) / 2, (y0 + y1) / 2]) - c
            depth = np.minimum.reduce([
                hx + a * np.abs(u[:, 0]) + b * np.abs(n[:, 0]) - np.abs(d[:, 0]),
                hy + a * np.abs(u[:, 1]) + b * np.abs(n[:, 1]) - np.abs(d[:, 1]),
                a + hx * np.abs(u[:, 0]) + hy * np.abs(u[:, 1]) - np.abs((d * u).sum(1)),
                b + hx * np.abs(n[:, 0]) + hy * np.abs(n[:, 1]) - np.abs((d * n).sum(1)),
            ])
        out.append(max(0.0, float(depth.max())))
    return np.array(out)


# ---------------------------------------------------------------------------
# Search
# ---------------------------------------------------------------------------


def _variables(
    route: rs.Route, shapes: List[NodeShape], options: OptimizerOptions, half_track: float,
    spin_nodes=frozenset(),
):
    """The search's variables. A node in spin_nodes is a spin-through turn in the
    layout being searched: its turn is free, and unless the route sets its heading
    range, so is its heading by position_heading_deg (spinning through re-aims it
    along the chord between the stops either side)."""
    variables = []
    last = len(route.nodes) - 1
    for i, rec in enumerate(route.nodes):
        limits = node_limits(rec, options)
        if i in spin_nodes:
            if limits.heading_deg > 0 and HEADING not in rec.optimize:
                limits.heading_deg = max(limits.heading_deg, options.position_heading_deg)
            variables.append(Variable(i, "turn", -180.0, 180.0, 15.0))
        free = limits.free()
        allowed = lambda s: free[s] and s in options.settings  # noqa: E731
        if limits.position_in > 0 and POSITION in options.settings:
            r = limits.position_in / hp.INCHES_PER_FOOT
            step = min(r / 2, 2.0 / hp.INCHES_PER_FOOT)
            variables.append(Variable(i, "dx", -r, r, step))
            variables.append(Variable(i, "dy", -r, r, step))
        if allowed(HEADING):
            r = math.radians(limits.heading_deg)
            variables.append(Variable(i, "heading", -r, r, math.radians(min(8.0, limits.heading_deg))))
        if allowed(HANDLES):
            lo_s, hi_s = (math.log(s) for s in limits.handle_scale)
            step = min(0.3, (hi_s - lo_s) / 2)
            if i > 0:
                variables.append(Variable(i, "log_in", lo_s, hi_s, step))
            if i < last:
                variables.append(Variable(i, "log_out", lo_s, hi_s, step))
        if allowed(CURVATURE):
            k0 = float(shapes[i].curvature @ _normal(shapes[i].direction))
            bound = max(abs(k0), 1.0 / half_track)
            lo, hi = max(-bound - k0, -limits.curvature), min(bound - k0, limits.curvature)
            variables.append(Variable(i, "kappa", lo, hi, min(0.5, (hi - lo) / 2)))
    return variables


def _shape_with(base: NodeShape, values: Dict[str, float]) -> NodeShape:
    theta = values.get("heading", 0.0)
    rot = _rotation(theta)
    direction = rot @ base.direction
    curvature = rot @ base.curvature + values.get("kappa", 0.0) * _normal(direction)
    return NodeShape(
        direction,
        base.m_in * math.exp(values.get("log_in", 0.0)),
        base.m_out * math.exp(values.get("log_out", 0.0)),
        curvature,
    )


def _moved_nodes(nodes, values, spin_nodes=frozenset()):
    """The nodes with each one's dx/dy (ft) and turn (deg) applied, and spin_through
    set on spin_nodes; untouched nodes are shared."""
    out = list(nodes)
    for i, v in enumerate(values):
        spin = i in spin_nodes
        if v.get("dx") or v.get("dy") or "turn" in v or spin != nodes[i].spin_through:
            node = copy.copy(nodes[i])
            node.x += v.get("dx", 0.0)
            node.y += v.get("dy", 0.0)
            node.turn = v.get("turn", node.turn)
            node.spin_through = spin
            out[i] = node
    return out


@dataclass
class _Unit:
    """Consecutive stretches joined at spin-through turns, timed as one: the robot is
    not at rest between them, so they are not independent."""

    ks: Tuple[int, ...]  # indices into the route's stretches
    stretch: _Stretch  # the joined stretch


def _join(stretches: Sequence[_Stretch], ks) -> _Stretch:
    first, last = stretches[ks[0]].first, stretches[ks[-1]].last
    aps = []
    for k in ks:
        for ap in stretches[k].action_points:
            local = copy.copy(ap)
            local.t = ap.t + stretches[k].first - first
            aps.append(local)
    return _Stretch(first, last, stretches[ks[-1]].ends_at_rest, aps)


def units_of(stretches: Sequence[_Stretch], spin_nodes) -> List[_Unit]:
    """The stretches, with each pair that meets at a spin-through node joined."""
    groups: List[List[int]] = []
    for k, s in enumerate(stretches):
        if groups and s.first in spin_nodes:
            groups[-1].append(k)
        else:
            groups.append([k])
    return [_Unit(tuple(ks), _join(stretches, ks)) for ks in groups]


def _joined_eval(evals: Sequence[_StretchEval], ks) -> _StretchEval:
    """The stretches' own evaluations as one, laid end to end (the reference a joined
    unit's guards compare against)."""
    parts = [evals[k] for k in ks]
    depth = [p.keep_out_depth for p in parts if len(p.keep_out_depth)]
    return _StretchEval(
        time=sum(p.time for p in parts),
        points=np.vstack([parts[0].points] + [p.points[1:] for p in parts[1:]]),
        directions=np.vstack([parts[0].directions] + [p.directions[1:] for p in parts[1:]]),
        min_radius=min(p.min_radius for p in parts),
        curvature_rate=max(p.curvature_rate for p in parts),
        action_point_headings=np.concatenate([p.action_point_headings for p in parts]),
        keep_out_depth=np.max(depth, axis=0) if depth else np.zeros(0),
    )


def optimize_route(
    route: rs.Route,
    action_names: Sequence[str],
    constraints: mpg.Constraints,
    options: Optional[OptimizerOptions] = None,
    progress: Optional[Callable[[str], None]] = None,
    should_stop: Optional[Callable[[], bool]] = None,
) -> OptimizationResult:
    """Search the free settings for the fastest route that passes the guards.

    ``progress`` is called with a line of text after each sweep. ``should_stop`` is
    polled between trial steps; once it returns True the search ends and the best
    route found so far is returned (``stopped_early``).

    With ``spin_through`` among the run's settings, the search then tries each
    candidate node (spin_candidate) as a spin-through turn: several starting turns,
    each with the node's heading as drawn and aimed along the chord between the stops
    either side, a short local search over the nodes of the joined stretch, and the
    turn kept only if it beats stopping there by spin_through_min_gain. A final sweep
    searches everything together.
    """
    options = options or OptimizerOptions()
    started = time.time()
    nodes, action_points = hp.nodes_from_route(route)
    manager = hp.build_spline_manager(nodes, action_points)
    sides = knot_sides(manager)
    base_shapes = materialize(manager)
    frozen = frozen_nodes(nodes, action_points, base_shapes)
    stretches = stretches_of(nodes, action_points)
    half_track = constraints.track_width / 2
    # Corridor, ft, at a stretch's evaluation points (_evaluate_stretch samples 60 per
    # segment): each node's own, interpolated along the spline parameter.
    node_corridor_ft = np.array([node_limits(rec, options).corridor_in for rec in route.nodes]) / hp.INCHES_PER_FOOT

    def corridor_of(s: _Stretch) -> np.ndarray:
        return np.interp(
            np.linspace(0, s.last - s.first, 60 * (s.last - s.first) + 1),
            np.arange(s.last - s.first + 1),
            node_corridor_ft[s.first : s.last + 1],
        )

    field_limit_ft = (FIELD_HALF_WIDTH_IN - options.robot_size_in / 2) / hp.INCHES_PER_FOOT
    zones = list(route.keep_out)

    def turn_sweep(cur_nodes, shapes, s: _Stretch):
        """Footprint poses of the turn in place a stretch starts with, if any, inches."""
        a = s.first
        if a == 0 or not cur_nodes[a].turn:
            return np.zeros((0, 2)), np.zeros((0, 2))
        start = math.atan2(shapes[a].direction[1], shapes[a].direction[0])
        angles = start + np.linspace(0.0, math.radians(cur_nodes[a].turn), 25)
        center = np.array([cur_nodes[a].x, cur_nodes[a].y]) * hp.INCHES_PER_FOOT
        return np.tile(center, (len(angles), 1)), np.column_stack([np.cos(angles), np.sin(angles)])

    def evaluate(cur_nodes, shapes, s: _Stretch, full: bool):
        ev = _evaluate_stretch(cur_nodes, shapes, s, constraints, frozen, sides, full=full)
        if ev is not None and zones:
            centers, directions = turn_sweep(cur_nodes, shapes, s)
            ev.keep_out_depth = keep_out_depth(
                zones,
                np.vstack([ev.points * hp.INCHES_PER_FOOT, centers, ev.swing_points * hp.INCHES_PER_FOOT]),
                np.vstack([ev.directions, directions, ev.swing_directions]),
                options.footprint_in,
            )
        return ev

    # Each stretch on its own: the reference every guard measures against.
    stretch_base = [evaluate(nodes, base_shapes, s, False) for s in stretches]
    if any(e is None for e in stretch_base):
        raise ValueError("the route could not be profiled stretch by stretch")

    # Only search stretches whose standalone build is the route's own geometry; the
    # rest keep every node as it is.
    full_segments = [seg for spline in manager.splines for seg in np.asarray(spline.segments, float)]
    locked_nodes = set(frozen) | {j for i in frozen for j in (i - 1, i + 1)}
    for s in stretches:
        sub = hp.build_spline_manager(_stretch_nodes(nodes, base_shapes, s, frozen, sides), s.action_points)
        sub_segments = [seg for spline in sub.splines for seg in np.asarray(spline.segments, float)]
        expected = full_segments[s.first : s.last]
        if len(sub_segments) != len(expected) or any(
            np.abs(a - b).max() > 1e-9 for a, b in zip(sub_segments, expected)
        ):
            locked_nodes |= set(range(s.first, s.last + 1))

    stretch_sensitive = [
        np.array([any(ap.actions.get(a) for a in HEADING_SENSITIVE_ACTIONS) for ap in s.action_points], bool)
        for s in stretches
    ]
    ap_tolerance = math.radians(options.action_point_heading_deg)
    worst_rate = max(e.curvature_rate for e in stretch_base)
    spin_allowed = SPIN_THROUGH in options.settings
    route_spin = frozenset(i for i, n in enumerate(nodes) if mpg.blends_turn(n, i, len(nodes)))

    class Layout:
        """Everything that depends on which nodes are spin-through turns."""

        def __init__(self, spin):
            self.spin = frozenset(spin)
            self.units = units_of(stretches, self.spin)
            self.refs = [_joined_eval(stretch_base, u.ks) for u in self.units]
            for u, ref in zip(self.units, self.refs):
                ref.warnings = initial_warnings.get(u.ks, 0)
            self.corridors = [corridor_of(u.stretch) for u in self.units]
            self.sensitive = [np.concatenate([stretch_sensitive[k] for k in u.ks]) for u in self.units]
            turn_free = self.spin if spin_allowed else frozenset()
            self.variables = [
                v for v in _variables(route, base_shapes, options, half_track, turn_free)
                if v.node not in locked_nodes
            ]
            self.node_units = {
                i: [j for j, u in enumerate(self.units) if u.stretch.first <= i <= u.stretch.last]
                for i in range(len(nodes))
            }

        def full(self, j):
            return len(self.units[j].ks) > 1

        def evaluate(self, j, cur_nodes, shapes):
            return evaluate(cur_nodes, shapes, self.units[j].stretch, self.full(j))

    # Warnings the route's own spin-through stretches already have may stay.
    initial_warnings = {}
    for u in units_of(stretches, route_spin):
        if len(u.ks) > 1:
            ev = evaluate(nodes, base_shapes, u.stretch, True)
            if ev is None:
                raise ValueError("the route could not be profiled stretch by stretch")
            initial_warnings[u.ks] = ev.warnings
    layout = Layout(route_spin)

    radius_ft = {
        v.node: node_limits(route.nodes[v.node], options).position_in / hp.INCHES_PER_FOOT
        for v in layout.variables if v.kind == "dx"
    }

    def free_to_move(s: _Stretch) -> bool:
        """A stretch a movable node shapes: no corridor there, the zones and walls bound it."""
        return any(s.first <= i <= s.last for i in radius_ft)

    def guard_ok(lay: Layout, j: int, ev: _StretchEval) -> bool:
        return guard(lay, j, ev)[0]

    def guard(lay: Layout, j: int, ev: _StretchEval) -> Tuple[bool, str]:
        """Whether a unit's evaluation passes the guards, and if not, which one failed."""
        base = lay.refs[j]
        s = lay.units[j].stretch
        
        # Corridor check
        moving = free_to_move(s)
        if not moving:
            # Both ways: the new path stays near the old one, and still covers all of it.
            corridor = lay.corridors[j]
            dist = np.sqrt(((ev.points[:, None, :] - base.points[None, :, :]) ** 2).sum(-1))
            if (dist.min(1) > corridor + 1e-9).any() or (dist.min(0) > corridor + 1e-9).any():
                return False, "corridor"
            
        # Keep-out check
        if zones and (ev.keep_out_depth > base.keep_out_depth + 1e-6).any():
            return False, "keep-out"
        
        # Action heading check
        if lay.sensitive[j].any():
            turned = (ev.action_point_headings - base.action_point_headings + math.pi) % (2 * math.pi) - math.pi
            if np.abs(turned[lay.sensitive[j]]).max() > ap_tolerance + 1e-9:
                return False, "action heading"

        # Field limit check
        reach = max(np.abs(ev.points).max(), np.abs(ev.swing_points).max() if len(ev.swing_points) else 0.0)
        if reach > max(field_limit_ft, np.abs(base.points).max()) + 1e-9:
            return False, "field"
        
        # Radius check
        if ev.min_radius < min(base.min_radius, half_track) - 1e-6:
            return False, f"radius {ev.min_radius * 12:.1f} < {min(base.min_radius, half_track) * 12:.1f} in"
        
        # Warnings check
        if ev.warnings > base.warnings:
            return False, "warnings"
        
        # Curvature rate check
        # Where a node moves, the stretch's old shape is no reference; it may ask as much
        # of RAMSETE as the route's worst stretch already does.
        reference = worst_rate if moving else base.curvature_rate
        cap = max(options.curvature_rate_margin * reference, options.curvature_rate_floor)
        return ev.curvature_rate <= cap, f"curvature rate {ev.curvature_rate:.0f} > {cap:.0f}"

    evaluations = 0
    stopped = False

    def total(current):
        return sum(e.time for e in current.values())

    def search(lay: Layout, values, shapes, current, label, only=None, max_sweeps=None):
        """Compass search from one starting layout; returns where it ended. current maps
        each unit's stretch indices to its evaluation. only limits the search to those
        nodes' variables."""
        nonlocal evaluations, stopped
        variables = [v for v in lay.variables if only is None or v.node in only]
        steps = [v.step for v in variables]
        for sweep in range(max_sweeps or options.max_sweeps):
            improved = False
            for vi, var in enumerate(variables):
                if should_stop is not None and should_stop():
                    stopped = True
                    break
                for sign in (1, -1):
                    old = values[var.node].get(var.kind, 0.0)
                    new = min(max(old + sign * steps[vi], var.lo), var.hi)
                    if new == old:
                        continue
                    node_values = dict(values[var.node], **{var.kind: new})
                    if var.kind in ("dx", "dy") and math.hypot(
                        node_values.get("dx", 0.0), node_values.get("dy", 0.0)
                    ) > radius_ft[var.node] + 1e-12:
                        continue
                    trial_values = list(values)
                    trial_values[var.node] = node_values
                    trial_shapes = list(shapes)
                    trial_shapes[var.node] = _shape_with(base_shapes[var.node], node_values)
                    trial_nodes = _moved_nodes(nodes, trial_values, lay.spin)
                    affected = lay.node_units[var.node]
                    trial = {j: lay.evaluate(j, trial_nodes, trial_shapes) for j in affected}
                    evaluations += 1
                    if any(ev is None or not guard_ok(lay, j, ev) for j, ev in trial.items()):
                        continue
                    keys = [lay.units[j].ks for j in affected]
                    if sum(ev.time for ev in trial.values()) < sum(current[k].time for k in keys) - 1e-4:
                        values, shapes, current = trial_values, trial_shapes, dict(current)
                        for j, ev in trial.items():
                            current[lay.units[j].ks] = ev
                        improved = True
                        break
            if stopped:
                break
            if progress:
                progress(f"{label}sweep {sweep + 1}: {total(current):.3f} s of driving")
            if not improved:
                steps = [s / 2 for s in steps]
                if all(s < v.step / 16 for s, v in zip(steps, variables)):
                    break
        return values, shapes, current

    start_values = [dict() for _ in nodes]
    if spin_allowed:
        for i in layout.spin:
            if any(v.node == i and v.kind == "turn" for v in layout.variables):
                start_values[i]["turn"] = float(nodes[i].turn)

    def evaluate_all(lay: Layout, values, shapes):
        cur_nodes = _moved_nodes(nodes, values, lay.spin)
        return {u.ks: lay.evaluate(j, cur_nodes, shapes) for j, u in enumerate(lay.units)}

    base_current = evaluate_all(layout, start_values, base_shapes)
    starts = 1 + (options.seeds if radius_ft else 0)
    label = (lambda n: f"start {n}/{starts}, ") if starts > 1 else (lambda n: "")
    best = search(layout, start_values, [s.copy() for s in base_shapes], base_current, label(1))
    rng = np.random.default_rng(0)
    for n in range(2, starts + 1):
        if stopped:
            break
        # Every movable node somewhere else in its circle; everything else as drawn.
        values = [dict(v) for v in start_values]
        for i, r in radius_ft.items():
            angle = 2 * math.pi * ((n - 2) / options.seeds + rng.uniform(0, 1 / options.seeds))
            reach = r * math.sqrt(rng.uniform(0.25, 1.0))
            values[i].update(dx=reach * math.cos(angle), dy=reach * math.sin(angle))
        start_nodes = _moved_nodes(nodes, values, layout.spin)
        current = dict(base_current)
        for j, u in enumerate(layout.units):
            if free_to_move(u.stretch):
                current[u.ks] = layout.evaluate(j, start_nodes, base_shapes)
        evaluations += 1
        if any(current[u.ks] is None or not guard_ok(layout, j, current[u.ks]) for j, u in enumerate(layout.units)):
            continue
        found = search(layout, values, [s.copy() for s in base_shapes], current, label(n))
        if total(found[2]) < total(best[2]) - 1e-4:
            best = found
    values, shapes, current = best

    # Spin-through placement: each candidate, tried against stopping there.
    if spin_allowed:
        candidates = [
            i for i, rec in enumerate(route.nodes)
            if i not in layout.spin and i not in locked_nodes and nodes[i].stop
            and node_limits(rec, options).spin_through
        ]
        for c in candidates:
            if stopped:
                break
            trial_layout = Layout(layout.spin | {c})
            j = trial_layout.node_units[c][0]
            unit = trial_layout.units[j]
            # The same joined stretch with c still a stop, through the same profiler,
            # so the comparison is like for like.
            cur_nodes = _moved_nodes(nodes, values, layout.spin)
            reference = evaluate(cur_nodes, shapes, unit.stretch, True)
            if reference is None:
                continue
            others = {k: e for k, e in current.items() if not set(k) <= set(unit.ks)}
            before = total(others) + reference.time

            heading_var = next(
                (v for v in trial_layout.variables if v.node == c and v.kind == "heading"), None
            )
            a, b = cur_nodes[unit.stretch.first], cur_nodes[unit.stretch.last]
            chord = math.atan2(b.y - a.y, b.x - a.x)
            drawn = math.atan2(base_shapes[c].direction[1], base_shapes[c].direction[0])
            aim = 0.0
            if heading_var is not None:
                aim = (chord - drawn + math.pi) % (2 * math.pi) - math.pi
                aim = min(max(aim, heading_var.lo), heading_var.hi)
            seeds = []
            for h, t in ((aim, 180.0), (aim, -180.0), (aim, 135.0), (aim, -135.0),
                         (0.0, 180.0), (0.0, -180.0), (0.0, 90.0), (0.0, -90.0)):
                if (h, t) not in seeds:
                    seeds.append((h, t))
            # Where the node may go toward the middle of that chord, within its radius:
            # a flip wants to sit on the line between the stops.
            placements = [None]
            if c in radius_ft:
                dx0, dy0 = values[c].get("dx", 0.0), values[c].get("dy", 0.0)
                mid = np.array([(a.x + b.x) / 2, (a.y + b.y) / 2])
                here = np.array([nodes[c].x, nodes[c].y])
                want = mid - here
                reach = np.linalg.norm(want)
                if reach > radius_ft[c]:
                    want *= radius_ft[c] / reach
                if np.hypot(want[0] - dx0, want[1] - dy0) > 1e-6:
                    placements.append((float(want[0]), float(want[1])))
            tried = []
            for (h, t), place in ((seed, place) for seed in seeds for place in placements):
                trial_values = list(values)
                trial_values[c] = dict(values[c], turn=t)
                if heading_var is not None:
                    trial_values[c]["heading"] = h
                if place is not None:
                    trial_values[c].update(dx=place[0], dy=place[1])
                trial_shapes = list(shapes)
                trial_shapes[c] = _shape_with(base_shapes[c], trial_values[c])
                ev = trial_layout.evaluate(j, _moved_nodes(nodes, trial_values, trial_layout.spin), trial_shapes)
                evaluations += 1
                if ev is not None and guard_ok(trial_layout, j, ev):
                    tried.append((ev.time, trial_values, trial_shapes, ev))
            tried.sort(key=lambda x: x[0])
            if progress:
                starts_text = ", ".join(f"{t:.2f}" for t, *_ in tried[:3]) or "none pass the guards"
                progress(f"node {c + 1}: stopping there {reference.time:.2f} s; spin-through starts {starts_text}")
            outcome = None
            for _, trial_values, trial_shapes, ev in tried[:2]:
                start = dict(others)
                start[unit.ks] = ev
                found = search(
                    trial_layout, trial_values, trial_shapes, start,
                    f"node {c + 1} spin-through, ",
                    only=set(range(unit.stretch.first, unit.stretch.last + 1)),
                    max_sweeps=options.spin_through_sweeps,
                )
                if outcome is None or total(found[2]) < total(outcome[2]):
                    outcome = found
                if stopped:
                    break
            if outcome is not None and total(outcome[2]) < before - options.spin_through_min_gain:
                if progress:
                    progress(f"node {c + 1}: spin-through saves {before - total(outcome[2]):.2f} s")
                layout = trial_layout
                values, shapes, current = outcome
            elif progress:
                progress(f"node {c + 1}: spin-through not worth it")

        if layout.spin != route_spin and not stopped:
            values, shapes, current = search(layout, values, shapes, current, "together, ")

    turn_changed = {
        i for i in layout.spin
        if i not in route_spin or abs(values[i].get("turn", nodes[i].turn) - nodes[i].turn) > 1e-9
    }
    shape_keys = ("heading", "log_in", "log_out", "kappa", "dx", "dy")
    changed = sorted(
        {i for i, v in enumerate(values) if any(abs(v.get(k, 0.0)) > 1e-12 for k in shape_keys)} | turn_changed
    )
    moves = {i: (v.get("dx", 0.0), v.get("dy", 0.0)) for i, v in enumerate(values) if v.get("dx") or v.get("dy")}
    turns = {i: values[i].get("turn", nodes[i].turn) for i in turn_changed}
    out_route = write_shapes(route, shapes, changed, frozen, moves, turns)

    before = hp.profile_route(manager, constraints)
    after_nodes, after_aps = hp.nodes_from_route(out_route)
    after_manager = hp.build_spline_manager(after_nodes, after_aps)
    after = hp.profile_route(after_manager, constraints)
    if moves:
        # Action points ride on the spline parameter; keep their stored x, y on it.
        for rec in out_route.action_points:
            x, y = after_manager.get_point_at_parameter(rec.t) * hp.INCHES_PER_FOOT
            rec.x, rec.y = float(x), float(y)

    changes = [
        NodeChange(
            i,
            heading_deg=math.degrees(values[i].get("heading", 0.0)),
            in_scale=math.exp(values[i].get("log_in", 0.0)),
            out_scale=math.exp(values[i].get("log_out", 0.0)),
            curvature_delta=values[i].get("kappa", 0.0),
            dx_in=values[i].get("dx", 0.0) * hp.INCHES_PER_FOOT,
            dy_in=values[i].get("dy", 0.0) * hp.INCHES_PER_FOOT,
            turn_deg=turns.get(i),
            spin_through=i in layout.spin and i not in route_spin,
        )
        for i in changed
    ]
    ap_headings = []
    for j, ap in enumerate(action_points):
        a0 = manager.get_derivative_at_parameter(ap.t)
        a1 = after_manager.get_derivative_at_parameter(ap.t)
        delta = math.degrees(math.atan2(a1[1], a1[0]) - math.atan2(a0[1], a0[0]))
        ap_headings.append((j, (delta + 180) % 360 - 180))

    # Per final unit. A unit the run joined is compared with the same stretches as
    # drawn, through the profiler, so its stop and in-place turn count.
    unit_results = []
    for u in layout.units:
        if len(u.ks) > 1:
            was = evaluate(nodes, base_shapes, u.stretch, True)
            was_time = was.time if was is not None else sum(stretch_base[k].time for k in u.ks)
        else:
            was_time = stretch_base[u.ks[0]].time
        unit_results.append(StretchResult(u.stretch.first, u.stretch.last, was_time, current[u.ks].time))

    return OptimizationResult(
        route=out_route,
        before_time=before.times[-1],
        after_time=after.times[-1],
        stretches=unit_results,
        changes=changes,
        warnings_before=list(before.report.warnings),
        warnings_after=list(after.report.warnings),
        action_point_headings=ap_headings,
        variables=len(layout.variables),
        evaluations=evaluations,
        seconds=time.time() - started,
        untouchable=sorted(locked_nodes),
        stopped_early=stopped,
    )


def write_shapes(route, shapes, changed, frozen=(), moves=None, turns=None) -> rs.Route:
    """The route with the optimized shapes written in, touching as few nodes as possible.

    Changed nodes get explicit values. A default (unpinned) curvature is derived from the
    neighbouring tangents, so every node next to a changed one is pinned at what it
    already was; the rest keep their records exactly as they were. ``moves`` maps a
    node to its (dx, dy) in feet; ``turns`` maps a node made (or kept) a spin-through
    turn to its turn in degrees.
    """
    out = copy.deepcopy(route)
    touched = set(changed)
    touched |= {j for i in changed for j in (i - 1, i + 1) if 0 <= j < len(route.nodes)}
    for i in sorted(touched - set(frozen)):
        rec, shape = out.nodes[i], shapes[i]
        rec.tangent = [float(v) for v in shape.direction]
        rec.incoming_magnitude = float(shape.m_in)
        rec.outgoing_magnitude = float(shape.m_out)
        rec.tangent_derivative = [float(v) for v in shape.curvature]
    for i, (dx, dy) in (moves or {}).items():
        out.nodes[i].x += dx * hp.INCHES_PER_FOOT
        out.nodes[i].y += dy * hp.INCHES_PER_FOOT
    for i, turn in (turns or {}).items():
        out.nodes[i].turn = round(float(turn), 3)
        out.nodes[i].spin_through = True
    return out
