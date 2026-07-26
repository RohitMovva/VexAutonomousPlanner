"""Route file serialization: the single owner of the on-disk route format.

Current format ("vap-route" version 2) is a self-describing JSON object:

    {
      "format": "vap-route",
      "version": 2,
      "actions": ["intake", "little_will", ...],   # config order at save time
      "nodes": [
        {"x": 62.938, "y": 35.405, "start": true, "reverse": true,
         "tangent": [0.0, -0.646], "actions": {"intake": 1.0}},
        ...
      ],
      "action_points": [
        {"x": 10.0, "y": -4.0, "t": 1.52, "actions": {"rake": 1.0}},
        ...
      ]
    }

Design rules (see docs/route-format.md for the full spec):
- Action values are keyed by NAME, never by position, so editing
  robot.actions in config.yaml cannot corrupt saved routes.
- Fields equal to their default are omitted; parsers fill defaults back in.
  Unknown fields and unknown action names survive a load/save round trip.
- Positions are field inches, angles radians, matching the rest of the app.

The legacy format (version 1, produced before July 2026) was a positional
list ``[nodes_data, action_data]`` with a hardcoded 14-field node header
followed by unlabeled action values and optional locked/visible flags.
parse_route still reads it (best effort) so old files and shared strings
keep working; dumps_route only ever writes the current format.

This module is intentionally free of Qt and numpy so tools/tests can use it
without the GUI stack.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

FORMAT_NAME = "vap-route"
FORMAT_VERSION = 2

# Version reported for files in the pre-versioning positional-list format.
LEGACY_VERSION = 1


class RouteParseError(ValueError):
    """Raised when route text cannot be understood as any known format."""


@dataclass
class NodeRecord:
    x: float = 0.0
    y: float = 0.0
    is_start: bool = False
    is_end: bool = False
    is_reverse: bool = False
    stop_mode: int = 0
    turn: float = 0
    wait_time: float = 0.0
    max_velocity: float = 0.0
    max_acceleration: float = 0.0
    tangent: Optional[List[float]] = None
    tangent_derivative: Optional[List[float]] = None
    incoming_magnitude: Optional[float] = None
    outgoing_magnitude: Optional[float] = None
    locked: bool = False
    visible: bool = True
    actions: Dict[str, float] = field(default_factory=dict)


@dataclass
class ActionPointRecord:
    x: float = 0.0
    y: float = 0.0
    t: float = 0.0
    stop: bool = False
    wait_time: float = 0.0
    max_velocity: float = 0.0
    max_acceleration: float = 0.0
    actions: Dict[str, float] = field(default_factory=dict)


@dataclass
class Route:
    nodes: List[NodeRecord] = field(default_factory=list)
    action_points: List[ActionPointRecord] = field(default_factory=list)
    # Human-readable notes produced while parsing (legacy migration, unknown
    # actions, ignored fields). Deduplicated, in first-occurrence order.
    warnings: List[str] = field(default_factory=list)
    # Version of the file the route was parsed from (LEGACY_VERSION for the
    # old positional format); FORMAT_VERSION for freshly built routes.
    source_version: int = FORMAT_VERSION


# ---------------------------------------------------------------------------
# Serialization (always writes the current format)
# ---------------------------------------------------------------------------


def _clean_number(value):
    """Round float noise for stable, readable files; keep ints as ints."""
    if isinstance(value, bool) or not isinstance(value, float):
        return value
    rounded = round(value, 6)
    return int(rounded) if rounded.is_integer() else rounded


def _clean_vector(vec):
    return None if vec is None else [_clean_number(float(v)) for v in vec]


def _nonzero_actions(actions: Dict[str, float]) -> Dict[str, float]:
    return {name: _clean_number(value) for name, value in actions.items() if value}


def _node_to_dict(rec: NodeRecord) -> Dict[str, Any]:
    d: Dict[str, Any] = {
        "x": _clean_number(rec.x),
        "y": _clean_number(rec.y),
    }
    if rec.is_start:
        d["start"] = True
    if rec.is_end:
        d["end"] = True
    if rec.is_reverse:
        d["reverse"] = True
    if rec.stop_mode:
        d["stop_mode"] = int(rec.stop_mode)
    if rec.turn:
        d["turn"] = _clean_number(rec.turn)
    if rec.wait_time:
        d["wait_time"] = _clean_number(rec.wait_time)
    if rec.max_velocity:
        d["max_velocity"] = _clean_number(rec.max_velocity)
    if rec.max_acceleration:
        d["max_acceleration"] = _clean_number(rec.max_acceleration)
    if rec.tangent is not None:
        d["tangent"] = _clean_vector(rec.tangent)
    if rec.tangent_derivative is not None:
        d["tangent_derivative"] = _clean_vector(rec.tangent_derivative)
    if rec.incoming_magnitude is not None:
        d["incoming_magnitude"] = _clean_number(rec.incoming_magnitude)
    if rec.outgoing_magnitude is not None:
        d["outgoing_magnitude"] = _clean_number(rec.outgoing_magnitude)
    if rec.locked:
        d["locked"] = True
    if not rec.visible:
        d["visible"] = False
    actions = _nonzero_actions(rec.actions)
    if actions:
        d["actions"] = actions
    return d


def _action_point_to_dict(rec: ActionPointRecord) -> Dict[str, Any]:
    d: Dict[str, Any] = {
        "x": _clean_number(rec.x),
        "y": _clean_number(rec.y),
        "t": _clean_number(rec.t),
    }
    if rec.stop:
        d["stop"] = True
    if rec.wait_time:
        d["wait_time"] = _clean_number(rec.wait_time)
    if rec.max_velocity:
        d["max_velocity"] = _clean_number(rec.max_velocity)
    if rec.max_acceleration:
        d["max_acceleration"] = _clean_number(rec.max_acceleration)
    actions = _nonzero_actions(rec.actions)
    if actions:
        d["actions"] = actions
    return d


def dumps_route(
    route: Route, action_names: Sequence[str], compact: bool = False
) -> str:
    """Serialize a route in the current format.

    Default layout is one record per line (readable diffs, small files);
    compact=True emits a single line for clipboard sharing, since paste
    targets like QInputDialog mangle embedded newlines.
    """
    obj = {
        "format": FORMAT_NAME,
        "version": FORMAT_VERSION,
        "actions": list(action_names),
        "nodes": [_node_to_dict(n) for n in route.nodes],
        "action_points": [_action_point_to_dict(a) for a in route.action_points],
    }
    if compact:
        return json.dumps(obj, separators=(",", ":"))

    def enc(value):
        return json.dumps(value, separators=(",", ":"))

    def block(records):
        if not records:
            return "[]"
        lines = ",\n    ".join(enc(r) for r in records)
        return f"[\n    {lines}\n  ]"

    return (
        "{\n"
        f'  "format": {enc(obj["format"])},\n'
        f'  "version": {obj["version"]},\n'
        f'  "actions": {enc(obj["actions"])},\n'
        f'  "nodes": {block(obj["nodes"])},\n'
        f'  "action_points": {block(obj["action_points"])}\n'
        "}\n"
    )


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def parse_route(text: str, action_names: Sequence[str]) -> Route:
    """Parse route text in any supported format.

    action_names is the current config.yaml robot.actions list; it maps
    positional values in legacy files to names and flags actions a v2 file
    mentions that the config no longer defines.

    Raises RouteParseError if the text is not a route in any known format.
    """
    try:
        data = json.loads(text)
    except json.JSONDecodeError as e:
        raise RouteParseError(f"not valid JSON: {e}") from e

    if isinstance(data, dict):
        route = _parse_v2(data, action_names)
    elif isinstance(data, list):
        route = _parse_legacy(data, action_names)
    else:
        raise RouteParseError(
            f"expected a route object or legacy list, got {type(data).__name__}"
        )

    seen = set()
    route.warnings = [w for w in route.warnings if not (w in seen or seen.add(w))]
    return route


def _as_float(value, default=0.0) -> float:
    return default if value is None else float(value)


def _as_vector(value, what: str) -> Optional[List[float]]:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{what} must be a list of numbers, got {value!r}")
    return [float(v) for v in value]


# --- current format ---------------------------------------------------------

_KNOWN_NODE_KEYS = {
    "x", "y", "start", "end", "reverse", "stop_mode", "turn", "wait_time",
    "max_velocity", "max_acceleration", "tangent", "tangent_derivative",
    "incoming_magnitude", "outgoing_magnitude", "locked", "visible", "actions",
}

_KNOWN_ACTION_POINT_KEYS = {
    "x", "y", "t", "stop", "wait_time", "max_velocity", "max_acceleration",
    "actions",
}


def _parse_actions_dict(raw, known_names, where: str, warnings: List[str]):
    actions: Dict[str, float] = {}
    for name, value in (raw or {}).items():
        actions[str(name)] = float(value)
    unknown = sorted(n for n in actions if n not in known_names)
    if unknown:
        warnings.append(
            f"{where}: action(s) {', '.join(unknown)} are not in config.yaml "
            "robot.actions; the values are preserved in the file but will not "
            "be exported to the trajectory"
        )
    return actions


def _parse_v2(data: dict, action_names: Sequence[str]) -> Route:
    warnings: List[str] = []
    known_names = set(action_names)

    fmt = data.get("format")
    if fmt is not None and fmt != FORMAT_NAME:
        warnings.append(f"unrecognized format tag {fmt!r}; loading anyway")
    version = data.get("version")
    if not isinstance(version, int):
        warnings.append("route file has no version; treating as current")
        version = FORMAT_VERSION
    elif version > FORMAT_VERSION:
        warnings.append(
            f"route version {version} is newer than this planner supports "
            f"({FORMAT_VERSION}); unknown fields will be ignored"
        )

    nodes: List[NodeRecord] = []
    for i, raw in enumerate(data.get("nodes") or []):
        try:
            nodes.append(_parse_v2_node(raw, known_names, i, warnings))
        except (TypeError, ValueError, KeyError, AttributeError) as e:
            raise RouteParseError(f"node {i}: {e}") from e

    action_points: List[ActionPointRecord] = []
    for i, raw in enumerate(data.get("action_points") or []):
        try:
            action_points.append(
                _parse_v2_action_point(raw, known_names, i, warnings)
            )
        except (TypeError, ValueError, KeyError, AttributeError) as e:
            raise RouteParseError(f"action point {i}: {e}") from e

    return Route(nodes, action_points, warnings, source_version=version)


def _parse_v2_node(raw: dict, known_names, index: int, warnings) -> NodeRecord:
    if not isinstance(raw, dict):
        raise ValueError(f"expected an object, got {type(raw).__name__}")
    ignored = sorted(set(raw) - _KNOWN_NODE_KEYS)
    if ignored:
        warnings.append(f"node {index}: ignored unknown field(s) {', '.join(ignored)}")
    in_mag = raw.get("incoming_magnitude")
    out_mag = raw.get("outgoing_magnitude")
    return NodeRecord(
        x=float(raw["x"]),
        y=float(raw["y"]),
        is_start=bool(raw.get("start", False)),
        is_end=bool(raw.get("end", False)),
        is_reverse=bool(raw.get("reverse", False)),
        stop_mode=int(raw.get("stop_mode", 0)),
        turn=raw.get("turn", 0),
        wait_time=_as_float(raw.get("wait_time")),
        max_velocity=_as_float(raw.get("max_velocity")),
        max_acceleration=_as_float(raw.get("max_acceleration")),
        tangent=_as_vector(raw.get("tangent"), f"node {index} tangent"),
        tangent_derivative=_as_vector(
            raw.get("tangent_derivative"), f"node {index} tangent_derivative"
        ),
        incoming_magnitude=None if in_mag is None else float(in_mag),
        outgoing_magnitude=None if out_mag is None else float(out_mag),
        locked=bool(raw.get("locked", False)),
        visible=bool(raw.get("visible", True)),
        actions=_parse_actions_dict(
            raw.get("actions"), known_names, f"node {index}", warnings
        ),
    )


def _parse_v2_action_point(
    raw: dict, known_names, index: int, warnings
) -> ActionPointRecord:
    if not isinstance(raw, dict):
        raise ValueError(f"expected an object, got {type(raw).__name__}")
    ignored = sorted(set(raw) - _KNOWN_ACTION_POINT_KEYS)
    if ignored:
        warnings.append(
            f"action point {index}: ignored unknown field(s) {', '.join(ignored)}"
        )
    return ActionPointRecord(
        x=float(raw["x"]),
        y=float(raw["y"]),
        t=float(raw["t"]),
        stop=bool(raw.get("stop", False)),
        wait_time=_as_float(raw.get("wait_time")),
        max_velocity=_as_float(raw.get("max_velocity")),
        max_acceleration=_as_float(raw.get("max_acceleration")),
        actions=_parse_actions_dict(
            raw.get("actions"), known_names, f"action point {index}", warnings
        ),
    )


# --- legacy positional format ------------------------------------------------


def _parse_legacy(data: list, action_names: Sequence[str]) -> Route:
    warnings = [
        "loaded a legacy (pre-v2) route file; it will be rewritten in the "
        "current format on the next save (or run tools/migrate_routes.py)"
    ]

    # Two shapes exist: [nodes_data, action_data] and a bare nodes_data list.
    # In the two-element shape data[0] is a list of node records (lists of
    # lists); in the bare shape data[0] is itself a record (list of numbers).
    if (
        len(data) == 2
        and isinstance(data[0], list)
        and (not data[0] or isinstance(data[0][0], list))
        and isinstance(data[1], list)
    ):
        nodes_raw, action_points_raw = data
    else:
        nodes_raw, action_points_raw = data, []

    nodes = []
    for i, raw in enumerate(nodes_raw):
        rec = _parse_legacy_node(raw, action_names, i, warnings)
        if rec is not None:
            nodes.append(rec)

    action_points = []
    for i, raw in enumerate(action_points_raw):
        rec = _parse_legacy_action_point(raw, action_names, i, warnings)
        if rec is not None:
            action_points.append(rec)

    return Route(nodes, action_points, warnings, source_version=LEGACY_VERSION)


def _positional_actions(values, action_names, where: str, warnings) -> Dict[str, float]:
    """Map unlabeled legacy action values onto the current action names.

    Historically actions were only ever appended to config.yaml, so value i
    belongs to name i. Values beyond the configured names cannot be named
    and are dropped (loudly).
    """
    actions: Dict[str, float] = {}
    for i, value in enumerate(values):
        if not value:
            continue
        if i < len(action_names):
            actions[action_names[i]] = float(value)
        else:
            warnings.append(
                f"{where}: non-zero action value at position {i} has no "
                "matching entry in config.yaml robot.actions and was dropped"
            )
    return actions


def _parse_legacy_node(raw, action_names, index: int, warnings) -> Optional[NodeRecord]:
    if not isinstance(raw, list) or len(raw) < 5:
        warnings.append(f"node {index}: record too short, skipped")
        return None

    # 14-field header: x y start end reverse stop_mode turn wait max_vel
    # max_accel tangent tangent_derivative incoming_mag outgoing_mag
    head = list(raw[:14]) + [None] * (14 - min(len(raw), 14))
    rest = raw[14:]

    # The tail is action values, optionally followed by locked/visible flags
    # (newer writers). With n configured actions the observed layouts are
    # n values, n+1 (plus locked) or n+2 (plus locked and visible).
    n = len(action_names)
    locked, visible = None, None
    if len(rest) >= n + 2:
        values, locked, visible = rest[:n], rest[n], rest[n + 1]
        extra = rest[n + 2:]
        if any(extra):
            warnings.append(
                f"node {index}: {len(extra)} unrecognized trailing value(s) dropped"
            )
        rest = values
    elif len(rest) == n + 1:
        rest, locked = rest[:n], rest[n]

    tangent = raw[10] if len(raw) > 10 else None
    derivative = raw[11] if len(raw) > 11 else None
    # Old writers stored a fake [0, 0] derivative for "not set".
    if derivative is not None and not any(derivative):
        derivative = None

    return NodeRecord(
        x=float(head[0]),
        y=float(head[1]),
        is_start=bool(head[2]),
        is_end=bool(head[3]),
        is_reverse=bool(head[4]),
        stop_mode=int(head[5] or 0),
        turn=head[6] or 0,
        wait_time=_as_float(head[7]),
        max_velocity=_as_float(head[8]),
        max_acceleration=_as_float(head[9]),
        tangent=_as_vector(tangent, f"node {index} tangent"),
        tangent_derivative=_as_vector(derivative, f"node {index} derivative"),
        incoming_magnitude=None if head[12] is None else float(head[12]),
        outgoing_magnitude=None if head[13] is None else float(head[13]),
        locked=bool(locked) if locked is not None else False,
        visible=bool(visible) if visible is not None else True,
        actions=_positional_actions(
            rest, action_names, f"node {index}", warnings
        ),
    )


def _parse_legacy_action_point(
    raw, action_names, index: int, warnings
) -> Optional[ActionPointRecord]:
    if not isinstance(raw, list) or len(raw) < 3:
        warnings.append(f"action point {index}: record too short, skipped")
        return None

    # Header: x y t stop wait max_vel max_accel, then action values.
    head = list(raw[:7]) + [None] * (7 - min(len(raw), 7))
    return ActionPointRecord(
        x=float(head[0]),
        y=float(head[1]),
        t=float(head[2]),
        stop=bool(head[3]),
        wait_time=_as_float(head[4]),
        max_velocity=_as_float(head[5]),
        max_acceleration=_as_float(head[6]),
        actions=_positional_actions(
            raw[7:], action_names, f"action point {index}", warnings
        ),
    )
