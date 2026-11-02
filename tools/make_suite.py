#!/usr/bin/env python3
"""Generate the path-following test suite: one route per scenario, at the
difficulty the real routes actually demand.

    src/.venv/bin/python tools/make_suite.py            # writes suite/routes/*.txt and suite/manifest.yaml
    src/.venv/bin/python tools/make_suite.py --install  # also copies the routes into <routes folder>/suite/

Every route starts at the same pose as turn_test - field (0, 24) in route
inches, heading -x (the robot's (0, -24, pi)) - except cruise_8ft, which needs
the length and starts at (+48, 24), and the slip_* odometry tests, which start
near the robot-frame (-x, -y) corner so their holds can see two walls (see
slip_routes). Arc nodes pin the circle's tangent and curvature, so each arc holds
its radius exactly; short blends carry the curvature between segments of
different radius.

Measured from the saved competition routes (tools/profile_routes.py, 2026-09-12):
segments average 2.6 ft, almost every node is a reverse, turning radius runs
p25 0.54 ft / median 1.0 ft / p75 2.0 ft at speed, and sub-0.3 ft radii appear
only at crawl speed next to the cusps. The suite spans exactly those bands.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from utilities import headless_profile as hp  # noqa: E402
from utilities import route_serializer  # noqa: E402

SUITE_DIR = ROOT / "suite"
ROUTES_DIR = SUITE_DIR / "routes"
START = (0.0, 24.0)  # route inches; the robot sees (0, -24)
FT = 12.0


@dataclass
class NodeSpec:
    x: float
    y: float
    tangent: Optional[List[float]] = None
    # Geometric second derivative dT/ds (1/ft) pinned at the node; a circle's is the
    # unit normal toward the centre over R. Without it the spline's natural end
    # condition (zero curvature at the end nodes) bulges every arc.
    curvature: Optional[List[float]] = None
    in_mag: Optional[float] = None
    out_mag: Optional[float] = None
    start: bool = False
    end: bool = False
    reverse: bool = False
    max_velocity: float = 0.0
    # Per-node constraint override (ft/s^2): binds accelerating AND braking from this
    # node on (motion_profile_generator.Constraints.accel_limit / decel_limit).
    max_acceleration: float = 0.0
    turn: float = 0.0          # in-place turn at the node, degrees, route frame
    wait_time: float = 0.0     # dwell at the node, seconds (after any turn)
    stop_mode: int = 0         # 0 auto / 1 forced / 2 disabled (gui/node.py)
    actions: Dict[str, float] = field(default_factory=dict)


class PathBuilder:
    """Walk a path of straights and arcs, emitting nodes with exact tangents.

    Positions in route inches, heading as a unit vector. Arc nodes are placed
    at most every `max_arc_deg` so the quintic Hermite can hold the radius.
    Tangent magnitudes are the segment lengths in feet scaled by `k` (tuned by
    the caller); a node between two segments carries each side's own value.
    """

    def __init__(self, start=START, heading=(-1.0, 0.0), max_arc_deg=45.0):
        self.pos = np.array(start, dtype=float)
        self.heading = np.array(heading, dtype=float)
        self.max_arc_deg = max_arc_deg
        # Signed curvature (1/ft, left positive in the route frame) the path currently
        # carries; a node can pin only one value, so segments of different curvature
        # are joined by a short blend (see _blend).
        self.kappa = 0.0
        self.nodes: List[NodeSpec] = [NodeSpec(*self.pos, tangent=list(self.heading), in_mag=0.0, start=True,
                                               curvature=[0.0, 0.0])]
        self.action_points: List[dict] = []

    def _curv_vec(self, kappa: float):
        left = np.array([-self.heading[1], self.heading[0]])
        return [float(left[0] * kappa), float(left[1] * kappa)]

    def _turn(self, radius_in: float, angle_rad: float, sign: float, pieces: int, kappa_end: float):
        """Sweep a circular arc, emitting `pieces` nodes pinned to curvature kappa_end.

        Tangent magnitude = the piece's arc length in feet: with the curvature pinned
        that parametrises the quintic as the circle itself (radius exact to <1%)."""
        normal = np.array([-self.heading[1], self.heading[0]]) * sign
        centre = self.pos + normal * radius_in
        step = angle_rad / pieces * sign
        mag = radius_in * abs(step) / FT
        self.nodes[-1].out_mag = mag
        rot = np.array([[math.cos(step), -math.sin(step)], [math.sin(step), math.cos(step)]])
        for _ in range(pieces):
            self.pos = centre + rot @ (self.pos - centre)
            self.heading = rot @ self.heading
            self._emit(mag, mag, curvature=self._curv_vec(kappa_end))
        self.kappa = kappa_end

    def _blend(self, kappa_to: float, length_in: float):
        """Carry the curvature from its current value to kappa_to over length_in,
        approximated as an arc at the mean curvature; the end node pins kappa_to."""
        if abs(kappa_to - self.kappa) < 1e-9:
            return 0.0
        mean = 0.5 * (self.kappa + kappa_to)
        if abs(mean) < 1e-9:  # symmetric sign flip: straight geometry, curvature through zero
            self.pos = self.pos + self.heading * length_in
            self.nodes[-1].out_mag = length_in / FT
            self._emit(length_in / FT, None, curvature=self._curv_vec(kappa_to))
            self.kappa = kappa_to
            return 0.0
        angle = abs(mean) * length_in / FT
        self._turn(FT / abs(mean), angle, 1.0 if mean > 0 else -1.0, 1, kappa_to)
        return angle

    @staticmethod
    def blend_length(radius_in: float) -> float:
        return min(max(0.25 * radius_in, 1.5), 6.0)

    def _emit(self, in_mag: float, out_mag: Optional[float] = None, curvature=None):
        last = self.nodes[-1]
        last.out_mag = in_mag if last.out_mag is None else last.out_mag
        self.nodes.append(NodeSpec(*self.pos, tangent=list(self.heading), in_mag=in_mag, out_mag=out_mag,
                                   curvature=curvature))

    def straight(self, length_in: float):
        if self.kappa:
            length_in -= self.blend_length(FT / abs(self.kappa))
            self._blend(0.0, self.blend_length(FT / abs(self.kappa)))
        self.pos = self.pos + self.heading * length_in
        mag = length_in / FT
        self.nodes[-1].out_mag = mag
        self._emit(mag, curvature=[0.0, 0.0])
        return self

    def arc(self, radius_in: float, angle_deg: float, side: str, next_kappa: Optional[float] = 0.0):
        """Turn `angle_deg` in total about a centre `radius_in` to the `side` ('L'/'R',
        route frame). The heading the blends in and out consume is taken off the
        constant-radius part so the total turn stays exact; `next_kappa` is what
        follows (0 for a straight, None when the path ends on the arc), so the exit
        blend can be budgeted.

        A pure arc from rest (the path starts on the circle) pins the start node to
        the arc's curvature and needs no blend.
        """
        sign = 1.0 if side == "L" else -1.0
        kappa = sign * FT / radius_in
        b = self.blend_length(radius_in)
        if len(self.nodes) == 1 and self.kappa == 0.0:
            self.nodes[0].curvature = self._curv_vec(kappa)
            self.kappa = kappa
            blend_in = 0.0
        else:
            blend_in = self._blend(kappa, b)
        blend_out = 0.0 if next_kappa is None else abs(0.5 * (kappa + next_kappa)) * b / FT
        main = math.radians(abs(angle_deg)) - blend_in - blend_out
        pieces = max(1, math.ceil(math.degrees(main) / self.max_arc_deg))
        self._turn(radius_in, main, sign, pieces, kappa)
        self.nodes[-1].out_mag = None
        return self

    def reverse_here(self, actions=None):
        """Mark the last node as a reverse cusp. Its tangent is left to the spline
        manager, which bisects the legs the way the GUI does for real routes."""
        node = self.nodes[-1]
        node.reverse = True
        node.tangent = None
        node.curvature = None
        node.in_mag = node.out_mag = None
        self.kappa = 0.0
        if actions:
            node.actions.update(actions)
        # After a cusp the robot backs away: the path heading flips.
        self.heading = -self.heading
        return self

    def turn_to(self, heading_deg: float):
        """Set the path heading (route frame, degrees) for the next straight leg."""
        r = math.radians(heading_deg)
        self.heading = np.array([math.cos(r), math.sin(r)])
        return self

    def curvature_profile(self, s_in, kappa_in, side: str, max_piece_in: float = 4.0, piece_deg: float = 15.0):
        """Follow a tabulated curvature profile: kappa_in[i] (1/in, >= 0) at arc length
        s_in[i] (in), turning to `side` ('L'/'R', route frame).

        For turns whose curvature has to change gradually: the planner caps yaw
        acceleration (motion.max_angular_acceleration), so a straight butted onto an arc
        makes the profiler brake hard at the joint, which would put speed changes into
        a turn meant to be taken at steady speed. Nodes go every `piece_deg` of turn or
        `max_piece_in` of path, each pinning the heading and curvature of the profile;
        tangent magnitude is the piece length, as in `_turn`.
        """
        s_in = np.asarray(s_in, dtype=float)
        kappa_in = np.asarray(kappa_in, dtype=float)
        sign = 1.0 if side == "L" else -1.0
        theta = np.concatenate([[0.0], np.cumsum((kappa_in[1:] + kappa_in[:-1]) / 2 * np.diff(s_in))])
        h0 = math.atan2(self.heading[1], self.heading[0])
        hx, hy = np.cos(h0 + sign * theta), np.sin(h0 + sign * theta)
        x = self.pos[0] + np.concatenate([[0.0], np.cumsum((hx[1:] + hx[:-1]) / 2 * np.diff(s_in))])
        y = self.pos[1] + np.concatenate([[0.0], np.cumsum((hy[1:] + hy[:-1]) / 2 * np.diff(s_in))])
        picks, last = [], 0
        for i in range(1, len(s_in)):
            if s_in[i] - s_in[last] >= max_piece_in or theta[i] - theta[last] >= math.radians(piece_deg):
                picks.append(i)
                last = i
        end = len(s_in) - 1
        if not picks or picks[-1] != end:
            if picks and s_in[end] - s_in[picks[-1]] < 0.3 * max_piece_in:
                picks[-1] = end   # fold a sliver into the last piece
            else:
                picks.append(end)
        self.nodes[-1].curvature = self._curv_vec(sign * kappa_in[0] * FT)
        prev = 0
        for i in picks:
            mag = (s_in[i] - s_in[prev]) / FT
            self.nodes[-1].out_mag = mag
            self.pos = np.array([x[i], y[i]])
            self.heading = np.array([hx[i], hy[i]])
            self._emit(mag, None, curvature=self._curv_vec(sign * kappa_in[i] * FT))
            prev = i
        self.kappa = sign * kappa_in[-1] * FT
        return self

    def hold(self, seconds: float):
        """Dwell at the last node. On the start node the stop is forced, which the
        profiler requires before it will hold there."""
        node = self.nodes[-1]
        node.wait_time = seconds
        if node.start:
            node.stop_mode = 1
        return self

    def spin(self, route_deg: float, wait: float = 0.0):
        """Turn in place at the last node by `route_deg` (route frame: positive is
        clockwise for the robot, whose y is flipped), then dwell `wait` seconds."""
        node = self.nodes[-1]
        node.turn = route_deg
        node.wait_time = wait
        r = math.radians(route_deg)
        h = self.heading
        self.heading = np.array([h[0] * math.cos(r) - h[1] * math.sin(r), h[0] * math.sin(r) + h[1] * math.cos(r)])
        self.kappa = 0.0
        return self

    def action_point(self, t: float, actions: Dict[str, float]):
        self.action_points.append({"t": t, "actions": actions})
        return self

    def finish(self, actions=None):
        self.nodes[-1].end = True
        self.nodes[-1].out_mag = 0.0 if self.nodes[-1].tangent is not None else None
        if actions:
            self.nodes[-1].actions.update(actions)
        return self


def route_dict(builder: PathBuilder, action_names, start_reverse=False, max_velocity=0.0) -> dict:
    nodes = []
    for i, n in enumerate(builder.nodes):
        d: dict = {"x": round(n.x, 3), "y": round(n.y, 3)}
        if n.start:
            d["start"] = True
        if n.end:
            d["end"] = True
        if n.reverse or (n.start and start_reverse):
            d["reverse"] = True
        if n.tangent is not None:
            d["tangent"] = [round(float(n.tangent[0]), 4), round(float(n.tangent[1]), 4)]
            if n.in_mag is not None:
                d["incoming_magnitude"] = round(float(n.in_mag), 4)
            if n.out_mag is not None:
                d["outgoing_magnitude"] = round(float(n.out_mag), 4)
        if n.curvature is not None:
            d["tangent_derivative"] = [round(n.curvature[0], 4), round(n.curvature[1], 4)]
        if n.stop_mode:
            d["stop_mode"] = n.stop_mode
        if n.turn:
            d["turn"] = n.turn
        if n.wait_time:
            d["wait_time"] = n.wait_time
        if max_velocity or n.max_velocity:
            d["max_velocity"] = n.max_velocity or max_velocity
        if n.max_acceleration:
            d["max_acceleration"] = n.max_acceleration
        if n.actions:
            d["actions"] = n.actions
        nodes.append(d)
    aps = []
    for ap in builder.action_points:
        aps.append({"x": 0.0, "y": 0.0, "t": ap["t"], "actions": ap["actions"]})
    return {"format": "vap-route", "version": 2, "actions": list(action_names), "nodes": nodes, "action_points": aps}


def dumps_route(d: dict) -> str:
    lines = ['{', '  "format": "vap-route",', '  "version": 2,',
             f'  "actions": {json.dumps(d["actions"])},', '  "nodes": [']
    lines.append(",\n".join("    " + json.dumps(n, separators=(",", ":")) for n in d["nodes"]))
    lines.append('  ],')
    lines.append('  "action_points": [')
    if d["action_points"]:
        lines.append(",\n".join("    " + json.dumps(a, separators=(",", ":")) for a in d["action_points"]))
    lines.append('  ]')
    lines.append('}')
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------ odometry slip tests
#
# Where does odometry drift come from: slip along the direction of travel, or
# displacement that only happens while the robot turns as its speed changes?
# (2026-10-04 log analysis: the drift tracked rotation during speed changes and
# changed sign between braking and accelerating; straights barely drifted.) Each
# route isolates one condition and starts and ends at a hold where two wall-reset
# sensors squarely see two different walls, so wall truth can be compared with
# odometry before and after. They fire no distance resets.

#: Robot-frame wall face distance from the field centre (Config::DISTANCE_RESET_WALL_POSITION).
WALL_IN = 70.25
#: Wall-reset sensors: beam angle off the heading (deg, CCW), ALONG the beam from the
#: tracking centre to the face, ACROSS to the beam's right (in). Keep in step with the
#: *_DISTANCE_ALONG / *_DISTANCE_ACROSS constants in OverrideAutonomous/src/main.cpp.
#: Moved 1.75 in forward with the tracking centre on 2026-10-09 (single traction).
RESET_SENSORS = {"front": (0.0, 4.75, 5.75), "left": (90.0, 5.5, 2.75), "right": (270.0, 6.0, -3.75)}
#: A hold counts a sensor only if its beam is this close to the wall normal and the
#: reading is in this range (in): the V5 distance sensor degrades beyond ~1.2 m.
VIEW_MAX_OFF_DEG = 10.0
VIEW_RANGE_IN = (3.0, 50.0)

SLIP_YAW_RATE = 1.8 * 0.95       # rad/s, under motion.max_angular_velocity
SLIP_YAW_SLOPE = 5.0             # rad/s^2, under motion.max_angular_acceleration
SLIP_MIN_RADIUS_IN = 6.5         # tightest radius, just outside the half-track (5.7 in)
SLIP_TURN_SPEED_IN_S = 36.0      # entry speed of the braking turn, exit of the accelerating one
SLIP_TURN_ACCEL_FT_S2 = 2.25     # the braking / accelerating rate held through the turn
SLIP_HOLD_S = 0.6


def robot_xy(x: float, y: float):
    """Robot-frame field inches -> route frame (the planner's y points the other way)."""
    return (x, -y)


def braking_turn_profile(v0_in_s: float, decel_in_s2: float, total_rad: float = math.pi / 2):
    """Curvature vs arc length for a turn taken entirely while braking from v0 to rest.

    Built in time: speed falls linearly, the yaw rate ramps up at SLIP_YAW_SLOPE to a
    plateau (solved so the turn totals `total_rad`) and is held under v / R_min near
    the stop so the radius never closes past SLIP_MIN_RADIUS_IN. Curvature = yaw / v.
    Profiled with the same braking rate as a node cap, the planner reproduces this
    speed trace, so the rotation really happens while braking. Reversed, it is a turn
    taken while accelerating from rest.
    """
    def build(w_max):
        t = np.linspace(0.0, v0_in_s / decel_in_s2, 4000)
        v = v0_in_s - decel_in_s2 * t
        w = np.minimum(np.minimum(SLIP_YAW_SLOPE * t, w_max), v / SLIP_MIN_RADIUS_IN)
        s = np.concatenate([[0.0], np.cumsum((v[1:] + v[:-1]) / 2 * np.diff(t))])
        turned = np.sum((w[1:] + w[:-1]) / 2 * np.diff(t))
        kappa = np.where(v > 1e-6, w / np.maximum(v, 1e-6), 1.0 / SLIP_MIN_RADIUS_IN)
        return s, kappa, turned

    lo, hi = 0.1, SLIP_YAW_RATE
    if build(hi)[2] < total_rad:
        raise ValueError("braking turn cannot reach the angle: lower the rate or raise the speed")
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if build(mid)[2] < total_rad else (lo, mid)
    s, kappa, _ = build((lo + hi) / 2)
    return s, kappa


def steady_turn_profile(radius_in: float, ramp_in: float, total_rad: float = math.pi / 2):
    """Curvature ramps linearly 0 -> 1/R over ramp_in, holds, ramps back: a turn the
    planner can take at constant speed without braking for yaw acceleration."""
    k1 = 1.0 / radius_in
    hold_in = (total_rad - k1 * ramp_in) / k1
    s = np.linspace(0.0, 2 * ramp_in + hold_in, 4000)
    kappa = np.where(s < ramp_in, k1 * s / ramp_in,
                     np.where(s < ramp_in + hold_in, k1, k1 * (2 * ramp_in + hold_in - s) / ramp_in))
    return s, kappa


def sensor_views(pose):
    """For a robot-frame pose (in, in, rad): {sensor: (wall, axis, reading_in, hit_x, hit_y)}
    for each wall-reset sensor squarely facing a wall within range."""
    x, y, th = pose
    out = {}
    for name, (angle, along, across) in RESET_SENSORS.items():
        beam = th + math.radians(angle)
        bx, by = math.cos(beam), math.sin(beam)
        rx, ry = math.cos(beam - math.pi / 2), math.sin(beam - math.pi / 2)
        fx, fy = x + along * bx + across * rx, y + along * by + across * ry
        for wall, (nx, ny) in {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}.items():
            cos_off = bx * nx + by * ny
            if cos_off < math.cos(math.radians(VIEW_MAX_OFF_DEG)):
                continue
            face = fx if nx else fy
            reading = (WALL_IN - face * (nx or ny)) / cos_off
            if VIEW_RANGE_IN[0] <= reading <= VIEW_RANGE_IN[1]:
                out[name] = (wall, "x" if nx else "y", reading, fx + bx * reading, fy + by * reading)
    return out


def holds_of(profiled, min_s: float = 0.3):
    """[(t0, t1, robot-frame pose)] for every stretch the profile sits still."""
    t = np.asarray(profiled.times)
    still = (np.abs(np.asarray(profiled.linear_vels)) < 1e-9) & (np.abs(np.asarray(profiled.angular_vels)) < 1e-9)
    out, i = [], 0
    while i < len(t):
        if still[i]:
            j = i
            while j + 1 < len(t) and still[j + 1]:
                j += 1
            if t[j] - t[i] >= min_s:
                c = profiled.coords[i]
                out.append((float(t[i]), float(t[j]), (float(c[0]) * FT, -float(c[1]) * FT, float(profiled.headings[i]))))
            i = j + 1
        else:
            i += 1
    return out


def rotation_by_accel(profiled):
    """Share of the rotation made while moving that happens braking (< -15 in/s^2),
    accelerating (> +15) and at steady speed (|a| < 8), plus in-place rotation (deg)."""
    t = np.asarray(profiled.times)
    v = np.asarray(profiled.linear_vels) * FT
    w = np.asarray(profiled.angular_vels)
    a = np.gradient(v, t) * np.sign(v)
    turned = np.abs(w) * np.gradient(t)
    moving = np.abs(v) > 0.5
    total = turned[moving].sum()
    share = lambda m: float(turned[moving & m].sum() / total) if total > 0 else 0.0
    return {"braking": share(a < -15), "accelerating": share(a > 15), "steady": share(np.abs(a) < 8),
            "moving_deg": float(np.degrees(total)), "in_place_deg": float(np.degrees(turned[~moving].sum()))}


def slip_routes(action_names) -> Dict[str, dict]:
    """Five routes in the robot-frame (-x, -y) quarter, all starting facing -x.

    Robot-left (CCW) is route 'R', and an in-place robot-left turn is a negative route
    `turn`. Positions are robot frame. Every -x wall beam lands at y >= -47 in to stay
    clear of the matchloader the skills logs place on that wall near y = -56.
    """
    turn_rate = SLIP_TURN_ACCEL_FT_S2 * FT
    s_b, k_b = braking_turn_profile(SLIP_TURN_SPEED_IN_S, turn_rate)
    s_a, k_a = (s_b[-1] - s_b[::-1]), k_b[::-1]
    s_s, k_s = steady_turn_profile(18.0, 12.0)
    routes = {}

    # Straight control: speed changes with no turn, left sensor alongside the -y wall.
    b = PathBuilder(start=robot_xy(-20, -36)).hold(SLIP_HOLD_S).straight(26).finish()
    b.hold(SLIP_HOLD_S + 0.2)
    routes["slip_straight"] = route_dict(b, action_names)

    # In-place 90 deg turns, alternating left / right, 4 in hops between holds.
    b = PathBuilder(start=robot_xy(-36, -30)).hold(SLIP_HOLD_S)
    for k in range(5):
        b.straight(4).spin(-90.0 if k % 2 == 0 else 90.0, SLIP_HOLD_S)
    b.straight(4).finish().hold(SLIP_HOLD_S + 0.2)
    routes["slip_spin"] = route_dict(b, action_names)

    # 90 deg left at constant speed: curvature ramps in and out under the yaw-accel cap.
    b = PathBuilder(start=robot_xy(-18, -15.5)).hold(SLIP_HOLD_S).straight(12)
    b.curvature_profile(s_s, k_s, "R").straight(5).finish().hold(SLIP_HOLD_S + 0.2)
    d = route_dict(b, action_names, max_velocity=round(26.0 / FT, 4))
    routes["slip_turn_steady"] = d

    # 90 deg left taken entirely while braking to rest at SLIP_TURN_ACCEL_FT_S2.
    b = PathBuilder(start=robot_xy(-17.5, -27)).hold(SLIP_HOLD_S)
    b.nodes[0].max_velocity = round(SLIP_TURN_SPEED_IN_S / FT, 4)
    b.straight(16)
    b.nodes[-1].max_acceleration = SLIP_TURN_ACCEL_FT_S2
    b.curvature_profile(s_b, k_b, "R").finish().hold(SLIP_HOLD_S + 0.2)
    routes["slip_turn_brake"] = route_dict(b, action_names)

    # The same turn mirrored in time: from rest, accelerating at the same rate.
    b = PathBuilder(start=robot_xy(-30, -15)).hold(SLIP_HOLD_S)
    b.nodes[0].max_acceleration = SLIP_TURN_ACCEL_FT_S2
    b.curvature_profile(s_a, k_a, "R")
    b.nodes[-1].max_acceleration = 12.0   # brake on the run-out at the normal limit
    b.straight(8).finish().hold(SLIP_HOLD_S + 0.2)
    routes["slip_turn_accel"] = route_dict(b, action_names)
    return routes


# --------------------------------------------------------------------- scenarios

def chicane_route(action_names, mid_deg: float, magnitude: float,
                  length_in: float, lateral_in: float) -> dict:
    """A three-node S-bend: start, the inflection, end.

    Not built from two butted arcs. That pinned +1/ft and -1/ft at either end of a
    short straight blend, which no spline can honour on a straight chord: the
    exported trajectory rang at 3.3 Hz with three yaw-rate zero crossings where a
    clean inflection has one, and the robot shuddered through it. An inflection is
    the one place on a path where curvature must pass through zero, so it is the
    one place not to over-constrain. Three nodes carry the entry and exit headings
    and the bend angle; the spline picks the rest, and the yaw rate crosses zero
    once. Geometry is tuned so the tightest part still comes out at R12 (1.01 ft
    measured), which is what the name means.
    """
    start = START
    end = (start[0] - length_in, start[1] - lateral_in)
    mid = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
    r = math.radians(180.0 + mid_deg)
    builder = PathBuilder.__new__(PathBuilder)
    builder.action_points = []
    builder.nodes = [
        NodeSpec(start[0], start[1], tangent=[-1.0, 0.0], in_mag=0.0, out_mag=magnitude,
                 curvature=[0.0, 0.0], start=True),
        NodeSpec(mid[0], mid[1], tangent=[math.cos(r), math.sin(r)],
                 in_mag=magnitude, out_mag=magnitude, curvature=[0.0, 0.0]),
        NodeSpec(end[0], end[1], tangent=[-1.0, 0.0], in_mag=magnitude, out_mag=0.0,
                 curvature=[0.0, 0.0], end=True),
    ]
    return route_dict(builder, action_names)


def build_suite(cfg) -> Dict[str, dict]:
    names = cfg["robot"]["actions"]
    routes: Dict[str, dict] = {}
    manifest: Dict[str, dict] = {}

    def add(name, d, scenario, exercises, metric, **band):
        routes[name] = d
        manifest[name] = {"scenario": scenario, "exercises": exercises, "watch": metric, **band}

    # 1. Straights ------------------------------------------------------------
    add("hop_30in", route_dict(PathBuilder().straight(30).finish(), names),
        "the typical 2.6 ft segment: a triangle profile that never reaches cruise",
        "accel/brake feedforward, stop accuracy at a low peak speed",
        "final position error, creep at stop", straight_in=30)
    add("cruise_8ft", route_dict(PathBuilder(start=(48.0, 24.0)).straight(96).finish(), names),
        "8 ft (4 tile) straight at max_velocity, x=+48 to x=-48",
        "top-end velocity tracking, usable-speed headroom",
        "saturation fraction, straight wheel gain", straight_in=96)

    # 2. Constant-radius 90 deg arcs from a standing start ----------------------
    for r_in in (6, 12, 24):
        add(f"arc_R{r_in}", route_dict(PathBuilder().arc(r_in, 90, "L", next_kappa=None).finish(), names),
            f"90 deg arc of {r_in} in radius from rest",
            "steady yaw at this radius band, effective track width, inner-wheel hold",
            "heading p95, commanded rotation delivered", radius_ft=r_in / FT, arc_deg=90)

    # 3. Corners entered at speed ----------------------------------------------
    for r_in in (6, 12):
        add(f"corner_R{r_in}", route_dict(PathBuilder().straight(24).arc(r_in, 90, "L").straight(24).finish(), names),
            f"24 in straight, {r_in} in radius 90 deg corner, 24 in straight",
            "corner entered at speed: ramp-in lag, outer-wheel saturation, RAMSETE catch-up",
            "saturation run length, peak heading lag, cross-track on exit", radius_ft=r_in / FT, arc_deg=90)

    # 4. Chicane: three nodes, the middle one on the inflection -----------------
    add("chicane_R12", chicane_route(names, mid_deg=45.0, magnitude=1.2,
                                     length_in=36.0, lateral_in=17.6),
        "S-bend to a 17.6 in lateral offset over 36 in, inflection at 45 deg",
        "curvature sign flip at the inflection: yaw acceleration, angular feedforward sign",
        "yaw-accel tracking, cross-track not one-sided",
        radius_ft=1.0, inflection_deg=45, holds_radius=False)

    # 5. Reverse cusp: 3 ft out, reverse, 3 ft back along a 60 deg V -------------
    add("vee_reverse", route_dict(
        PathBuilder().straight(36).reverse_here().turn_to(-60).straight(36).finish(), names),
        "36 in forward, reverse cusp, 36 in backwards along a leg 60 deg off the first",
        "the most common real event: stop at the cusp, heading through the reversal, backward RAMSETE",
        "pose error at the cusp, unintended reverse test, final error", straight_in=36)

    # 6. Arc driven backwards ---------------------------------------------------
    add("reverse_arc_R12", route_dict(PathBuilder().arc(12, 90, "L", next_kappa=None).finish(), names, start_reverse=True),
        "the 12 in arc driven in reverse from rest",
        "negative v through RAMSETE and the drive controller, kS sign, observer in reverse",
        "heading p95 versus arc_R12, velocity tracking", radius_ft=1.0, arc_deg=90)

    # 7. Crawl through a cusp-tight radius --------------------------------------
    add("crawl_R3", route_dict(PathBuilder().arc(3, 180, "L", next_kappa=None).finish(), names, max_velocity=round(4 / FT, 4)),
        "180 deg arc of 3 in radius at 4 in/s (per-node max_velocity)",
        "stiction and encoder quantisation at the speeds the real cusps are taken",
        "low-speed velocity tracking, creep", radius_ft=0.25, arc_deg=180, max_velocity_in_s=4)

    # 8. Lift and intake while driving ------------------------------------------
    add("lift_while_driving", route_dict(
        PathBuilder().straight(30).action_point(0.5, {"intake": 1, "lift_state": 1}).finish(), names),
        "hop_30in with intake on and a lift score fired halfway",
        "lift and intake load during a move, intake inhibit while the lift is busy",
        "tracking delta versus hop_30in", straight_in=30)

    # 9. Wall reset ---------------------------------------------------------------
    add("wall_reset", route_dict(
        PathBuilder().straight(52).reverse_here(actions={"front_reset": 1}).straight(52).finish(), names),
        "52 in toward the left wall, front distance reset at the cusp, 52 in back",
        "DistanceResetOdometry correction size and whether it helps the return leg",
        "reset correction magnitude, pose-jump test, final error", straight_in=52)

    # 10. Chain of four hops with reverses ----------------------------------------
    add("chain_4seg", route_dict(
        PathBuilder().straight(30).reverse_here().turn_to(-45).straight(30)
        .reverse_here().turn_to(180).straight(30).reverse_here().turn_to(45).straight(30).finish(), names),
        "four 30 in hops joined by reverse cusps: a miniature match auton",
        "error accumulation across segments, cusp handling back to back",
        "final error growth per segment", straight_in=30, segments=4)

    # 11. Odometry slip tests -------------------------------------------------------
    slip = slip_routes(names)
    slip_specs = {
        "slip_straight": ("26 in straight toward the -x wall, full acceleration and braking, no turn",
                          "along-track wheel slip on its own: the control for the turn tests",
                          "x drift (front) vs the 26 in odometry says; y drift (left) should stay ~0",
                          {"straight_in": 26}),
        "slip_spin": ("five 90 deg in-place turns, alternating left/right, 4 in hops, a hold after each",
                      "displacement from turning with no speed change: is the pivot where odometry assumes?",
                      "x/y drift per spin, split by spin direction", {}),
        "slip_turn_steady": ("12 in run-up, 90 deg left at a constant 26 in/s (R18, 12 in curvature ramps), 5 in run-out",
                             "rotation at constant speed",
                             "x/y drift at the end hold", {}),
        "slip_turn_brake": ("16 in run-up to 36 in/s, then a 90 deg left taken entirely while braking to rest",
                            "rotation while braking (the skills pickup turn)",
                            "x/y drift at the end hold; compare with slip_turn_steady and slip_turn_accel", {}),
        "slip_turn_accel": ("the braking turn mirrored in time: from rest, 90 deg left while accelerating, 8 in run-out",
                            "rotation while accelerating (the swing out of the loader)",
                            "x/y drift at the end hold; sign should oppose slip_turn_brake if the effect is real", {}),
    }
    for name, d in slip.items():
        scenario, exercises, metric, band = slip_specs[name]
        profiled = profile_dict(d, cfg)
        holds = []
        for t0, t1, pose in holds_of(profiled):
            views = sensor_views(pose)
            holds.append({
                "t": [round(t0, 2), round(t1, 2)],
                "pose": [round(pose[0], 1), round(pose[1], 1), round(math.degrees(pose[2]), 1)],
                "views": {s: f"{v[0]} wall, {v[2]:.1f} in, beam hits ({v[3]:.1f}, {v[4]:.1f})" for s, v in views.items()},
            })
        shares = rotation_by_accel(profiled)
        add(name, d, scenario, exercises, metric, **band, slip_test=True,
            rotation_share={k: round(v, 2) for k, v in shares.items()}, holds=holds)

    # 12. Spin-rate sweep ---------------------------------------------------------------
    # Turn cost (scrub) against yaw rate, the tracking wheels' offsets from the pivot, and
    # the top spin rate: the double vs single traction A/B (OverrideAutonomous
    # metrics.spin_scrub / tracking_wheel_offsets). Each turn node's max_velocity /
    # max_acceleration override sets its spin (motion_profile_angle).
    half_track_ft = cfg["robot"]["track_width"] / 2 / FT
    for name, (rate, accel) in SPIN_RATES.items():
        b = PathBuilder()
        b.straight(SPIN_HOP_IN).spin(-SPIN_DEG, SPIN_HOLD_S)  # route frame: negative is CCW for the robot
        b.nodes[-1].max_velocity = round(rate * half_track_ft, 4)
        b.straight(SPIN_HOP_IN).spin(SPIN_DEG, SPIN_HOLD_S)
        b.nodes[-1].max_velocity = round(rate * half_track_ft, 4)
        if accel:
            # End node only: on the first spin the override would also bind the hop after
            # it, so its spin keeps the config's max_turn_acceleration.
            b.nodes[-1].max_acceleration = round(accel * half_track_ft, 4)
        add(name, route_dict(b.finish(), names),
            f"{SPIN_DEG:.0f} deg spin CCW, hold, {SPIN_HOP_IN:.0f} in hop, {SPIN_DEG:.0f} deg CW, hold, "
            + (f"at {rate:g} rad/s" if rate < 8 else "planned at 8 rad/s so the robot runs flat out"),
            "turn cost against yaw rate (scrub), effective track width, tracking-wheel offsets"
            + ("; top spin rate and spin acceleration" if rate >= 8 else ""),
            "spin_scrub fit and tracking_wheel_offsets; compare double vs single traction",
            spin_rate=rate, spin_deg=SPIN_DEG, **({"exceeds_caps": True} if rate >= 8 else {}))

    return routes, manifest


#: Spin-rate sweep: route -> (planned rate rad/s, planned acceleration rad/s^2 or None
#: for the config's max_turn_acceleration; applied to the final CW spin only). spin_max
#: is planned beyond what the robot can do so it saturates and the log shows the real
#: top rate and acceleration.
SPIN_RATES = {"spin_w1": (1.0, None), "spin_w2": (2.0, None), "spin_w3": (3.0, None),
              "spin_w4": (4.0, None), "spin_max": (8.0, 14.0)}
SPIN_DEG = 720.0
SPIN_HOLD_S = 1.0
SPIN_HOP_IN = 8.0


def profile_dict(d: dict, cfg) -> "hp.ProfiledRoute":
    """Profile a route dict under the config, as profile_routes.py would."""
    route = route_serializer.parse_route(dumps_route(d), cfg["robot"]["actions"])
    nodes, aps = hp.nodes_from_route(route)
    manager = hp.build_spline_manager(nodes, aps)
    constraints = hp.constraints_from_config(cfg["motion"], cfg["robot"]["track_width"])
    return hp.profile_route(manager, constraints)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--install", action="store_true", help="copy the routes into the config's routes folder")
    args = parser.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())

    routes, manifest = build_suite(cfg)
    ROUTES_DIR.mkdir(parents=True, exist_ok=True)
    for name, d in routes.items():
        (ROUTES_DIR / f"{name}.txt").write_text(dumps_route(d))
    header = {
        "generated_by": "tools/make_suite.py",
        "start_pose_robot_frame": "(0.069, -23.664, pi) for every route except cruise_8ft, which starts at (48, -24, pi), and the slip_* routes, whose start is the pose of their first hold",
        "log_naming": "import the robot log and copy it to OverrideAutonomous/tests/logs/suite_<route>-<n>.txt",
        "routes": manifest,
    }
    (SUITE_DIR / "manifest.yaml").write_text(yaml.safe_dump(header, sort_keys=False))
    print(f"wrote {len(routes)} routes to {ROUTES_DIR} and {SUITE_DIR / 'manifest.yaml'}")

    if args.install:
        dest = Path(cfg["files"]["routes_folder"]) / "suite"
        dest.mkdir(parents=True, exist_ok=True)
        for name in routes:
            shutil.copy(ROUTES_DIR / f"{name}.txt", dest / f"{name}.txt")
        print(f"installed into {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
