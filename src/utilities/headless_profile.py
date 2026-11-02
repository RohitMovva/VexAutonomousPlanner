"""Drive the motion profiler from a route file without Qt.

The GUI feeds the profiler ``Node`` / ``ActionPoint`` graphics items, but the
spline manager and profiler only read plain attributes off them. These stand-ins
carry exactly those attributes, so a saved route can be profiled from a script or
a test: ``load_route_file`` -> ``build_spline_manager`` -> ``profile_route``.

Units follow the app: route files store inches, the profiler works in feet
(``feet = inches / 12``, the same conversion ``load_nodes`` -> ``convert_point``
-> ``update_spline`` nets out to).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from motion_profiling_v2 import motion_profile_generator as mpg
from splines.spline_manager import QuinticHermiteSplineManager
from utilities import action_spec, route_serializer

INCHES_PER_FOOT = 12.0

# Mirrors gui.node's stop modes without importing Qt.
STOP_AUTO, STOP_FORCED, STOP_DISABLED = 0, 1, 2


@dataclass
class HeadlessNode:
    """The attributes the spline manager and profiler read off a ``gui.node.Node``."""

    x: float  # feet
    y: float  # feet
    is_start_node: bool = False
    is_end_node: bool = False
    is_reverse_node: bool = False
    stop_mode: int = STOP_AUTO
    turn: float = 0.0
    # Overlap the turn's spin with the ramps either side (mpg.blends_turn).
    spin_through: bool = False
    wait_time: float = 0.0
    max_velocity: float = 0.0
    max_acceleration: float = 0.0
    tangent: Optional[np.ndarray] = None
    # gui.node.Node stores the tangent derivative under this name.
    curvature: Optional[np.ndarray] = None
    incoming_magnitude: Optional[float] = None
    outgoing_magnitude: Optional[float] = None
    actions: Dict[str, float] = field(default_factory=dict)

    def auto_stop(self) -> bool:
        return (
            self.is_reverse_node
            or self.turn != 0
            or self.is_end_node
            or action_spec.waits_for_lift(self.actions)
        )

    @property
    def stop(self) -> bool:
        if self.stop_mode == STOP_FORCED:
            return True
        if self.stop_mode == STOP_DISABLED:
            return False
        return self.auto_stop()


@dataclass
class HeadlessActionPoint:
    x: float  # feet
    y: float  # feet
    t: float
    stop: bool = False
    wait_time: float = 0.0
    max_velocity: float = 0.0
    max_acceleration: float = 0.0
    actions: Dict[str, float] = field(default_factory=dict)


def nodes_from_route(route: route_serializer.Route):
    nodes = []
    for rec in route.nodes:
        nodes.append(
            HeadlessNode(
                x=rec.x / INCHES_PER_FOOT,
                y=rec.y / INCHES_PER_FOOT,
                is_start_node=rec.is_start,
                is_end_node=rec.is_end,
                is_reverse_node=rec.is_reverse,
                stop_mode=rec.stop_mode,
                turn=rec.turn,
                spin_through=rec.spin_through,
                wait_time=rec.wait_time,
                max_velocity=rec.max_velocity,
                max_acceleration=rec.max_acceleration,
                tangent=None if rec.tangent is None else np.array(rec.tangent, dtype=float),
                curvature=(
                    None
                    if rec.tangent_derivative is None
                    else np.array(rec.tangent_derivative, dtype=float)
                ),
                incoming_magnitude=rec.incoming_magnitude,
                outgoing_magnitude=rec.outgoing_magnitude,
                actions=dict(rec.actions),
            )
        )
    action_points = [
        HeadlessActionPoint(
            x=rec.x / INCHES_PER_FOOT,
            y=rec.y / INCHES_PER_FOOT,
            t=rec.t,
            stop=rec.stop,
            wait_time=rec.wait_time,
            max_velocity=rec.max_velocity,
            max_acceleration=rec.max_acceleration,
            actions=dict(rec.actions),
        )
        for rec in route.action_points
    ]
    return nodes, action_points


def build_spline_manager(
    nodes: Sequence[HeadlessNode], action_points: Sequence[HeadlessActionPoint] = ()
) -> QuinticHermiteSplineManager:
    manager = QuinticHermiteSplineManager()
    points = np.array([[n.x, n.y] for n in nodes], dtype=float)
    if not manager.build_path(points, list(nodes), list(action_points)):
        raise ValueError("spline manager rejected the path (need >= 2 nodes)")
    return manager


def load_route_file(path: Path, action_names: Sequence[str] = ()) -> route_serializer.Route:
    return route_serializer.parse_route(Path(path).read_text(), action_names)


def constraints_from_config(motion: dict, track_width_in: float) -> mpg.Constraints:
    """Build Constraints the way ``PathWidget.generate_motion_profile_lists`` does,
    from the ``motion:`` section of config.yaml and the robot track width in inches."""
    max_acc = motion["max_acceleration"]
    return mpg.Constraints(
        max_vel=motion["max_velocity"],
        max_acc=max_acc,
        max_dec=motion.get("max_deceleration") or max_acc,
        friction_coef=0.8,
        max_jerk=motion.get("max_jerk", 0.0),
        track_width=track_width_in / INCHES_PER_FOOT,
        stall_accel=motion.get("stall_accel"),
        free_speed=motion.get("free_speed"),
        traction_accel=motion.get("traction_accel"),
        steering_speed_penalty=motion.get("steering_speed_penalty"),
        max_angular_velocity=motion.get("max_angular_velocity"),
        max_angular_acceleration=motion.get("max_angular_acceleration"),
        max_angular_velocity_left_outer=motion.get("max_angular_velocity_left_outer"),
        max_angular_velocity_right_outer=motion.get("max_angular_velocity_right_outer"),
        max_turn_velocity=motion.get("max_turn_velocity"),
        max_turn_acceleration=motion.get("max_turn_acceleration"),
        spin_through_velocity=motion.get("spin_through_velocity"),
        spin_through_acceleration=motion.get("spin_through_acceleration"),
    )


@dataclass
class ProfiledRoute:
    times: List[float]
    positions: List[float]
    linear_vels: List[float]
    accelerations: List[float]
    headings: List[float]
    angular_vels: List[float]
    nodes_map: List[int]
    actions_map: List[int]
    coords: List
    report: mpg.FeasibilityReport


def profile_route(
    manager: QuinticHermiteSplineManager, constraints: mpg.Constraints
) -> ProfiledRoute:
    result = mpg.generate_motion_profile(manager, constraints)
    times, _, linear_vels, _, _, angular_vels = result[:6]
    report = mpg.check_trajectory_feasibility(
        times, linear_vels, angular_vels, constraints,
        spin_through=mpg.spin_through_samples(manager.nodes, result[6], len(times)),
    )
    return ProfiledRoute(*result, report)


def export_trajectory(
    profiled: ProfiledRoute,
    nodes: Sequence[HeadlessNode],
    action_points: Sequence[HeadlessActionPoint],
    action_names: Sequence[str],
) -> str:
    """Render a profiled route in the firmware's trajectory file format.

    Mirrors ``AutonomousPlannerGUIManager.save_nodes_to_file`` line for line so a
    headless export is byte-compatible with a GUI one: sample rows are
    ``0 t x y theta v w`` with inches, the canvas y flipped and v in in/s; action
    rows are ``1`` followed by one value per configured action, in config order.
    Node rows are inserted at ``nodes_map[i] + i`` first and action-point rows at
    ``actions_map[i] + i`` afterwards, exactly as the GUI does (the second pass
    does not account for the rows the first inserted, so an action point can land a
    few samples early; kept for parity).
    """
    rows: List[list] = [
        [
            0,
            t,
            coord[0] * INCHES_PER_FOOT,
            coord[1] * -INCHES_PER_FOOT,
            heading,
            v * INCHES_PER_FOOT,
            w,
        ]
        for t, coord, heading, v, w in zip(
            profiled.times,
            profiled.coords,
            profiled.headings,
            profiled.linear_vels,
            profiled.angular_vels,
        )
    ]

    def values(actions: Dict[str, float]) -> list:
        return [1] + [actions.get(name, 0) for name in action_names]

    node_rows = [values(n.actions) for n in nodes]
    action_rows = [values(ap.actions) for ap in action_points]
    for i, sample in enumerate(profiled.nodes_map):
        rows.insert(int(sample) + i, node_rows[i])
    for i, sample in enumerate(profiled.actions_map):
        rows.insert(int(sample) + i, action_rows[i])

    return "".join("".join(f"{v} " for v in row) + "\n" for row in rows)
