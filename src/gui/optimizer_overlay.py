"""Canvas drawing for the path optimizer: what each node may change, the route's
keep-out zones, and the last run's result.

Limits:
- a band around the path as wide as each node's corridor (interpolated between nodes,
  as the optimizer's guard does),
- a wedge at every node whose heading may turn, as wide as the turn allowed,
- a ring around every node: green where heading, handles and curvature are all free,
  amber where some are, red where none are,
- a dashed circle around every node that may move, as far as it may move.

Keep-out zones: filled red, the one being edited outlined.

Result: the optimized path dashed over the current one, with rings on the nodes it
changed.

Everything is computed from route text through the headless spline builder, the same
route the optimizer is given, so what is drawn is what it will search.
"""

from __future__ import annotations

import logging
import math
from typing import Iterable, Optional

import numpy as np
from PyQt6.QtCore import QPointF, Qt
from PyQt6.QtGui import QBrush, QColor, QPainterPath, QPen, QPolygonF
from PyQt6.QtWidgets import QGraphicsPathItem, QGraphicsScene

from motion_profiling_v2 import motion_profile_generator as mpg
from utilities import headless_profile as hp
from utilities import path_optimizer as po
from utilities import route_serializer as rs
from utilities.field_scale import FIELD_IMAGE_FT, FIELD_IMAGE_PX

logger = logging.getLogger(__name__)

# Field feet -> canvas pixels, as PathWidget.update_spline converts.
FIELD_FT = FIELD_IMAGE_FT
CANVAS_PX = FIELD_IMAGE_PX

CORRIDOR_COLOR = QColor(74, 163, 223, 45)
WEDGE_COLOR = QColor(230, 162, 60, 70)
FREE_COLOR = QColor("#3cb371")
PARTLY_COLOR = QColor("#e6a23c")
LOCKED_COLOR = QColor("#e64c3c")
CANDIDATE_COLOR = QColor("#ff66cc")
POSITION_COLOR = QColor("#4aa3df")
KEEP_OUT_COLOR = QColor(230, 76, 60, 70)
KEEP_OUT_EDGE = QColor("#e64c3c")

SAMPLES_PER_SEGMENT = 40
WEDGE_RADIUS_PX = 110.0
RING_RADIUS_PX = 22.0


def ft_to_px(points) -> np.ndarray:
    return (np.asarray(points, float) / FIELD_FT + 0.5) * CANVAS_PX


def _polygon(points_px) -> QPolygonF:
    return QPolygonF([QPointF(float(x), float(y)) for x, y in points_px])


def _ring(path: QPainterPath, center, radius: float) -> None:
    path.addEllipse(QPointF(float(center[0]), float(center[1])), radius, radius)


def in_to_px(inches) -> np.ndarray:
    """Field inches (route coordinates) -> canvas pixels."""
    return ft_to_px(np.asarray(inches, float) / hp.INCHES_PER_FOOT)


def inches_px(length_in: float) -> float:
    """A length in inches as canvas pixels."""
    return length_in / hp.INCHES_PER_FOOT / FIELD_FT * CANVAS_PX


def keep_out_path(zones) -> QPainterPath:
    path = QPainterPath()
    path.setFillRule(Qt.FillRule.WindingFill)
    for zone in zones:
        if zone.shape == "circle":
            x, y, r = zone.values
            _ring(path, in_to_px([x, y]), inches_px(r))
        else:
            x0, y0, x1, y1 = zone.values
            (px0, py0), (px1, py1) = in_to_px([x0, y0]), in_to_px([x1, y1])
            path.addRect(float(min(px0, px1)), float(min(py0, py1)), float(abs(px1 - px0)), float(abs(py1 - py0)))
    return path


class OptimizerOverlay:
    """Owns the overlay items on the path canvas; hidden until shown."""

    def __init__(self, scene: QGraphicsScene):
        # Created before any node, so at the default z every overlay draws above the
        # field and path but under the nodes, in creation order.
        def item(pen=None, brush=None):
            it = QGraphicsPathItem()
            it.setPen(pen if pen is not None else QPen(Qt.PenStyle.NoPen))
            if brush is not None:
                it.setBrush(brush)
            scene.addItem(it)
            return it

        self.corridor_item = item(brush=QBrush(CORRIDOR_COLOR))
        self.wedge_item = item(brush=QBrush(WEDGE_COLOR))
        self.ring_items = {
            color.name(): item(pen=QPen(color, 3))
            for color in (FREE_COLOR, PARTLY_COLOR, LOCKED_COLOR)
        }
        self.position_item = item(pen=QPen(POSITION_COLOR, 3, Qt.PenStyle.DashLine))
        self.keep_out_item = item(pen=QPen(KEEP_OUT_EDGE, 2), brush=QBrush(KEEP_OUT_COLOR))
        self.keep_out_selected_item = item(pen=QPen(KEEP_OUT_EDGE, 5))
        dashed = QPen(CANDIDATE_COLOR, 3, Qt.PenStyle.DashLine)
        self.candidate_item = item(pen=dashed)
        self.candidate_nodes_item = item(pen=QPen(CANDIDATE_COLOR, 3))

    # --- limits -----------------------------------------------------------------

    def clear_limits(self) -> None:
        for it in (self.corridor_item, self.wedge_item, self.position_item, *self.ring_items.values()):
            it.setPath(QPainterPath())

    def show_limits(self, route: rs.Route, options: po.OptimizerOptions) -> None:
        """Draw what each node of the route may change under these run options."""
        self.clear_limits()
        try:
            nodes, action_points = hp.nodes_from_route(route)
            manager = hp.build_spline_manager(nodes, action_points)
        except Exception:
            return  # no buildable path yet (no start/end, one node, ...)
        limits = [po.node_limits(rec, options) for rec in route.nodes]
        self.corridor_item.setPath(self._corridor_path(manager, limits))

        sides = po.knot_sides(manager)
        wedges = QPainterPath()
        wedges.setFillRule(Qt.FillRule.WindingFill)
        rings = {name: QPainterPath() for name in self.ring_items}
        reach = QPainterPath()
        for i, (node, lim) in enumerate(zip(nodes, limits)):
            center = ft_to_px([node.x, node.y])
            if lim.position_in > 0 and po.POSITION in options.settings:
                _ring(reach, center, inches_px(lim.position_in))
            free = {k: v and k in options.settings for k, v in lim.free().items()}
            count = sum(free.values())
            color = FREE_COLOR if count == len(free) else LOCKED_COLOR if count == 0 else PARTLY_COLOR
            _ring(rings[color.name()], center, RING_RADIUS_PX)
            if free[po.HEADING]:
                vec = sides[i].out_vec if sides[i].out_vec is not None else sides[i].in_vec
                if vec is None or np.linalg.norm(vec) < 1e-12:
                    continue
                # Canvas and field axes scale by the same positive factor, so the
                # field angle is the canvas angle.
                angle = math.atan2(vec[1], vec[0])
                spread = math.radians(lim.heading_deg)
                arc = np.linspace(angle - spread, angle + spread, 24)
                pts = [center] + [
                    center + WEDGE_RADIUS_PX * np.array([math.cos(a), math.sin(a)]) for a in arc
                ]
                wedges.addPolygon(_polygon(pts))
                wedges.closeSubpath()
        self.wedge_item.setPath(wedges)
        self.position_item.setPath(reach)
        for name, path in rings.items():
            self.ring_items[name].setPath(path)

    @staticmethod
    def _corridor_path(manager, limits) -> QPainterPath:
        """One filled strip per segment, offset along the normal by the corridor."""
        path = QPainterPath()
        path.setFillRule(Qt.FillRule.WindingFill)
        corridor_ft = np.array([lim.corridor_in for lim in limits]) / hp.INCHES_PER_FOOT
        for j in range(len(limits) - 1):
            u = np.linspace(j, j + 1, SAMPLES_PER_SEGMENT + 1)
            pts = np.asarray(manager.get_points_at_parameters(u), float)
            d = np.asarray(manager.get_derivatives_at_parameters(u), float)
            norm = np.linalg.norm(d, axis=1)
            ok = norm > 1e-9
            if ok.sum() < 2:
                continue
            pts, d, norm, u = pts[ok], d[ok], norm[ok], u[ok]
            normal = np.column_stack([-d[:, 1], d[:, 0]]) / norm[:, None]
            width = np.interp(u, [j, j + 1], corridor_ft[j : j + 2])[:, None]
            left, right = pts + normal * width, pts - normal * width
            path.addPolygon(_polygon(ft_to_px(np.vstack([left, right[::-1]]))))
            path.closeSubpath()
        return path

    # --- keep-out zones ----------------------------------------------------------

    def show_keep_out(self, zones, selected: Optional[int] = None) -> None:
        self.keep_out_item.setPath(keep_out_path(zones))
        chosen = [zones[selected]] if selected is not None and 0 <= selected < len(zones) else []
        self.keep_out_selected_item.setPath(keep_out_path(chosen))

    def clear_keep_out(self) -> None:
        self.keep_out_item.setPath(QPainterPath())
        self.keep_out_selected_item.setPath(QPainterPath())

    # --- result -----------------------------------------------------------------

    def clear_candidate(self) -> None:
        self.candidate_item.setPath(QPainterPath())
        self.candidate_nodes_item.setPath(QPainterPath())

    def show_candidate(self, route: rs.Route, changed: Iterable[int] = (), constraints=None) -> None:
        """Draw a result's path. constraints (the run's) let a route with spin-through
        turns be drawn where the profile sends the robot rather than along the spline."""
        self.clear_candidate()
        try:
            nodes, action_points = hp.nodes_from_route(route)
            manager = hp.build_spline_manager(nodes, action_points)
        except Exception:
            logger.exception("Could not build the candidate path for the overlay")
            return
        u = np.linspace(0, len(nodes) - 1, SAMPLES_PER_SEGMENT * (len(nodes) - 1) + 1)
        pts = ft_to_px(manager.get_points_at_parameters(u))
        if any(mpg.blends_turn(n, i, len(nodes)) for i, n in enumerate(nodes)):
            # A spin-through turn swings the robot off the spline: show where it goes.
            try:
                coords = hp.profile_route(manager, constraints).coords if constraints else None
            except Exception:
                logger.exception("Could not profile the candidate for the overlay")
                coords = None
            if coords:
                pts = ft_to_px(np.array([np.asarray(c, float).ravel()[:2] for c in coords]))
        path = QPainterPath()
        path.moveTo(float(pts[0][0]), float(pts[0][1]))
        for x, y in pts[1:]:
            path.lineTo(float(x), float(y))
        self.candidate_item.setPath(path)

        rings = QPainterPath()
        for i in changed:
            _ring(rings, ft_to_px([nodes[i].x, nodes[i].y]), RING_RADIUS_PX + 6)
        self.candidate_nodes_item.setPath(rings)

    def clear(self) -> None:
        self.clear_limits()
        self.clear_keep_out()
        self.clear_candidate()


def parse_or_none(text: str, action_names) -> Optional[rs.Route]:
    try:
        return rs.parse_route(text, action_names)
    except Exception:
        return None
