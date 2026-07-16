import json
import logging
import math
from typing import List

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, QSize, QSizeF, Qt, QTimer
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QMouseEvent,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QTransform,
    QUndoCommand,
    QVector2D,
)
from PyQt6.QtWidgets import (
    QApplication,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsView,
    QMessageBox,
)

import utilities.file_management
from gui import action_point, node
from motion_profiling_v2 import motion_profile_generator
from splines.spline_manager import QuinticHermiteSplineManager
from utilities import config_manager

logger = logging.getLogger(__name__)


class RobotVisualizer(QGraphicsRectItem):
    def __init__(self, rect: QRectF = None, parent=None):
        super().__init__(rect if rect else QRectF(), parent)
        self.setAcceptHoverEvents(True)
        # Dark blue color
        self._color = QColor("#152238")
        self._setup_style()

    def _setup_style(self):
        # Set up the pen (outline)
        pen = QPen(self._color)
        pen.setWidth(2)
        self.setPen(pen)

        # Set up the brush (fill)
        fill_color = QColor(self._color)
        fill_color.setAlpha(40)
        self.setBrush(QBrush(fill_color))


class PathStateCommand(QUndoCommand):
    """Snapshot-based undo command.

    The whole path already round-trips through convert_nodes/load_nodes, so a
    command is just the serialized state before and after an edit. One
    mechanism covers node adds, deletes, drags, and every context-menu
    property edit - nothing can be forgotten.
    """

    def __init__(self, widget, before: str, after: str, text: str):
        super().__init__(text)
        self._widget = widget
        self._before = before
        self._after = after
        # push() calls redo() immediately, but the edit has already happened.
        self._first_redo = True

    def redo(self):
        if self._first_redo:
            self._first_redo = False
            return
        self._widget._restore_state(self._after)

    def undo(self):
        self._widget._restore_state(self._before)


class PathWidget(QGraphicsView):
    def __init__(
        self,
        config_manager: config_manager.ConfigManager,
        parent=None,
        image_path=utilities.file_management.resource_path(
            "../assets/V5RC-PushBack-Match-2000x2000.png"
        ),
        size=QSize(250, 250),
    ):
        super().__init__(parent)
        self.parent = parent
        self.config_manager = config_manager
        self.image = QPixmap(image_path) if image_path else None
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

        # Create a QGraphicsScene
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Load the image and add it to the scene
        pixmap = QPixmap(image_path)
        self.image_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(self.image_item)

        # Set up the view
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)

        # Fit the entire image in view
        self.fitInView(self.image_item, Qt.AspectRatioMode.KeepAspectRatio)

        self.shift_pressed = False
        self.setMouseTracking(True)

        # Store the initial transform for limiting zoom out
        self.initial_transform = self.transform()

        self.time_intervals = []
        self.positions = []
        self.velocities = []
        self.accelerations = []
        self.headings = []
        self.nodes_map = []  # Represents index of node n in any of the above lists
        self.actions_map = []  # Represents index of action point n in any of the above lists

        self.nodes: List[node.Node] = []
        self.action_points: List[action_point.ActionPoint] = []
        self.start_node = None
        self.end_node = None

        self.path = None
        self.line_data: List[List[float]] = []

        self.path_item = QGraphicsPathItem()
        self.scene.addItem(self.path_item)

        # Overlays drawn on top of the path where curvature forces the robot
        # below a fraction of max velocity (added after path_item so they
        # render above it).
        self.feasibility_coloring = True
        self.slow_overlay_item = QGraphicsPathItem()
        self.slow_overlay_item.setPen(QPen(QColor("#e6a23c"), 4))
        self.scene.addItem(self.slow_overlay_item)
        self.crawl_overlay_item = QGraphicsPathItem()
        self.crawl_overlay_item.setPen(QPen(QColor("#e64c3c"), 4))
        self.scene.addItem(self.crawl_overlay_item)

        # Undo support: snapshot taken at the start of a user edit, committed
        # (pushed to the undo stack) when the edit finishes.
        self._pending_undo_state = None
        self._restoring_state = False

        self.rect_item = RobotVisualizer()
        self.scene.addItem(self.rect_item)

        self.setMinimumSize(size)

        self.zoom_factor = 1.0
        self.initial_fit = True

        self.mouseDown = False

        self.visualize = False

        self.robot_width = self.config_manager.get_value("robot", "width")
        self.robot_length = self.config_manager.get_value("robot", "length")

        self.path = QPainterPath()
        self.spline_manager = QuinticHermiteSplineManager()

        # Last generated motion profile, reused while the path and constraints
        # are unchanged (viewing two graphs, or a graph then a save, would
        # otherwise regenerate the whole profile each time).
        self._profile_cache_key = None
        self._profile_cache = None

        # Dense canvas-space samples of the drawn path (pixels) and their
        # spline parameters, kept in sync by update_spline for fast
        # closest-point queries.
        self._canvas_samples = None
        self._canvas_params = None

        # Add a timer for throttling update_path calls
        self.update_timer = QTimer()
        self.update_timer.setInterval(10)  # 10 milliseconds
        self.update_timer.timeout.connect(self._execute_update_path)
        self.update_pending = False

    def fit_image_to_view(self):
        self.fitInView(self.image_item, Qt.AspectRatioMode.KeepAspectRatio)
        if self.initial_fit:
            # Scale up the view to fill the widget as much as possible
            current_rect = self.mapToScene(self.viewport().rect()).boundingRect()
            image_rect = self.image_item.boundingRect()
            self.zoom_factor = min(
                current_rect.width() / image_rect.width(),
                current_rect.height() / image_rect.height(),
            )
            self.scale(self.zoom_factor, self.zoom_factor)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.initial_fit:
            self.fit_image_to_view()
        else:
            self.initial_fit = True

    def get_fit_in_view_scale(self):
        # Calculate the scale factor that would make the image fit in the view
        view_rect = self.viewport().rect()
        scene_rect = self.image_item.boundingRect()
        x_ratio = view_rect.width() / scene_rect.width()
        y_ratio = view_rect.height() / scene_rect.height()
        return min(x_ratio, y_ratio)

    def wheelEvent(self, event):
        if event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            return
        # Zoom factor
        zoom_factor = 1.3

        # Get the current transformation
        old_pos = self.mapToScene(event.position().toPoint())
        self.centerOn(old_pos)

        # Current scale factor
        current_scale = self.transform().m11()

        # Calculate new scale factor
        if event.angleDelta().y() > 0:
            new_scale = current_scale * zoom_factor
        else:
            new_scale = current_scale / zoom_factor

        # Check if zooming out would make the image smaller than the view
        if new_scale < self.get_fit_in_view_scale():
            new_scale = self.get_fit_in_view_scale()

        # Calculate the scale factor
        factor = new_scale / current_scale
        # Scale the view
        self.scale(factor, factor)

        # Get the new position
        # new_pos = self.mapToScene(event.position().toPoint())
        new_pos = self.mapToScene(self.viewport().rect().center())
        # Move scene to old position
        delta = self.mapToScene(event.position().toPoint()) - new_pos

        self.centerOn(old_pos - delta)

        self.initial_fit = False

    def mousePressEvent(self, event: QMouseEvent):
        self.mouseDown = True

        if (
            event.button() == Qt.MouseButton.LeftButton
            and event.modifiers() & Qt.KeyboardModifier.ShiftModifier
            and not event.modifiers() & Qt.KeyboardModifier.ControlModifier
        ):
            scene_pos = self.mapToScene(event.position().toPoint())
            self.begin_undo_state()
            self.add_node(scene_pos)
            self.commit_undo_state("Add node")

            logger.info(f"Added node at ({scene_pos.x()}, {scene_pos.y()})")

        elif (
            event.button() == Qt.MouseButton.LeftButton
            and event.modifiers() & Qt.KeyboardModifier.ControlModifier
            and event.modifiers() & Qt.KeyboardModifier.ShiftModifier
        ):
            scene_pos = self.mapToScene(event.position().toPoint())
            path_point, path_param = self.find_closest_point_on_path(
                self.path, scene_pos
            )
            if path_param is not None:
                self.begin_undo_state()
                self.add_action_point(path_point, path_param)
                self.commit_undo_state("Add action point")

                logger.info(
                    f"Added action point at ({path_point.x()}, {path_point.y()})"
                )

        # Call the parent class mousePressEvent to maintain drag functionality
        else:
            # Call the parent class mousePressEvent for other cases (like panning)
            QApplication.setOverrideCursor(Qt.CursorShape.ClosedHandCursor)
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        self.mouseDown = False

        if self.shift_pressed:
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
        else:
            QApplication.setOverrideCursor(Qt.CursorShape.OpenHandCursor)
        super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        item = self.itemAt(event.position().toPoint())
        scene_pos = self.mapToScene(event.position().toPoint())
        if isinstance(item, node.Node) or (
            item and isinstance(item.parentItem(), node.Node)
        ):
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
            self.parent.update_coords(scene_pos)
        elif isinstance(item, action_point.ActionPoint) or (
            item and isinstance(item.parentItem(), action_point.ActionPoint)
        ):
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
            self.parent.update_coords(scene_pos)
        elif self.shift_pressed:
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
            self.parent.update_coords(scene_pos)
        elif (
            scene_pos.x() < 0
            or scene_pos.x() > 2000
            or scene_pos.y() < 0
            or scene_pos.y() > 2000
        ):
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
        elif self.mouseDown:
            QApplication.setOverrideCursor(Qt.CursorShape.ClosedHandCursor)
            self.parent.update_coords(scene_pos)
        else:
            QApplication.setOverrideCursor(Qt.CursorShape.OpenHandCursor)
            if self.visualize:
                self.draw_rect(scene_pos)
            else:
                self.rect_item.hide()
            self.parent.update_coords(scene_pos)

        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Shift:
            self.shift_pressed = True
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key.Key_Shift:
            self.shift_pressed = False
            QApplication.setOverrideCursor(Qt.CursorShape.OpenHandCursor)
        super().keyReleaseEvent(event)

    def update_image_path(self, new_path: str):
        self.image = QPixmap(new_path)
        self.image_item.setPixmap(self.image)
        self.update()

    # UNDO SUPPORT
    def begin_undo_state(self):
        """Snapshot the path before a user edit. Safe to call and then never
        commit (a click that moved nothing pushes no command)."""
        if self._restoring_state or self.parent is None:
            return
        self._pending_undo_state = self.parent.convert_nodes()

    def commit_undo_state(self, text: str):
        """Push an undo command if the path actually changed since begin."""
        if self._restoring_state or self._pending_undo_state is None:
            return
        before = self._pending_undo_state
        self._pending_undo_state = None
        after = self.parent.convert_nodes()
        if after != before:
            self.parent.undo_stack.push(PathStateCommand(self, before, after, text))

    def _restore_state(self, state: str):
        self._restoring_state = True
        try:
            self.load_nodes(state)
        finally:
            self._restoring_state = False

    def _path_fingerprint(self):
        """Everything the motion profile depends on, as a hashable value.

        Keying the profile cache on actual state (instead of a dirty flag set
        from every edit path) means a missed invalidation can never serve a
        stale profile.
        """

        def as_tuple(value):
            if value is None:
                return None
            return tuple(np.asarray(value, dtype=float).ravel())

        node_state = tuple(
            (
                n.x(),
                n.y(),
                n.is_start_node,
                n.is_end_node,
                n.is_reverse_node,
                n.stop_mode,
                n.turn,
                n.wait_time,
                n.max_velocity,
                n.max_acceleration,
                as_tuple(n.tangent),
                as_tuple(n.curvature),
                n.incoming_magnitude,
                n.outgoing_magnitude,
            )
            for n in self.nodes
        )
        action_state = tuple(
            (a.t, a.stop, a.wait_time, a.max_velocity, a.max_acceleration)
            for a in self.action_points
        )
        return (node_state, action_state)

    def generate_motion_profile_lists(
        self,
        v_max: float,
        a_max: float,
        j_max: float,
        track_width: float,
        d_max: float = None,
    ):
        cfg = self.config_manager

        cache_key = (
            v_max,
            a_max,
            j_max,
            track_width,
            d_max,
            cfg.get_value("motion", "stall_accel"),
            cfg.get_value("motion", "free_speed"),
            cfg.get_value("motion", "traction_accel"),
            cfg.get_value("motion", "steering_speed_penalty"),
            self._path_fingerprint(),
        )
        if self._profile_cache is not None and self._profile_cache_key == cache_key:
            logger.info("Path and constraints unchanged; reusing cached profile")
            return self._profile_cache

        # Clear lists
        self.time_intervals: List[float] = []
        self.positions: List[float] = []
        self.velocities: List[float] = []
        self.accelerations: List[float] = []
        self.headings: List[float] = []
        self.angular_velocities: List[float] = []
        self.nodes_map: List[int] = []
        self.coords: List[float] = []
        constraints = motion_profile_generator.Constraints(
            max_vel=v_max,  # Maximum velocity in ft/s
            max_acc=a_max,  # Maximum acceleration in ft/s^2
            max_dec=a_max if d_max is None else d_max,  # Max deceleration in ft/s^2
            friction_coef=0.8,  # Coefficient of friction
            max_jerk=j_max,  # Maximum jerk in ft/s^3
            track_width=track_width,  # Distance between wheels in ft
            # Optional drivetrain model. Absent from the config -> None -> the flat
            # max_acc / max_dec above are used, exactly as before.
            stall_accel=cfg.get_value("motion", "stall_accel"),
            free_speed=cfg.get_value("motion", "free_speed"),
            traction_accel=cfg.get_value("motion", "traction_accel"),
            # ft of extra effective half-track charged to the outer wheel's speed budget,
            # paying for the voltage that turning scrub costs. Absent -> None -> no penalty.
            steering_speed_penalty=cfg.get_value("motion", "steering_speed_penalty"),
        )

        (
            self.time_intervals,
            self.positions,
            self.velocities,
            self.accelerations,
            self.headings,
            self.angular_velocities,
            self.nodes_map,
            self.actions_map,
            self.coords,
        ) = motion_profile_generator.generate_motion_profile(
            self.spline_manager, constraints
        )

        logger.info(f"Accumulated value: {self.positions[-1]}")
        logger.info(f"Goal value: {self.spline_manager.get_total_arc_length()}")
        logger.info(
            f"Error: {self.spline_manager.get_total_arc_length() - self.positions[-1]}"
        )
        # generate_motion_profile now emits the final node's nodes_map entry
        # itself (placed before any final turn/wait samples), so no caller-side
        # append is needed.

        result = (
            self.time_intervals,
            self.positions,
            self.velocities,
            self.accelerations,
            self.headings,
            self.angular_velocities,
            self.nodes_map,
            self.actions_map,
            self.coords,
        )
        self._profile_cache_key = cache_key
        self._profile_cache = result
        return result

    def update_spline(
        self,
        points: List[QPointF],
        nodes: List[node.Node],
        action_points: List[action_point.ActionPoint],
    ):
        # convert points to numpy array of floats and from pixels to feet
        points = np.array([[point.x(), point.y()] for point in points])
        points = (points / 2000 - 0.5) * 12.2038433932

        self.spline_manager.build_path(points, nodes, action_points)
        t_values = np.linspace(0, len(points) - 1, 100 * len(self.nodes))
        spline_points = self.spline_manager.get_points_at_parameters(t_values)
        spline_points = (spline_points / 12.2038433932 + 0.5) * 2000

        # Keep the sample table for fast closest-point queries.
        self._canvas_params = t_values
        self._canvas_samples = spline_points

        self.path = QPainterPath()
        if len(spline_points) > 0:
            self.path.moveTo(spline_points[0][0], spline_points[0][1])
            for p in spline_points[1:]:
                self.path.lineTo(p[0], p[1])

        self._update_feasibility_overlay(t_values, spline_points)

        return spline_points

    def _update_feasibility_overlay(self, t_values, spline_points_px):
        """Color path stretches where curvature caps the chassis well below
        max velocity: orange under 60%, red under 35%. Uses the same
        v <= max_vel / (1 + |k| * lever) bound as the profiler."""
        if not self.feasibility_coloring or len(t_values) < 2:
            self.slow_overlay_item.setPath(QPainterPath())
            self.crawl_overlay_item.setPath(QPainterPath())
            return

        d1 = self.spline_manager.get_derivatives_at_parameters(t_values)
        d2 = self.spline_manager.get_second_derivatives_at_parameters(t_values)
        speed_sq = d1[:, 0] ** 2 + d1[:, 1] ** 2
        curvature = np.zeros(len(t_values))
        mask = speed_sq >= 1e-10
        curvature[mask] = (
            d1[mask, 0] * d2[mask, 1] - d1[mask, 1] * d2[mask, 0]
        ) / speed_sq[mask] ** 1.5

        track_width = self.config_manager.get_value("robot", "track_width") / 12
        penalty = (
            self.config_manager.get_value("motion", "steering_speed_penalty") or 0
        )
        lever = track_width / 2 + penalty
        speed_ratio = 1.0 / (1.0 + np.abs(curvature) * lever)

        self.slow_overlay_item.setPath(
            self._path_from_mask(spline_points_px, speed_ratio < 0.6)
        )
        self.crawl_overlay_item.setPath(
            self._path_from_mask(spline_points_px, speed_ratio < 0.35)
        )

    @staticmethod
    def _path_from_mask(points, mask) -> QPainterPath:
        """One QPainterPath covering every consecutive run of masked samples,
        widened by one sample each side so runs connect to the base path."""
        path = QPainterPath()
        idx = np.flatnonzero(mask)
        if idx.size == 0:
            return path

        breaks = np.flatnonzero(np.diff(idx) > 1)
        run_starts = np.concatenate(([0], breaks + 1))
        run_ends = np.concatenate((breaks, [idx.size - 1]))
        for s, e in zip(run_starts, run_ends):
            i0 = max(idx[s] - 1, 0)
            i1 = min(idx[e] + 1, len(points) - 1)
            path.moveTo(points[i0][0], points[i0][1])
            for p in points[i0 + 1 : i1 + 1]:
                path.lineTo(p[0], p[1])
        return path

    def update_path(self):
        # Schedule the update if not already pending
        if not self.update_pending:
            self.update_pending = True
            if not self.update_timer.isActive():
                self.update_timer.start()

    def _execute_update_path(self):
        # Perform the actual update logic
        self.update_pending = False
        self.update_timer.stop()

        if self.start_node and self.end_node and len(self.nodes) > 1:
            points = [self.start_node.pos()]
            for node in self.nodes:
                if node.is_end_node or node.is_start_node:
                    continue
                points.append(node.pos())
            points.append(self.end_node.pos())
            self.update_spline(points, self.nodes, self.action_points)
        else:
            self.path = QPainterPath()
            self._canvas_samples = None
            self._canvas_params = None
            self.slow_overlay_item.setPath(QPainterPath())
            self.crawl_overlay_item.setPath(QPainterPath())

        for action_node in self.action_points:
            t = action_node.t
            point = self.spline_manager.get_point_at_parameter(t)
            point = (point / (12.2038433932) + 0.5) * 2000
            action_node.setPos(QPointF(point[0], point[1]))

        pen = QPen(QColor("#008080"), 4)  # dark purple (looks cool)
        # pen = QPen(QColor("#0a0612"), 4)  # dark purple (looks cool)
        self.path_item.setPen(pen)
        self.path_item.setPath(self.path)

        self.viewport().update()  # Request a repaint of the viewport

    def save(self):
        self.parent.auto_save()

    def update_lines(self):
        self.repaint()
        self.update()
        self.show()

    # NODE LOGIC
    def get_nodes(self):
        return self.nodes

    def add_node(self, point: QPointF, pos=-1):
        new_node = node.Node(point.x(), point.y(), self)
        if pos == -1 and self.end_node is not None:
            pos = len(self.nodes) - 1
        if pos == -1:
            self.nodes.append(new_node)
        else:
            self.nodes.insert(pos, new_node)

        for p in self.action_points:
            if (p.t//1 >= pos):
                p.t += 1

        new_node.setPos(point)
        self.scene.addItem(new_node)

        new_node.show()
        self.update_path()

        self.parent.update_path_page()

        logger.info(f"Node created at ({new_node.abs_x}, {new_node.abs_y})")
        return new_node

    def add_action_point(self, point: QPointF, t: float, pos=-1):
        new_action_point = action_point.ActionPoint(point.x(), point.y(), t, self)
        if len(self.action_points) == 0:
            self.action_points.append(new_action_point)

        else:
            for i in range(len(self.action_points)):
                if self.action_points[i].t > t:
                    self.action_points.insert(i, new_action_point)
                    break
            if i == len(self.action_points) - 1:
                self.action_points.append(new_action_point)

        self.scene.addItem(new_action_point)
        new_action_point.show()

        logger.info(
            f"Action point created at ({new_action_point.abs_x}, {new_action_point.abs_y})"
        )
        return new_action_point

    def remove_node(self, remove_node):
        if remove_node in self.nodes:
            node_idx = self.nodes.index(remove_node)
            for action_point in self.action_points:
                if action_point.t >= node_idx and node_idx != 0:
                    action_point.t -= 1

            self.nodes.remove(remove_node)
            if remove_node == self.start_node:
                self.start_node = None
            if remove_node == self.end_node:
                self.end_node = None

        self.parent.update_path_page()
        self.update_path()

    def remove_action_point(self, remove_action_point):
        if remove_action_point in self.action_points:
            self.action_points.remove(remove_action_point)

        self.update_path()

    def set_start_node(self, new_start_node):
        if self.start_node:
            self.start_node.is_start_node = False
            self.start_node.update()
            self.update()

        self.parent.update_path_page()
        self.start_node = new_start_node

        # Move start node to start of nodes list
        nodeindex = 0
        for i in range(len(self.nodes)):
            if self.nodes[i] == self.start_node:
                nodeindex = i

        temp = self.nodes[0]
        self.nodes[0] = self.start_node
        self.nodes[nodeindex] = temp

        self.update_path()

    def set_selected_node(self, node):
        self.parent.set_selected_node(node)

    def get_tangent_at_node(self, node):
        if self.spline_manager.splines:
            return self.spline_manager.get_derivative_at_parameter(
                self.nodes.index(node)
            )
        else:
            return [0, 0]
        
    def get_tangent_derivative_at_node(self, node):
        if self.spline_manager.splines:
            return self.spline_manager.get_second_derivative_at_parameter(
                self.nodes.index(node)
            )
        else:
            return [0, 0]

    def get_incoming_magnitude_at_node(self, node):
        if self.spline_manager.splines:
            return self.spline_manager.get_magnitudes_at_parameter(
                self.nodes.index(node)
            )[0]
        else:
            return 0

    def get_outgoing_magnitude_at_node(self, node):
        if self.spline_manager.splines:
            return self.spline_manager.get_magnitudes_at_parameter(
                self.nodes.index(node)
            )[1]
        else:
            return 0

    def set_end_node(self, node):
        if self.end_node:
            self.end_node.is_end_node = False
            self.end_node.update()
            self.update()

        self.parent.update_path_page()
        self.end_node = node

        # Move end node to end of nodes list
        nodeindex = 0
        for i in range(len(self.nodes)):
            if self.nodes[i] == self.end_node:
                nodeindex = i

        temp = self.nodes[-1]
        self.nodes[-1] = self.end_node
        self.nodes[nodeindex] = temp

        self.update_path()

    def clear_start_node(self):
        self.start_node = None

    def clear_end_node(self):
        self.end_node = None

    def clear_nodes(self):
        self.clearing_nodes = True
        for node in list(self.nodes):
            node.delete_node()
        self.nodes.clear()

        for action_point in list(self.action_points):
            action_point.delete_action_point()
        self.action_points.clear()

        self.start_node = None
        self.end_node = None
        self.clearing_nodes = False

        self.update_path()
        self.update()
        # self.auto_save()

    def convert_point(self, point: QPointF):
        point.setX((point.x() / (146.446120719) + 0.5) * 2000)
        point.setY((point.y() / (146.446120719) + 0.5) * 2000)

        return point

    def mirror_nodes(self):
        for n in self.nodes:
            n.setPos(QPointF(n.x(), 2000 - n.y()))
            # n.turn = -n.turn
            # if (n.get_tangent() is not None):
            #     n.set_tangent(n.get_tangent() * -1)
        self.update_path()

    def load_nodes(self, node_str: str) -> None:
        try:
            data = json.loads(node_str)
        except json.JSONDecodeError as e:
            logger.error(f"Could not parse route data: {e}")
            QMessageBox.warning(
                self,
                "Invalid Route",
                f"The route file is not valid JSON and cannot be loaded.\n\n{e}",
            )
            return
        if len(data) == 2:
            nodes_data = data[0]
            action_data = data[1]
        else:
            nodes_data = data
            action_data = []
        self.clear_nodes()
        for node_data in nodes_data:
            if len(node_data) > 4:
                logger.info(f"Loading node data: {node_data}")
                print(node_data)
                node = self.add_node(
                    self.convert_point(QPointF(node_data[0], node_data[1]))
                )
                self.start_node = node if bool(node_data[2]) else self.start_node
                node.is_start_node = bool(node_data[2])
                self.end_node = node if bool(node_data[3]) else self.end_node
                node.is_end_node = bool(node_data[3])
                node.is_reverse_node = bool(node_data[4])
                node.stop_mode = int(node_data[5])
                node.turn = node_data[6]

                node.wait_time = node_data[7]
                node.max_velocity = node_data[8]
                node.max_acceleration = node_data[9]
                node.tangent = node_data[10]

                # node.curvature is set below, where the legacy zero is filtered out.
                node.incoming_magnitude = node_data[12]
                node.outgoing_magnitude = node_data[13]
                node.set_action_values(node_data[14:])


                # node.wait_time = node_data[7]
                # node.tangent = node_data[8]
                # # node.incoming_magnitude = node_data[9]
                # # node.outgoing_magnitude = node_data[10]
                # # node.set_action_values(node_data[11:])
                # node.curvature = node_data[9]
                # node.incoming_magnitude = node_data[10]
                # node.outgoing_magnitude = node_data[11]
                # node.set_action_values(node_data[12:])

                if node_data[10] is not None:
                    node.set_tangent(np.array(node_data[10]))

                # Fix for [0,0] fake write that was happening earlier - will just count as not set
                derivative = node_data[11]
                if derivative is not None and np.any(np.asarray(derivative, dtype=float)):
                    node.set_tangent_derivative(np.array(derivative))
                else:
                    node.set_tangent_derivative(None)
                node.show()

        for action_data in action_data:
            action_point = self.add_action_point(
                self.convert_point(QPointF(action_data[0], action_data[1])),
                action_data[2],
            )
            action_point.stop = action_data[3]
            action_point.wait_time = action_data[4]
            action_point.max_velocity = action_data[5]
            action_point.max_acceleration = action_data[6]
            action_point.set_action_values(action_data[7:])

            action_point.show()

        self.update_path()

    def index_of(self, node: node.Node) -> int:
        return self.nodes.index(node)

    def auto_save(self):
        if self.parent.current_working_file is not None:
            self.parent.auto_save()

    def toggle_visualization(self, state: bool) -> None:
        logger.info(f"Visualizing toggled to : {state}")
        self.visualize = state
        self.update()

    def set_feasibility_coloring(self, enabled: bool) -> None:
        logger.info(f"Feasibility coloring toggled to: {enabled}")
        self.feasibility_coloring = enabled
        self.update_path()

    def find_closest_point_on_path(self, path: QPainterPath, point: QPointF) -> QPointF:
        """
        Find the closest point on a QPainterPath to a given point.

        Args:
            path (QPainterPath): The path to search on
            point (QPointF): The reference point to find the closest point to

        Returns:
            QPointF: The closest point on the path
        """
        if (
            path is None
            or path.isEmpty()
            or self._canvas_samples is None
            or len(self._canvas_samples) == 0
        ):
            return QPointF(), None

        target = np.array([point.x(), point.y()])

        # Coarse pass: nearest of the already-drawn path samples (pixel space).
        dists_sq = np.sum((self._canvas_samples - target) ** 2, axis=1)
        idx = int(np.argmin(dists_sq))

        # Fine pass: re-sample the parameter window around that sample.
        lo = self._canvas_params[max(idx - 1, 0)]
        hi = self._canvas_params[min(idx + 1, len(self._canvas_params) - 1)]
        fine_params = np.linspace(lo, hi, 64)
        fine_points = self.spline_manager.get_points_at_parameters(fine_params)
        fine_points = (fine_points / 12.2038433932 + 0.5) * 2000

        fine_idx = int(np.argmin(np.sum((fine_points - target) ** 2, axis=1)))
        closest_point = fine_points[fine_idx]

        return (
            QPointF(closest_point[0], closest_point[1]),
            float(fine_params[fine_idx]),
        )

    def find_path_angle_at_point(
        self, path: QPainterPath, point: QPointF, delta: float = 0.01
    ) -> float:
        """
        Calculate the angle of the path at the given point by sampling nearby points.

        Args:
            path: QPainterPath to calculate angle on
            point: Point to find the closest position on path
            delta: Small offset for calculating tangent

        Returns:
            Angle in radians
        """
        # Canvas x/y scale to field x/y with the same positive factor, so the
        # tangent's angle is identical in both spaces.
        _, param = self.find_closest_point_on_path(path, point)
        if param is None:
            return 0.0
        derivative = self.spline_manager.get_derivative_at_parameter(param)
        return math.atan2(derivative[1], derivative[0])

    def draw_rect(self, pt):
        # Create the styled rect item if it doesn't exist
        if not hasattr(self, "rect_item"):
            self.rect_item = RobotVisualizer()
            self.scene().addItem(self.rect_item)

        path_point, path_param = self.find_closest_point_on_path(self.path, pt)
        if path_point is None or path_param is None:
            self.rect_item.hide()
            return
        if path_point == QPointF(0, 0):
            self.rect_item.hide()
            return

        dist = QVector2D(pt - path_point).length()
        if dist > 100:
            self.rect_item.hide()
            return

        # Angle at the path point, straight from the spline tangent (canvas and
        # field axes scale identically, so the angle carries over).
        derivative = self.spline_manager.get_derivative_at_parameter(path_param)
        angle = math.atan2(derivative[1], derivative[0])

        # Create rectangle centered at path point
        rect_size = QSizeF(
            self.robot_width * (2000 / (146.446120719)),
            self.robot_length * (2000 / (146.446120719)),
        )
        center_rect = QRectF(
            path_point.x() - rect_size.width() / 2,
            path_point.y() - rect_size.height() / 2,
            rect_size.width(),
            rect_size.height(),
        )

        # Create transform for rotation around rectangle center
        transform = QTransform()
        transform.translate(path_point.x(), path_point.y())
        transform.rotate(math.degrees(angle))
        transform.translate(-path_point.x(), -path_point.y())

        # Apply transform to rectangle
        self.rect_item.setTransform(transform)
        self.rect_item.setRect(center_rect)
        self.rect_item.show()

        self.viewport().update()
