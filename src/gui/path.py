import logging
import math
from typing import List

import numpy as np
from PyQt6.QtCore import QPointF, QRect, QRectF, QSize, QSizeF, Qt, QTimer
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
    QRubberBand,
)

import utilities.file_management
from gui import action_point, node
from gui.optimizer_overlay import OptimizerOverlay
from motion_profiling_v2 import motion_profile_generator
from splines.spline_manager import QuinticHermiteSplineManager
from utilities.field_scale import FIELD_IMAGE_FT, FIELD_IMAGE_IN
from utilities import config_manager, route_serializer

logger = logging.getLogger(__name__)

# Opacity of everything outside the focus range (PathWidget.set_focus_from/_to)
FOCUS_DIM_OPACITY = 0.15


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

    Any change to the path will result in a difference in the path state, so a
    command is really just the serialized state before and after an edit.
    """

    def __init__(
        self,
        widget,
        before: str,
        after: str,
        text: str,
        point_before=None,
        point_after=None,
    ):
        super().__init__(text)
        self._widget = widget
        self._before = before
        self._after = after
        # Index of the selected action point in each state. An edit can
        # reorder action_points (sorted by t), so the index has to be taken
        # from the state being restored, not from the current list.
        self._point_before = point_before
        self._point_after = point_after
        # push() calls redo() immediately, but the edit has already happened.
        self._first_redo = True

    def redo(self):
        if self._first_redo:
            self._first_redo = False
            return
        self._widget._restore_state(self._after, self._point_after)

    def undo(self):
        self._widget._restore_state(self._before, self._point_before)


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
        
        # Create a QGraphicsScene
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # Load the image and add it to the scene
        pixmap = QPixmap(image_path)
        self.image_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(self.image_item)

        # Set up the view
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)

        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
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

        # Multi-selection (rubber band / ctrl+click); the single selected_node
        # panel model on the GUI manager is unchanged and coexists with this.
        self.selected_nodes: set = set()
        self._rubber_band = None
        self._rubber_band_origin = None  # viewport QPoint while a band drag is active

        self.path = None
        self.line_data: List[List[float]] = []

        self.path_item = QGraphicsPathItem()
        self.scene.addItem(self.path_item)
        # Focus range: the nodes from focus_from to focus_to (None = the last node)
        # draw and edit as normal; the rest of the path, its nodes and action points
        # fade to FOCUS_DIM_OPACITY and stop taking clicks. View-only: not saved with
        # the route or pushed to the undo stack. While it is set, path_item is drawn
        # faded and focus_path_item redraws the focused stretch over it.
        self.focus_from = None
        self.focus_to = None
        self.focus_path_item = QGraphicsPathItem()
        self.scene.addItem(self.focus_path_item)

        # Overlays drawn on top of the path where curvature forces the robot
        # below a fraction of max velocity (added after path_item so they
        # render above it).
        self.feasibility_coloring = True
        # What the last generated profile asks of the chassis (FeasibilityReport);
        # None until a profile has been generated.
        self.feasibility_report = None
        self.slow_overlay_item = QGraphicsPathItem()
        self.slow_overlay_item.setPen(QPen(QColor("#e6a23c"), 4))
        self.scene.addItem(self.slow_overlay_item)
        self.crawl_overlay_item = QGraphicsPathItem()
        self.crawl_overlay_item.setPen(QPen(QColor("#e64c3c"), 4))
        self.scene.addItem(self.crawl_overlay_item)
        # Where the robot is actually sent when a spin-through turn swings it off the
        # spline: the profile's own poses, dashed over the path.
        self.drive_overlay_item = QGraphicsPathItem()
        drive_pen = QPen(QColor("#f0f0f0"), 2)
        drive_pen.setStyle(Qt.PenStyle.DashLine)
        self.drive_overlay_item.setPen(drive_pen)
        self.scene.addItem(self.drive_overlay_item)
        # Path-optimizer limits, keep-out zones and result preview (the Optimize page
        # drives it).
        self.optimizer_overlay = OptimizerOverlay(self.scene)
        # The route's keep-out zones (route_serializer.KeepOutRecord, field inches).
        # Saved with the route and part of every undo snapshot; only the optimizer
        # reads them.
        self.keep_out: list = []

        # Undo support: snapshot taken at the start of a user edit, committed
        # (pushed to the undo stack) when the edit finishes.
        self._pending_undo_state = None
        self._pending_undo_point = None
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
        new_scale = max(new_scale, self.get_fit_in_view_scale())

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
            if path_param is not None and self.is_param_in_focus(path_param):
                self.begin_undo_state()
                self.add_action_point(path_point, path_param)
                self.commit_undo_state("Add action point")

                logger.info(
                    f"Added action point at ({path_point.x()}, {path_point.y()})"
                )

        elif (
            event.button() == Qt.MouseButton.LeftButton
            and event.modifiers() & Qt.KeyboardModifier.ControlModifier
            and not event.modifiers() & Qt.KeyboardModifier.ShiftModifier
        ):
            # Ctrl+click on a node toggles it in the multi-selection;
            # ctrl+drag on empty canvas draws a rubber-band selection box.
            # Swallowed (no super()) so neither gesture pans the view.
            hit = self._resolve_node_at(event.position().toPoint())
            if hit is not None:
                self.toggle_node_selection(hit)
            else:
                self._rubber_band_origin = event.position().toPoint()
                if self._rubber_band is None:
                    self._rubber_band = QRubberBand(
                        QRubberBand.Shape.Rectangle, self.viewport()
                    )
                self._rubber_band.setGeometry(
                    QRect(self._rubber_band_origin, QSize())
                )
                self._rubber_band.show()
                QApplication.setOverrideCursor(Qt.CursorShape.CrossCursor)

        # Call the parent class mousePressEvent to maintain drag functionality
        else:
            # Call the parent class mousePressEvent for other cases (like panning)
            if (
                event.button() == Qt.MouseButton.LeftButton
                and self._resolve_node_at(event.position().toPoint()) is None
            ):
                self.clear_multi_selection()
                self.set_selected_node(None)
            QApplication.setOverrideCursor(Qt.CursorShape.ClosedHandCursor)
            super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if self._rubber_band_origin is not None:
            band = QRect(
                self._rubber_band_origin, event.position().toPoint()
            ).normalized()
            scene_rect = self.mapToScene(band).boundingRect()
            self.set_multi_selection(
                {
                    n
                    for n in self.nodes
                    if scene_rect.contains(n.scenePos()) and self.is_node_in_focus(n)
                }
            )
            # A panel selection outside the fresh box selection would keep a
            # stray ring (and stale panel) around - drop it.
            if getattr(self.parent, "selected_node", None) not in self.selected_nodes:
                self.set_selected_node(None)
            self._rubber_band.hide()
            self._rubber_band_origin = None
            self.mouseDown = False
            QApplication.setOverrideCursor(Qt.CursorShape.OpenHandCursor)
            return

        self.mouseDown = False

        if self.shift_pressed:
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
        else:
            QApplication.setOverrideCursor(Qt.CursorShape.OpenHandCursor)
        super().mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        if (
            self._rubber_band_origin is not None
            and event.buttons() & Qt.MouseButton.LeftButton
        ):
            self._rubber_band.setGeometry(
                QRect(
                    self._rubber_band_origin, event.position().toPoint()
                ).normalized()
            )
            return

        item = self.itemAt(event.position().toPoint())
        scene_pos = self.mapToScene(event.position().toPoint())
        if isinstance(item, node.Node) or (
            item and isinstance(item.parentItem(), node.Node)
        ) or isinstance(item, action_point.ActionPoint) or (
            item and isinstance(item.parentItem(), action_point.ActionPoint)
        ) or self.shift_pressed:
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
        elif event.key() == Qt.Key.Key_Escape:
            # First Esc drops the selection; with nothing selected, it clears the
            # focus range
            nothing_selected = not self.selected_nodes and (
                getattr(self.parent, "selected_node", None) is None
            )
            if nothing_selected and self.focus_from is not None:
                self.clear_focus()
            self.clear_multi_selection()
            self.set_selected_node(None)
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
        """Snapshot the path before an edit. Safe to call and then never
        commit (a click that moved nothing pushes no command)."""
        if self._restoring_state or self.parent is None:
            return
        self._pending_undo_state = self.parent.convert_nodes()
        self._pending_undo_point = self._selected_point_index()

    def begin_undo_state_once(self):
        """begin_undo_state, but keep an already-pending snapshot.

        Spin-box edits fire once per tick/keystroke and only commit on
        editingFinished, so the first tick takes the snapshot and the rest of
        the session accumulates into the same undo entry.
        """
        if self._pending_undo_state is None:
            self.begin_undo_state()

    def commit_undo_state(self, text: str):
        """Push an undo command if the path actually changed since begin."""
        if self._restoring_state or self._pending_undo_state is None:
            return
        before = self._pending_undo_state
        self._pending_undo_state = None
        after = self.parent.convert_nodes()
        if after != before:
            self.parent.undo_stack.push(
                PathStateCommand(
                    self,
                    before,
                    after,
                    text,
                    self._pending_undo_point,
                    self._selected_point_index(),
                )
            )
            # Wait, speed and accel edits change the timing without a path rebuild.
            self.parent.schedule_path_time_update()

    def _selected_point_index(self):
        selected = getattr(self.parent, "selected_action_point", None)
        if selected is not None and selected in self.action_points:
            return self.action_points.index(selected)
        return None

    def _restore_state(self, state: str, point_idx=None):
        # The restore rebuilds every node object, so remember the selection by
        # index and re-select its replacement - otherwise the Path panel is
        # left pointing at a destroyed node.
        selected_idx = None
        selected = getattr(self.parent, "selected_node", None)
        if selected is not None and selected in self.nodes:
            selected_idx = self.nodes.index(selected)
        # The focus range too, by index
        focus_idx = None
        bounds = self._focus_bounds()
        if bounds is not None:
            focus_idx = (bounds[0], None if self.focus_to is None else bounds[1])

        # The multi-selection is not preserved across undo/redo; only the
        # single panel selection survives (by index, below).
        self.selected_nodes.clear()

        self._restoring_state = True
        try:
            self.load_nodes(state)
        finally:
            self._restoring_state = False

        if focus_idx is not None and focus_idx[0] < len(self.nodes):
            self.focus_from = self.nodes[focus_idx[0]]
            self.focus_to = (
                self.nodes[focus_idx[1]]
                if focus_idx[1] is not None and focus_idx[1] < len(self.nodes)
                else None
            )
            self._apply_focus()

        if selected_idx is not None and selected_idx < len(self.nodes):
            self.parent.set_selected_node(self.nodes[selected_idx])
        elif point_idx is not None and point_idx < len(self.action_points):
            self.parent.set_selected_action_point(self.action_points[point_idx])
        else:
            self.parent.set_selected_node(None)

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
                n.stop,  # auto_stop also follows the wait_lift action
                n.turn,
                n.spin_through,
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

    def build_constraints(
        self,
        v_max: float,
        a_max: float,
        j_max: float,
        track_width: float,
        d_max: float = None,
    ) -> motion_profile_generator.Constraints:
        """The profiler constraints for these limits plus the config's drivetrain
        model. The path optimizer is given the same ones, so its times match the
        status bar's."""
        cfg = self.config_manager
        return motion_profile_generator.Constraints(
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
            # Yaw budget (rad/s, rad/s^2): what the chassis can actually turn at, which
            # the wheel-speed bound overstates on pivot-tight corners. Absent -> None ->
            # no cap.
            max_angular_velocity=cfg.get_value("motion", "max_angular_velocity"),
            max_angular_acceleration=cfg.get_value("motion", "max_angular_acceleration"),
            # Per-side overrides of the yaw cap, by which wheel is outer. Absent ->
            # None -> max_angular_velocity for that side.
            max_angular_velocity_left_outer=cfg.get_value(
                "motion", "max_angular_velocity_left_outer"
            ),
            max_angular_velocity_right_outer=cfg.get_value(
                "motion", "max_angular_velocity_right_outer"
            ),
            # In-place turns have their own (rad/s, rad/s^2): a spin loads the chassis
            # differently from a pivot. Absent -> None -> the flat limits above.
            max_turn_velocity=cfg.get_value("motion", "max_turn_velocity"),
            max_turn_acceleration=cfg.get_value("motion", "max_turn_acceleration"),
            # Spin-through turns spin while braking and launching, which the in-place
            # limits overstate. Absent -> None -> max_turn_velocity / _acceleration.
            spin_through_velocity=cfg.get_value("motion", "spin_through_velocity"),
            spin_through_acceleration=cfg.get_value("motion", "spin_through_acceleration"),
        )

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
            cfg.get_value("motion", "max_angular_velocity"),
            cfg.get_value("motion", "max_angular_acceleration"),
            cfg.get_value("motion", "max_angular_velocity_left_outer"),
            cfg.get_value("motion", "max_angular_velocity_right_outer"),
            cfg.get_value("motion", "max_turn_velocity"),
            cfg.get_value("motion", "max_turn_acceleration"),
            cfg.get_value("motion", "spin_through_velocity"),
            cfg.get_value("motion", "spin_through_acceleration"),
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
        constraints = self.build_constraints(v_max, a_max, j_max, track_width, d_max)

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

        self.feasibility_report = motion_profile_generator.check_trajectory_feasibility(
            self.time_intervals, self.velocities, self.angular_velocities, constraints,
            spin_through=motion_profile_generator.spin_through_samples(
                self.spline_manager.nodes, self.nodes_map, len(self.time_intervals)
            ),
        )
        logger.info(f"Trajectory feasibility: {self.feasibility_report.summary()}")
        for warning in self.feasibility_report.warnings:
            logger.warning(f"Trajectory feasibility: {warning}")

        logger.info(f"Accumulated value: {self.positions[-1]}")
        logger.info(f"Goal value: {self.spline_manager.get_total_arc_length()}")
        logger.info(
            f"Error: {self.spline_manager.get_total_arc_length() - self.positions[-1]}"
        )
        # generate_motion_profile now emits the final node's nodes_map entry
        # itself (placed before any final turn/wait samples), so no caller-side
        # append is needed.

        self._update_drive_overlay()

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
        points = (points / 2000 - 0.5) * FIELD_IMAGE_FT

        self.spline_manager.build_path(points, nodes, action_points)
        t_values = np.linspace(0, len(points) - 1, 100 * len(self.nodes))
        spline_points = self.spline_manager.get_points_at_parameters(t_values)
        spline_points = (spline_points / FIELD_IMAGE_FT + 0.5) * 2000

        # Keep the sample table for fast closest-point queries.
        self._canvas_params = t_values
        self._canvas_samples = spline_points

        self.path = QPainterPath()
        if len(spline_points) > 0:
            self.path.moveTo(spline_points[0][0], spline_points[0][1])
            for p in spline_points[1:]:
                self.path.lineTo(p[0], p[1])

        in_focus = self._focus_mask(t_values)
        self.focus_path_item.setPath(
            QPainterPath() if in_focus is None
            else self._path_from_mask(spline_points, in_focus)
        )

        self._update_feasibility_overlay(t_values, spline_points)

        return spline_points

    def _update_feasibility_overlay(self, t_values, spline_points_px):
        """Color path stretches where curvature caps the chassis well below
        max velocity: orange under 60%, red under 35%. Uses the same
        v <= max_vel / (1 + |k| * lever) bound as the profiler, and the yaw-rate
        cap v <= max_angular_velocity / |k| when one is configured."""
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

        max_vel = self.config_manager.get_value("motion", "max_velocity") or 0
        yaw_cap = self.config_manager.get_value("motion", "max_angular_velocity")
        if yaw_cap and max_vel > 0:
            abs_curvature = np.abs(curvature)
            yaw_ratio = np.full(len(t_values), np.inf)
            bending = abs_curvature > 1e-9
            yaw_ratio[bending] = yaw_cap / (abs_curvature[bending] * max_vel)
            speed_ratio = np.minimum(speed_ratio, yaw_ratio)

        # Only inside the focus range, when one is set
        in_focus = self._focus_mask(t_values)
        if in_focus is None:
            in_focus = np.ones(len(t_values), dtype=bool)
        self.slow_overlay_item.setPath(
            self._path_from_mask(spline_points_px, (speed_ratio < 0.6) & in_focus)
        )
        self.crawl_overlay_item.setPath(
            self._path_from_mask(spline_points_px, (speed_ratio < 0.35) & in_focus)
        )

    def _update_drive_overlay(self):
        """Draw the profile's poses when any spin-through turn blends, since those
        leave the spline; clear it otherwise."""
        path = QPainterPath()
        blending = any(
            motion_profile_generator.blends_turn(n, i, len(self.nodes))
            for i, n in enumerate(getattr(self.spline_manager, "nodes", None) or [])
        )
        coords = getattr(self, "coords", None) or []
        bounds = self._focus_bounds()
        nodes_map = getattr(self, "nodes_map", None) or []
        if bounds is not None and coords and len(nodes_map) > bounds[1]:
            # The profile's samples from arriving at the first focused node to
            # arriving at the last (nodes_map: node -> its first sample)
            coords = coords[nodes_map[bounds[0]] : nodes_map[bounds[1]] + 1]
        if blending and coords:
            points = [
                (np.asarray(c, dtype=float).ravel()[:2] / FIELD_IMAGE_FT + 0.5) * 2000
                for c in coords
            ]
            path.moveTo(points[0][0], points[0][1])
            for p in points[1:]:
                path.lineTo(p[0], p[1])
        self.drive_overlay_item.setPath(path)

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
            # Drop splines built from the old node list; the per-node accessors
            # index them by node position and would run off the end.
            self.spline_manager.splines = []
            self.path = QPainterPath()
            self._canvas_samples = None
            self._canvas_params = None
            self.slow_overlay_item.setPath(QPainterPath())
            self.crawl_overlay_item.setPath(QPainterPath())
            self.drive_overlay_item.setPath(QPainterPath())
            self.focus_path_item.setPath(QPainterPath())

        if self.spline_manager.splines:
            for action_node in self.action_points:
                t = action_node.t
                point = self.spline_manager.get_point_at_parameter(t)
                point = (point / FIELD_IMAGE_FT + 0.5) * 2000
                action_node.setPos(QPointF(point[0], point[1]))

        if getattr(self.parent, "selected_action_point", None) is not None:
            self.parent.refresh_selected_action_point()

        pen = QPen(QColor("#008080"), 4)  # dark purple (looks cool)
        # pen = QPen(QColor("#0a0612"), 4)  # dark purple (looks cool)
        self.path_item.setPen(pen)
        self.path_item.setPath(self.path)
        self.focus_path_item.setPen(pen)
        self._apply_focus()

        self.viewport().update()  # Request a repaint of the viewport
        self.parent.schedule_path_time_update()
        # The Optimize page's limit overlay follows the path (no-op while hidden).
        if hasattr(self.parent, "refresh_optimizer_overlay"):
            self.parent.refresh_optimizer_overlay()

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
        self.clear_multi_selection()
        new_node = node.Node(point.x(), point.y(), self)
        if pos == -1 and self.end_node is not None:
            pos = len(self.nodes) - 1
        if pos == -1:
            # Appended past the last segment: no existing segment is renumbered.
            self.nodes.append(new_node)
        else:
            self.nodes.insert(pos, new_node)
            # Segments from pos onward are renumbered one higher; keep their
            # action points on them.
            for p in self.action_points:
                if p.t // 1 >= pos:
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
        self.parent.update_path_page()

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

        # A deleted node cannot stay selected - the Path panel would keep
        # editing a destroyed object.
        self.selected_nodes.discard(remove_node)
        if getattr(self.parent, "selected_node", None) is remove_node:
            self.parent.set_selected_node(None)

        self.parent.update_path_page()
        self.update_path()

    def remove_action_point(self, remove_action_point):
        if remove_action_point in self.action_points:
            self.action_points.remove(remove_action_point)

        # Same as remove_node: the panel must not keep editing a deleted point.
        if getattr(self.parent, "selected_action_point", None) is remove_action_point:
            self.parent.set_selected_action_point(None)

        self.parent.update_path_page()
        self.update_path()

    def sort_action_points(self):
        """Keep action_points in path order after a t change.

        The profile generator walks this list in order and only fires the next
        point once the path passes its t, so a point moved past another would
        otherwise be skipped entirely.
        """
        self.action_points.sort(key=lambda a: a.t)
        self.parent.update_path_page()

    def max_action_point_t(self):
        """Largest valid t: the path runs over len(nodes) - 1 segments."""
        return max(len(self.nodes) - 1, 0)

    def action_point_distance(self, point):
        """Arc length (inches) from the start of the path to point, or None
        while there is no path."""
        if not self.spline_manager.splines:
            return None
        return self.spline_manager.time_to_distance(point.t) * 12

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

    def set_selected_action_point(self, point):
        self.parent.set_selected_action_point(point)

    def is_action_point_selected(self, point):
        return getattr(self.parent, "selected_action_point", None) is point

    # MULTI-SELECTION
    def is_node_selected(self, n):
        return n in self.selected_nodes

    def is_node_highlighted(self, n):
        """Whether n should paint the selection ring: part of the
        multi-selection or the single panel-selected node."""
        return (
            n in self.selected_nodes
            or getattr(self.parent, "selected_node", None) is n
        )

    def set_multi_selection(self, nodes):
        new_selection = set(nodes)
        changed = self.selected_nodes ^ new_selection
        self.selected_nodes = new_selection
        for n in changed:
            n.update()

    def toggle_node_selection(self, n):
        if n in self.selected_nodes:
            self.selected_nodes.discard(n)
            if getattr(self.parent, "selected_node", None) is n:
                self.set_selected_node(None)
        else:
            self.selected_nodes.add(n)
            self.set_selected_node(n)
        n.update()

    def clear_multi_selection(self):
        self.set_multi_selection(set())

    def _resolve_node_at(self, viewport_pos):
        # Topmost node under the cursor that is not faded out by the focus range,
        # so a click in a cluster reaches the focused node under a faded one
        for item in self.items(viewport_pos):
            if not isinstance(item, node.Node) and item is not None:
                item = item.parentItem()
            if isinstance(item, node.Node) and self.is_node_in_focus(item):
                return item
        return None

    def delete_selected_nodes(self):
        self.parent.delete_selected_node()

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
            node.delete_node(force=True)
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
        point.setX((point.x() / FIELD_IMAGE_IN + 0.5) * 2000)
        point.setY((point.y() / FIELD_IMAGE_IN + 0.5) * 2000)

        return point

    def mirror_nodes(self):
        for n in self.nodes:
            n.setPos(QPointF(n.x(), 2000 - n.y()))
            # n.turn = -n.turn
            # if (n.get_tangent() is not None):
            #     n.set_tangent(n.get_tangent() * -1)
        self.update_path()

    def load_nodes(self, node_str: str) -> None:
        action_names = self.config_manager.get_value("robot", "actions") or []
        try:
            route = route_serializer.parse_route(node_str, action_names)
        except route_serializer.RouteParseError as e:
            logger.error(f"Could not parse route data: {e}")
            QMessageBox.warning(
                self,
                "Invalid Route",
                f"The route cannot be loaded.\n\n{e}",
            )
            return

        self.clear_nodes()
        for rec in route.nodes:
            logger.info(f"Loading node record: {rec}")
            node = self.add_node(self.convert_point(QPointF(rec.x, rec.y)))
            node.is_start_node = rec.is_start
            if rec.is_start:
                self.start_node = node
            node.is_end_node = rec.is_end
            if rec.is_end:
                self.end_node = node
            node.is_reverse_node = rec.is_reverse
            node.stop_mode = rec.stop_mode
            node.turn = rec.turn
            node.spin_through = rec.spin_through
            node.wait_time = rec.wait_time
            node.max_velocity = rec.max_velocity
            node.max_acceleration = rec.max_acceleration
            node.incoming_magnitude = rec.incoming_magnitude
            node.outgoing_magnitude = rec.outgoing_magnitude

            if rec.tangent is not None:
                node.set_tangent(np.array(rec.tangent))
            node.set_tangent_derivative(
                np.array(rec.tangent_derivative)
                if rec.tangent_derivative is not None
                else None
            )

            node.apply_action_map(rec.actions)
            node.optimize = dict(rec.optimize)
            node.set_locked(rec.locked)
            # set_visible show()s or hide()s to match; a node hidden when the
            # route was saved must come back hidden, not unconditionally shown.
            node.set_visible(rec.visible)

        for rec in route.action_points:
            action_point = self.add_action_point(
                self.convert_point(QPointF(rec.x, rec.y)), rec.t
            )
            action_point.stop = rec.stop
            action_point.wait_time = rec.wait_time
            action_point.max_velocity = rec.max_velocity
            action_point.max_acceleration = rec.max_acceleration
            action_point.apply_action_map(rec.actions)

            action_point.show()

        self.keep_out = list(route.keep_out)
        self.update_path()

        # Migration/compatibility notes. Suppressed during undo/redo, where
        # the state string always came from convert_nodes and is current.
        if route.warnings and not self._restoring_state:
            message = "\n".join(f"- {w}" for w in route.warnings)
            logger.warning(f"Route loaded with notes:\n{message}")
            QMessageBox.information(self, "Route Loaded with Notes", message)

    def index_of(self, node: node.Node) -> int:
        return self.nodes.index(node)

    def auto_save(self):
        if self.parent.current_working_file is not None:
            self.parent.auto_save()

    def toggle_visualization(self, state: bool) -> None:
        logger.info(f"Visualizing toggled to : {state}")
        self.visualize = state
        self.update()

    # FOCUS RANGE
    def _focus_bounds(self):
        """(first, last) node index of the focus range, or None when there is none.
        A range whose nodes were deleted (or rebuilt by a load) clears itself."""
        if self.focus_from is None:
            return None
        if self.focus_from not in self.nodes:
            self.focus_from = self.focus_to = None
            return None
        if self.focus_to is not None and self.focus_to not in self.nodes:
            self.focus_to = None
        a = self.nodes.index(self.focus_from)
        b = len(self.nodes) - 1 if self.focus_to is None else self.nodes.index(self.focus_to)
        return (min(a, b), max(a, b))

    def _focus_mask(self, t_values):
        """Which spline parameters lie in the focus range; None without one."""
        bounds = self._focus_bounds()
        if bounds is None:
            return None
        t_values = np.asarray(t_values)
        return (t_values >= bounds[0] - 1e-9) & (t_values <= bounds[1] + 1e-9)

    def is_node_in_focus(self, n) -> bool:
        bounds = self._focus_bounds()
        if bounds is None or n not in self.nodes:
            return True
        return bounds[0] <= self.nodes.index(n) <= bounds[1]

    def is_param_in_focus(self, t: float) -> bool:
        bounds = self._focus_bounds()
        return bounds is None or bounds[0] - 1e-9 <= t <= bounds[1] + 1e-9

    def set_focus_from(self, n) -> None:
        """Focus from node n on: to the last node, or the existing end of the range."""
        self.focus_from = n
        logger.info(f"Focus range from node {self.nodes.index(n)}")
        self._focus_changed()

    def set_focus_to(self, n) -> None:
        """Focus up to node n: from the first node, or the existing start."""
        if self.focus_from is None or self.focus_from not in self.nodes:
            self.focus_from = self.nodes[0]
        self.focus_to = n
        logger.info(f"Focus range to node {self.nodes.index(n)}")
        self._focus_changed()

    def clear_focus(self) -> None:
        if self.focus_from is None:
            return
        self.focus_from = self.focus_to = None
        logger.info("Focus range cleared")
        self._focus_changed()

    def _focus_changed(self) -> None:
        # A selection outside the range would leave a hidden node editable
        for n in [n for n in self.selected_nodes if not self.is_node_in_focus(n)]:
            self.selected_nodes.discard(n)
            n.update()
        selected = getattr(self.parent, "selected_node", None)
        if selected is not None and not self.is_node_in_focus(selected):
            self.set_selected_node(None)
        self._update_drive_overlay()
        self.update_path()

    def _apply_focus(self) -> None:
        """Fade and disable the nodes / action points outside the focus range."""
        active = self._focus_bounds() is not None
        self.path_item.setOpacity(FOCUS_DIM_OPACITY if active else 1.0)
        items = [(n, self.is_node_in_focus(n)) for n in self.nodes]
        items += [(ap, self.is_param_in_focus(ap.t)) for ap in self.action_points]
        for item, inside in items:
            item.setOpacity(1.0 if inside else FOCUS_DIM_OPACITY)
            item.setAcceptedMouseButtons(
                Qt.MouseButton.AllButtons if inside else Qt.MouseButton.NoButton
            )
            item.setAcceptHoverEvents(inside)

    def set_drive_overlay_visible(self, visible: bool) -> None:
        """Show or hide the dashed spin-through drive line. Still kept up to date while
        hidden, so showing it again needs no regeneration."""
        logger.info(f"Spin-through drive line toggled to: {visible}")
        self.drive_overlay_item.setVisible(visible)

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
        fine_points = (fine_points / FIELD_IMAGE_FT + 0.5) * 2000

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
            self.robot_width * (2000 / FIELD_IMAGE_IN),
            self.robot_length * (2000 / FIELD_IMAGE_IN),
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
