import json
import logging
import math
from typing import List

import numpy as np
from PyQt6.QtCore import QTimer, QPointF, QRectF, QSize, QSizeF, Qt
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QMouseEvent,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QTransform,
    QVector2D,
)
from PyQt6.QtWidgets import (
    QApplication,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsView,
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
            self.add_node(scene_pos)

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
            self.add_action_point(path_point, path_param)

            logger.info(f"Added action point at ({path_point.x()}, {path_point.y()})")

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

    def generate_motion_profile_lists(
        self, v_max: float, a_max: float, j_max: float, track_width: float
    ):
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
            max_vel=v_max,  # Maximum velocity in meters/second
            max_acc=a_max,  # Maximum acceleration in meters/second^2
            max_dec=a_max,  # Maximum deceleration in meters/second^2
            friction_coef=0.8,  # Coefficient of friction
            max_jerk=j_max,  # Maximum jerk (rate of change of acceleration) in meters/second^3
            track_width=track_width,  # Distance between wheels in meters
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
        self.nodes_map.append(len(self.time_intervals))

        return (
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

    def update_spline(
        self,
        points: List[QPointF],
        nodes: List[node.Node],
        action_points: List[action_point.ActionPoint],
    ):
        # convert points to numpy array of floats and from pixels to inches
        points = np.array([[point.x(), point.y()] for point in points])

        for i in range(len(points)):
            points[i][0] = (points[i][0] / (2000) - 0.5) * 12.1090395251
            points[i][1] = (points[i][1] / (2000) - 0.5) * 12.1090395251

        self.spline_manager.build_path(points, nodes, action_points)
        t_values = np.linspace(0, len(points) - 1, 25 * len(self.nodes))
        spline_points = np.array(
            [self.spline_manager.get_point_at_parameter(t) for t in t_values]
        )

        if len(spline_points) > 0:
            self.path = QPainterPath()
            for i in range(len(spline_points)):
                spline_points[i][0] = (
                    spline_points[i][0] / (12.1090395251) + 0.5
                ) * 2000
                spline_points[i][1] = (
                    spline_points[i][1] / (12.1090395251) + 0.5
                ) * 2000
            self.path.moveTo(spline_points[0][0], spline_points[0][1])
            for p in spline_points[1:]:
                self.path.lineTo(p[0], p[1])
        else:
            self.path = QPainterPath()

        return spline_points

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

        for action_node in self.action_points:
            t = action_node.t
            point = self.spline_manager.get_point_at_parameter(t)
            point = (point / (12.1090395251) + 0.5) * 2000
            action_node.setPos(QPointF(point[0], point[1]))

        pen = QPen(QColor("#0a0612"), 4)  # dark purple (looks cool)
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
            if i == len(self.action_points)-1:
                self.action_points.append(new_action_point)

        self.scene.addItem(new_action_point)
        new_action_point.show()

        logger.info(
            f"Action point created at ({new_action_point.abs_x}, {new_action_point.abs_y})"
        )
        return new_action_point

    def remove_node(self, remove_node):
        if remove_node in self.nodes:
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
        if (self.spline_manager.splines):
            return self.spline_manager.get_derivative_at_parameter(self.nodes.index(node))
        else:
            return [0, 0]

    def get_incoming_magnitude_at_node(self, node):
        if (self.spline_manager.splines):
            return self.spline_manager.get_magnitudes_at_parameter(self.nodes.index(node))[0]
        else:
            return 0
        
    def get_outgoing_magnitude_at_node(self, node):
        if (self.spline_manager.splines):
            return self.spline_manager.get_magnitudes_at_parameter(self.nodes.index(node))[1]
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
        while self.nodes:
            node = self.nodes.pop()
            node.delete_node()

        while self.action_points:
            action_point = self.action_points.pop()
            action_point.delete_action_point()

        self.start_node = None
        self.end_node = None
        self.clearing_nodes = False

        self.update_path()
        self.update()
        # self.auto_save()

    def convert_point(self, point: QPointF):
        point.setX((point.x() / (145.308474301) + 0.5) * 2000)
        point.setY((point.y() / (145.308474301) + 0.5) * 2000)

        return point

    def mirror_nodes(self):
        for n in self.nodes:
            n.setPos(QPointF(2000 - n.x(), n.y()))
            n.turn = -n.turn
        self.update_path()

    def load_nodes(self, node_str: str) -> None:
        data = json.loads(node_str)
        if (len(data) == 2):
            nodes_data = data[0]
            action_data = data[1]
        else:
            nodes_data = data
            action_data = []
        self.clear_nodes()
        for node_data in nodes_data:
            if len(node_data) > 4:
                node = self.add_node(
                    self.convert_point(QPointF(node_data[0], node_data[1]))
                )
                self.start_node = node if bool(node_data[2]) else self.start_node
                node.is_start_node = bool(node_data[2])
                self.end_node = node if bool(node_data[3]) else self.end_node
                node.is_end_node = bool(node_data[3])
                node.is_reverse_node = bool(node_data[4])
                node.stop = bool(node_data[5])
                node.turn = node_data[6]
                node.wait_time = node_data[7]
                node.tangent = node_data[8]
                node.incoming_magnitude = (node_data[9])
                node.outgoing_magnitude = node_data[10]
                node.set_action_values(node_data[11:])

                if (node_data[8] is not None):
                    node.set_tangent(np.array(node_data[8]))
                node.show()

                

        for action_data in action_data:
            action_point = self.add_action_point(
                self.convert_point(QPointF(action_data[0], action_data[1])),
                action_data[2],
            )
            action_point.stop = action_data[3]
            action_point.wait_time = action_data[4]
            action_point.set_action_values(action_data[5:])

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

    def find_closest_point_on_path(self, path: QPainterPath, point: QPointF) -> QPointF:
        """
        Find the closest point on a QPainterPath to a given point.

        Args:
            path (QPainterPath): The path to search on
            point (QPointF): The reference point to find the closest point to

        Returns:
            QPointF: The closest point on the path
        """
        logger.info(f"Finding closest point on path to ({point.x()}, {point.y()})")
        if path.isEmpty():
            logger.info("Path is None")
            return QPointF(), None

        # Get the length of the path
        path_length = path.length()
        if path_length == 0:
            logger.info("Path length is 0")
            return QPointF(), None

        # Binary search parameters
        min_dist = float("inf")
        closest_point = np.ndarray([0, 0])
        closest_percent = 0.0
        closest_parameter = 0.0

        # Convert point to numpy array w/ inches
        point = np.array([point.x(), point.y()])
        point = (point / (2000) - 0.5) * 12.1090395251

        # First pass: coarse search with larger steps
        num_steps = 25 * len(self.nodes)
        for i in range(num_steps + 1):
            percent = i / num_steps
            path_param = self.spline_manager.percent_to_parameter(percent)
            path_point = self.spline_manager.get_point_at_parameter(path_param)
            dist = math.hypot(path_point[0] - point[0], path_point[1] - point[1])

            if dist < min_dist:
                min_dist = dist
                closest_point = path_point
                closest_percent = percent
                closest_parameter = path_param

        # Second pass: fine search around the closest point found
        # Search within ±2% of the closest point found
        search_range = 0.02
        start_percent = max(0.0, closest_percent - search_range)
        end_percent = min(1.0, closest_percent + search_range)

        fine_steps = 500
        percent_step = (end_percent - start_percent) / fine_steps

        for i in range(fine_steps + 1):
            percent = start_percent + (i * percent_step)
            path_param = self.spline_manager.percent_to_parameter(percent)
            path_point = self.spline_manager.get_point_at_parameter(path_param)
            dist = math.hypot(path_point[0] - point[0], path_point[1] - point[1])

            if dist < min_dist:
                min_dist = dist
                closest_point = path_point
                closest_parameter = path_param

        # Convert closest point to QPointF with pixels
        closest_point = (closest_point / (12.1090395251) + 0.5) * 2000

        return QPointF(closest_point[0], closest_point[1]), closest_parameter

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
        # Find the percentage along the path for the given point
        min_dist = float("inf")
        closest_percent = 0.0

        for i in range(101):
            percent = i / 100
            path_point = path.pointAtPercent(percent)
            dist = QVector2D(point - path_point).length()
            if dist < min_dist:
                min_dist = dist
                closest_percent = percent

        # Get points slightly before and after to calculate tangent
        p1 = path.pointAtPercent(max(0, closest_percent - delta))
        p2 = path.pointAtPercent(min(1, closest_percent + delta))

        # Calculate angle from vector between points
        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        return math.atan2(dy, dx)

    def draw_rect(self, pt):
        # Create the styled rect item if it doesn't exist
        if not hasattr(self, "rect_item"):
            self.rect_item = RobotVisualizer()
            self.scene().addItem(self.rect_item)

        path_point, _ = self.find_closest_point_on_path(self.path, pt)
        if path_point is None:
            self.rect_item.hide()
            return
        if (path_point == QPointF(0, 0)):
            self.rect_item.hide()
            return

        dist = QVector2D(pt - path_point).length()
        if dist > 100:
            self.rect_item.hide()
            return

        # Calculate angle at the path point
        angle = self.find_path_angle_at_point(self.path, path_point)

        # Create rectangle centered at path point
        rect_size = QSizeF(
            self.robot_width * (2000 / (145.308474301)),
            self.robot_length * (2000 / (145.308474301)),
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
