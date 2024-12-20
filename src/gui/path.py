import json
import math
from math import sqrt
from typing import List
import numpy as np
from splines.natural_cubic_spline import NaturalCubicSpline

from PyQt6.QtCore import QLineF, QPointF, QSize, Qt, QSizeF, QRectF
from PyQt6.QtGui import QColor, QMouseEvent, QPainter, QPainterPath, QPen, QPixmap, QVector2D, QTransform, QBrush
from PyQt6.QtWidgets import (
    QApplication,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
    QGraphicsRectItem
)

import utilities
from bezier import cubic_bezier, quadratic_bezier
from gui import node
from motion_profiling_v2 import motion_profile_generator


class StyledRectItem(QGraphicsRectItem):
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
        fill_color.setAlpha(40)  # Very transparent fill (0-255)
        self.setBrush(QBrush(fill_color))


def create_curve_segments(start, end, control1, control2=None) -> List[float]:
    numsegments = 250
    segments = [0]
    ox, oy = None, None
    if control2:
        ox, oy = cubic_bezier.cubic_bezier_point(start, control1, control2, end, 0)
    else:
        ox, oy = quadratic_bezier.quadratic_bezier_point(start, control1, end, 0)
    dx, dy = None, None
    currlen = 0
    for i in range(1, numsegments + 1):
        t = (i) / (numsegments)
        cx, cy = None, None
        if control2:
            cx, cy = cubic_bezier.cubic_bezier_point(start, control1, control2, end, t)
        else:
            cx, cy = quadratic_bezier.quadratic_bezier_point(start, control1, end, t)

        dx, dy = ox - cx, oy - cy
        currlen += sqrt(dx**2 + dy**2)
        segments.append(currlen * (12.3266567842 / 2000))

        ox, oy = cx, cy
    return segments


class PathWidget(QGraphicsView):
    def __init__(
        self,
        parent=None,
        image_path=utilities.resource_path(
            "../assets/V5RC-HighStakes-Match-2000x2000.png"
        ),
        size=QSize(250, 250),
    ):
        super().__init__(parent)
        self.parent = parent
        # self.setGeometry(0, 0, 700, 700)
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

        self.all_time_intervals = []
        self.all_positions = []
        self.all_velocities = []
        self.all_accelerations = []
        self.all_headings = []
        self.all_nodes_map = []  # Represents index of node n in any of the above lists

        self.nodes: List[node.Node] = []
        self.start_node = None
        self.end_node = None

        self.path = None
        self.line_data: List[List[float]] = []

        self.path_item = QGraphicsPathItem()
        self.scene.addItem(self.path_item)

        self.rect_item = StyledRectItem()
        self.scene.addItem(self.rect_item)

        self.setMinimumSize(size)

        self.zoom_factor = 1.0
        self.initial_fit = True

        self.mouseDown = False

        self.visualize = False
        
        self.path = QPainterPath()
        self.spline_points = []
        self.steps = 50  # Number of interpolation points between control points
        

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
        ):
            scene_pos = self.mapToScene(event.position().toPoint())
            self.add_node(scene_pos)

            print(f"Mouse clicked at ({scene_pos.x()}, {scene_pos.y()})")

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
            if (self.visualize):
                self.draw_rect(scene_pos)
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

    def handle_coords(self, point: QPointF):
        x = round(((point.x() / (2000)) - 0.5) * 12**2, 2)
        y = round(((point.y() / (2000)) - 0.5) * 12**2, 2)
        if not point.isNull():
            self.labelCoords.setText(f"{x}, {y}")
        else:
            self.labelCoords.clear()

    def update_image_path(self, new_path: str):
        self.image = QPixmap(new_path)
        self.image_item.setPixmap(self.image)
        self.update()

    def generate_motion_profile_lists(
        self, v_max: float, a_max: float, j_max: float, track_width: float
    ):
        # Clear lists
        self.all_time_intervals: List[float] = []
        self.all_positions: List[float] = []
        self.all_velocities: List[float] = []
        self.all_accelerations: List[float] = []
        self.all_headings: List[float] = []
        self.all_angular_velocities: List[float] = []
        self.all_nodes_map: List[int] = []
        self.all_coords: List[float] = []

        current_position: int = 0
        segment_data: List[List[List[float]]] = [[], []]
        segment_length: int = 0
        turn_values: List[float] = []
        reverse_values: List[bool] = []
        wait_times: List[float] = []
        is_reversed: bool = False
        
        for i in range(0, len(self.line_data)):
            if (self.nodes[i].is_reverse_node):
                is_reversed = not is_reversed
            line = self.line_data[i][:]

            if len(line) == 3:
                segments = create_curve_segments(line[0], line[1], line[2])

            else:
                segments = create_curve_segments(line[0], line[1], line[2], line[3])

            segment_data[0].append(line)
            segment_data[1].append(segments)
            segment_length += segments[-1]

            turn_values.append(self.nodes[i].turn)
            reverse_values.append(is_reversed)
            wait_times.append(self.nodes[i].wait_time)

            if (not self.nodes[i + 1].is_end_node) and (
                not self.nodes[i + 1].is_stop_node()
            ):
                continue
            (
                time_intervals,
                positions,
                velocities,
                accelerations,
                headings,
                angular_velocities,
                nodes_map,
                coords,
            ) = motion_profile_generator.generate_motion_profile(
                [], segment_data[0], segment_data[1], v_max, a_max, j_max, track_width, turn_values, reverse_values, wait_times
            )

            if self.all_time_intervals != []:
                self.all_time_intervals.extend(
                    [time + self.all_time_intervals[-1] for time in time_intervals]
                )
            else:
                self.all_time_intervals = time_intervals
            self.all_positions.extend([p + current_position for p in positions])
            self.all_velocities.extend(velocities)
            self.all_accelerations.extend(accelerations)
            self.all_headings.extend(headings)
            self.all_angular_velocities.extend(angular_velocities)
            self.all_nodes_map.extend(
                (mapping + len(self.all_time_intervals) - len(time_intervals))
                for mapping in nodes_map
            )
            self.all_coords.extend(coords)

            current_position += segment_length
            segment_data = [[], []]
            turn_values = []
            wait_times = []
            reverse_values = []
            segment_length = 0


        print("Accumulated value", self.all_positions[-1])
        print("Goal value:", current_position)
        print("Error:", current_position-self.all_positions[-1])
        self.all_nodes_map.append(len(self.all_time_intervals))

        return (
            self.all_time_intervals,
            self.all_positions,
            self.all_velocities,
            self.all_accelerations,
            self.all_headings,
            self.all_angular_velocities,
            self.all_nodes_map,
            self.all_coords,
        )
    
    def compute_spline_coefficients(self, x: np.ndarray, y: np.ndarray):
        """Compute natural cubic spline coefficients for a set of points"""
        n = len(x)
        if n < 3:
            return None
            
        # Build the tridiagonal system for the second derivatives
        h = np.diff(x)  # Intervals between x points
        
        # Build the tridiagonal matrix A
        A = np.zeros((n, n))
        r = np.zeros(n)
        
        # Interior points
        for i in range(1, n-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            
            r[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])
        
        # Boundary conditions for natural spline (second derivatives = 0 at endpoints)
        A[0, 0] = 1
        A[-1, -1] = 1
        
        # Solve for second derivatives
        m = np.linalg.solve(A, r)
        
        # Calculate coefficients for each segment
        coeffs = {'a': np.zeros(n-1), 'b': np.zeros(n-1), 
                 'c': np.zeros(n-1), 'd': np.zeros(n-1)}
                 
        for i in range(n-1):
            coeffs['a'][i] = y[i]
            coeffs['b'][i] = (y[i+1] - y[i]) / h[i] - h[i] * (2 * m[i] + m[i+1]) / 3
            coeffs['c'][i] = m[i]
            coeffs['d'][i] = (m[i+1] - m[i]) / (3 * h[i])
            
        return coeffs
        
    def evaluate_spline_segment(self, t: np.ndarray, coeffs: dict, i: int, x0: float) -> np.ndarray:
        """Evaluate the spline segment i at points t"""
        t_norm = t - x0
        return (coeffs['a'][i] + 
                coeffs['b'][i] * t_norm + 
                coeffs['c'][i] * t_norm**2 + 
                coeffs['d'][i] * t_norm**3)

    def build_path(self, points: List[QPointF], nodes=None):
        # Convert points to numpy array to use in build path function
        points = np.array([[point.x(), point.y()] for point in points])
        print("Points:", points)
        # Create a natural cubic spline
        spline = NaturalCubicSpline()
        path = spline.build_path(points)

        # Create a QPainterPath to draw the spline
        self.path = QPainterPath()
        self.path.moveTo(points[0][0], points[0][1])
        for i in range(1, len(points)):
            self.path.lineTo(points[i][0], points[i][1])

        
        return path
    
    def update_path(self):
        # Should update with any new, moved, modified, or removed nodes
        if self.start_node and self.end_node and len(self.nodes) > 1:
            points = [self.start_node.pos()]
            for node in self.nodes:
                if node.is_end_node or node.is_start_node:
                    continue
                points.append(node.pos())
            points.append(self.end_node.pos())
            self.build_path(points)

        else:
            self.path = QPainterPath()

        pen = QPen(QColor("#0a0612"), 4)  # dark purple (looks cool)
        self.path_item.setPen(pen)
        self.path_item.setPath(self.path)

        self.viewport().update()  # Request a repaint of the viewport

    def save(self):
        self.parent.auto_save()

    # # Modified version of path generation logic from @musicamante on stackoverflow
    # def build_path(self, points: list[QPointF]):
    #     # print("Building path...")
    #     factor = 0.25
    #     self.path.clear()
    #     self.path = QPainterPath(points[0])
    #     self.line_data: List[List[float]] = []
    #     cp1 = None
    #     for p, current in enumerate(points[1:-1], 1):
    #         # previous segment
    #         source = QLineF(points[p - 1], current)
    #         # next segment
    #         target = QLineF(current, points[p + 1])

    #         targetAngle = target.angleTo(source)
    #         turnVal = (self.nodes[p].turn)
    #         if (self.nodes[p].turn and self.nodes[p].is_reverse_node):
    #             angle = source.angle() + 180
    #         elif self.nodes[p].turn:
    #             angle = source.angle()
    #         elif self.nodes[p].is_reverse_node:
    #             angle = (target.angle() - 90 + (targetAngle) / 2) % 360
    #         elif targetAngle > 180:
    #             angle = (source.angle() + source.angleTo(target) / 2) % 360
    #         else:
    #             angle = (target.angle() + target.angleTo(source) / 2) % 360

    #         if self.nodes[p].is_reverse_node:
    #             revTarget = QLineF.fromPolar(
    #                 source.length() * factor, angle
    #             ).translated(current)
    #         else:
    #             revTarget = QLineF.fromPolar(
    #                 source.length() * factor, angle + 180
    #             ).translated(current)

    #         cp2 = revTarget.p2()

    #         if p == 1:
    #             self.line_data.append([self.path.currentPosition(), current, cp2])
    #             self.path.quadTo(cp2, current)
    #         else:
    #             self.line_data.append([self.path.currentPosition(), current, cp1, cp2])
    #             self.path.cubicTo(cp1, cp2, current)
    #         revSource = QLineF.fromPolar(
    #             target.length() * factor, angle + turnVal
    #         ).translated(current)
    #         cp1 = revSource.p2()

    #     # The final curve, that joins to the last point
    #     if cp1 is None:
    #         return

    #     self.line_data.append([self.path.currentPosition(), points[-1], cp1])
    #     self.path.quadTo(cp1, points[-1])

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
        self.auto_save()

        print(f"Node created at ({new_node.abs_x}, {new_node.abs_y})")
        return new_node

    def remove_node(self, remove_node):
        if remove_node in self.nodes:
            self.nodes.remove(remove_node)
            if remove_node == self.start_node:
                self.start_node = None
            if remove_node == self.end_node:
                self.end_node = None

        self.update_path()

    def set_start_node(self, new_start_node):
        if self.start_node:
            self.start_node.is_start_node = False
            self.start_node.update()
            self.update()
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

    def set_end_node(self, node):
        if self.end_node:
            self.end_node.is_end_node = False
            self.end_node.update()
            self.update()
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
        self.start_node = None
        self.end_node = None
        self.clearing_nodes = False

        self.update_path()
        self.update()
        self.auto_save()

    def convert_point(self, point: QPointF):
        point.setX((point.x() / (12.3266567842 * 12) + 0.5) * 2000)
        point.setY((point.y() / (12.3266567842 * 12) + 0.5) * 2000)

        return point
    
    def mirror_nodes(self):
        for n in self.nodes:
            n.setPos(QPointF(2000 - n.x(), n.y()))
            n.turn = -n.turn
        self.update_path()

    def load_nodes(self, node_str: str) -> None:
        nodes_data = json.loads(node_str)
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
                node.spin_intake = (node_data[4])
                node.clamp_goal = bool(node_data[5])
                node.doink = bool(node_data[6])
                node.is_reverse_node = bool(node_data[7])
                node.stop = bool(node_data[8])
                node.turn = node_data[9]
                node.lb = node_data[10]
                node.wait_time = node_data[11]

                node.show()

        self.update_path()

    def index_of(self, node: node.Node) -> int:
        return self.nodes.index(node)

    def auto_save(self):
        if self.parent.current_working_file is not None:
            self.parent.auto_save()

    def toggle_visualization(self, state: bool) -> None:
        print("Visualizing:", state)
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
        if path is None:
            return QPointF()
        # Get the length of the path
        path_length = path.length()
        if path_length == 0:
            return QPointF()
        
        # Binary search parameters
        min_dist = float('inf')
        closest_point = QPointF()
        closest_percent = 0.0
        
        # First pass: coarse search with larger steps
        num_steps = 100
        for i in range(num_steps + 1):
            percent = i / num_steps
            path_point = path.pointAtPercent(percent)
            dist = math.hypot(path_point.x() - point.x(), path_point.y() - point.y())
            
            if dist < min_dist:
                min_dist = dist
                closest_point = path_point
                closest_percent = percent
        
        # Second pass: fine search around the closest point found
        # Search within ±2% of the closest point found
        search_range = 0.02
        start_percent = max(0.0, closest_percent - search_range)
        end_percent = min(1.0, closest_percent + search_range)
        
        fine_steps = 20
        percent_step = (end_percent - start_percent) / fine_steps
        
        for i in range(fine_steps + 1):
            percent = start_percent + (i * percent_step)
            path_point = path.pointAtPercent(percent)
            dist = math.hypot(path_point.x() - point.x(), path_point.y() - point.y())
            
            if dist < min_dist:
                min_dist = dist
                closest_point = path_point
        
        return closest_point

    def find_path_angle_at_point(self, path: QPainterPath, point: QPointF, delta: float = 0.01) -> float:
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
        min_dist = float('inf')
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
        if not hasattr(self, 'rect_item'):
            self.rect_item = StyledRectItem()
            self.scene().addItem(self.rect_item)
        
        path_point = self.find_closest_point_on_path(self.path, pt)
        if path_point is None:
            self.rect_item.hide()
            return
        
        dist = QVector2D(pt - path_point).length()
        if dist > 100:
            self.rect_item.hide()
            return
        
        # Calculate angle at the path point
        angle = self.find_path_angle_at_point(self.path, path_point)
        
        # Create rectangle centered at path point
        rect_size = QSizeF(1.47916666667 * (2000 / 12.3266567842), 1.33333333333 * (2000 / 12.3266567842))
        center_rect = QRectF(
            path_point.x() - rect_size.width()/2,
            path_point.y() - rect_size.height()/2,
            rect_size.width(),
            rect_size.height()
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