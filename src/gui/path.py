import json
from math import sqrt

from PyQt6.QtCore import QLineF, QPointF, QSize, Qt
from PyQt6.QtGui import QColor, QMouseEvent, QPainter, QPainterPath, QPen, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsScene,
    QGraphicsView,
)

import utilities
from bezier import cubic_bezier, quadratic_bezier
from gui import node
from motion_profiling_v2 import motion_profile_generator


def create_curve_segments(start, end, control1, control2=None):
    numsegments = 100
    segments = [0]
    ox, oy = None, None
    if control2:
        ox, oy = cubic_bezier.cubic_bezier_point(start, control1, control2, end, 0)
    else:
        ox, oy = quadratic_bezier.quadratic_bezier_point(start, control1, end, 0)
    dx, dy = None, None
    currlen = 0
    for i in range(1, numsegments+1):
        t = (i) / (numsegments)
        cx, cy = None, None
        if control2:
            cx, cy = cubic_bezier.cubic_bezier_point(start, control1, control2, end, t)
        else:
            cx, cy = quadratic_bezier.quadratic_bezier_point(start, control1, end, t)

        dx, dy = ox - cx, oy - cy
        currlen += sqrt(dx**2 + dy**2)
        segments.append(currlen * (12.3266567842 / 2000))  #
        # print(segments[-1] - segments[-2])

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

        self.nodes = []
        self.start_node = None
        self.end_node = None

        self.path = None
        self.line_data = []

        self.path_item = QGraphicsPathItem()
        self.scene.addItem(self.path_item)

        self.setMinimumSize(size)

        self.zoom_factor = 1.0
        self.initial_fit = True

        self.mouseDown = False

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
        # old_pos = self.

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
        new_pos = self.mapToScene(event.position().toPoint())

        # Move scene to old position
        delta = new_pos - old_pos
        self.translate(delta.x(), delta.y())

        self.initial_fit = False

    def mousePressEvent(self, event: QMouseEvent):
        self.mouseDown = True

        if (
            event.button() == Qt.MouseButton.LeftButton
            and event.modifiers() & Qt.KeyboardModifier.ShiftModifier
        ):
            scene_pos = self.mapToScene(event.position().toPoint())
            node = self.add_node(scene_pos)

            print(f"Mouse clicked at ({scene_pos.x()}, {scene_pos.y()})")

            # Create a new Node instance
            self.scene.addItem(node)

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
        self.all_time_intervals = []
        self.all_positions = []
        self.all_velocities = []
        self.all_accelerations = []
        self.all_headings = []
        self.all_nodes_map = []
        self.all_coords = []

        current_position = 0
        segment_data = [[], []]
        segment_length = 0
        for i in range(0, len(self.line_data)):
            line = self.line_data[i][:]

            if len(line) == 3:
                segments = create_curve_segments(line[0], line[1], line[2])

            else:
                segments = create_curve_segments(line[0], line[1], line[2], line[3])

            segment_data[0].append(line)
            segment_data[1].append(segments)
            segment_length += segments[-1]

            if (not self.nodes[i + 1].is_end_node) and (
                not self.nodes[i + 1].has_action()
            ):
                continue
            (
                time_intervals,
                positions,
                velocities,
                accelerations,
                headings,
                nodes_map,
                coords,
            ) = motion_profile_generator.generate_motion_profile(
                [], segment_data[0], segment_data[1], v_max, a_max, j_max, track_width
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
            self.all_nodes_map.extend(
                (mapping + len(self.all_time_intervals) - len(time_intervals))
                for mapping in nodes_map
            )
            self.all_coords.extend(coords)

            current_position += segment_length
            segment_data = [[], []]
            segment_length = 0

        print("DIFF: ", current_position, " ", self.all_positions[-1], " ", current_position-self.all_positions[-1])
        print("END VELO: ", self.all_velocities[-1])
        self.all_nodes_map.append(len(self.all_time_intervals))

        return (
            self.all_time_intervals,
            self.all_positions,
            self.all_velocities,
            self.all_accelerations,
            self.all_headings,
            self.all_nodes_map,
            self.all_coords,
        )

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

    # Modified version of path generation logic from @musicamante on stackoverflow
    def build_path(self, points: list[QPointF]):
        # print("Building path...")
        factor = 0.25
        self.path = QPainterPath(points[0])
        self.line_data = []
        cp1 = None
        for p, current in enumerate(points[1:-1], 1):
            # previous segment
            source = QLineF(points[p - 1], current)
            # next segment
            target = QLineF(current, points[p + 1])

            targetAngle = target.angleTo(source)
            turnVal = self.nodes[p].turn
            if self.nodes[p].turn:
                angle = source.angle()
            elif self.nodes[p].is_reverse_node:
                if targetAngle > 180:
                    angle = (source.angle() + 90 + (targetAngle - 180) / 2) % 360
                else:
                    angle = (target.angle() - 90 + (targetAngle) / 2) % 360
            elif targetAngle > 180:
                angle = (source.angle() + source.angleTo(target) / 2) % 360
            else:
                angle = (target.angle() + target.angleTo(source) / 2) % 360

            if self.nodes[p].is_reverse_node:
                revTarget = QLineF.fromPolar(
                    source.length() * factor, angle
                ).translated(current)
            else:
                revTarget = QLineF.fromPolar(
                    source.length() * factor, angle + 180
                ).translated(current)

            cp2 = revTarget.p2()

            if p == 1:
                self.line_data.append([self.path.currentPosition(), current, cp2])
                self.path.quadTo(cp2, current)
            else:
                self.line_data.append([self.path.currentPosition(), current, cp1, cp2])
                self.path.cubicTo(cp1, cp2, current)
            revSource = QLineF.fromPolar(
                target.length() * factor, angle + turnVal
            ).translated(current)
            cp1 = revSource.p2()

        # The final curve, that joins to the last point
        if cp1 is None:
            return

        self.line_data.append([self.path.currentPosition(), points[-1], cp1])
        self.path.quadTo(cp1, points[-1])

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
        point.setX((point.x()/(12.3266567842*12) + 0.5) * 2000)
        point.setY((point.y()/(12.3266567842*12) + 0.5) * 2000)

        return point

    def load_nodes(self, node_str):
        nodes_data = json.loads(node_str)
        self.clear_nodes()
        for node_data in nodes_data:
            if len(node_data) > 4:
                node = self.add_node(self.convert_point(QPointF(node_data[0], node_data[1])))
                self.start_node = node if bool(node_data[2]) else self.start_node
                node.is_start_node = bool(node_data[2])
                self.end_node = node if bool(node_data[3]) else self.end_node
                node.is_end_node = bool(node_data[3])
                node.spin_intake = bool(node_data[4])
                node.clamp_goal = bool(node_data[5])
                node.is_reverse_node = bool(node_data[6])
                node.turn = node_data[7]
                node.wait_time = node_data[8]

                node.show()

        self.update_path()

    def index_of(self, node):
        return self.nodes.index(node)

    def auto_save(self):
        if self.parent.current_working_file is not None:
            self.parent.auto_save()
