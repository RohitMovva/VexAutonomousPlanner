from PyQt6.QtWidgets import QGraphicsPathItem, QGraphicsScene, QGraphicsView, QGraphicsPixmapItem, QApplication
from PyQt6.QtGui import QPixmap, QPainter, QColor, QPen, QPainterPath, QColor, QPixmap
from PyQt6.QtCore import Qt, QLineF, Qt, QSize, Qt
from math import sqrt
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *
from motion_profiling_v2 import motion_profile_generator
from gui.node import *
from utilities import *
import json

# def convertPoint(p: QPointF):
#     p.setX(((p.x() / (2000)) - 0.5) * 12.1622315686**2)
#     p.setY(((p.y() / (2000)) - 0.5) * 12.1622315686**2)

#     return p

def createCurveSegments(start, end, control1, control2=None):
    numsegments = 1001
    segments = [0]
    ox, oy = None, None
    # start = convertPoint(start)
    # end = convertPoint(end)
    # control1 = convertPoint(control1)
    # if (control2):

    #     control2 = convertPoint(control2)
    # start = convertPoint(start)
    if (control2):
        ox, oy = cubic_bezier_point(start, control1, control2, end, 0)
    else:
        ox, oy = quadratic_bezier_point(start, control1, end, 0)
    dx, dy = None, None
    currlen = 0
    for i in range(1, numsegments):
        t = (i)/numsegments
        cx, cy = None, None
        if (control2):
            cx, cy = cubic_bezier_point(start, control1, control2, end, t)
        else:
            cx, cy = quadratic_bezier_point(start, control1, end, t)

        # print("THINGYS: ", cx, cy, end)
        dx, dy = ox-cx, oy-cy
        currlen += sqrt(dx**2 + dy**2)
        segments.append(currlen * (12.3420663695/2000)) #

        ox, oy = cx, cy
    print("DIFFS: ", ox, " ",oy)
    # print("SEGMENT LEN THING: ", segments[-1])
    return segments

class PathWidget(QGraphicsView):
    def __init__(self, parent=None, image_path=resource_path('../assets/V5RC-HighStakes-Match-2000x2000.png'), size=QSize(250, 250)):
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
        self.all_nodes_map = [] # Represents index of node n in any of the above lists

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

    def fit_image_to_view(self):
        self.fitInView(self.image_item, Qt.AspectRatioMode.KeepAspectRatio)
        if self.initial_fit:
            # Scale up the view to fill the widget as much as possible
            current_rect = self.mapToScene(self.viewport().rect()).boundingRect()
            image_rect = self.image_item.boundingRect()
            self.zoom_factor = min(current_rect.width() / image_rect.width(),
                                   current_rect.height() / image_rect.height())
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
        if (event.modifiers() & Qt.KeyboardModifier.ShiftModifier):
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
        if event.button() == Qt.MouseButton.LeftButton and event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            scene_pos = self.mapToScene(event.position().toPoint())
            node = self.add_node(scene_pos)

            print(f"Mouse clicked at ({scene_pos.x()}, {scene_pos.y()})")
            
            # Create a new Node instance
            self.scene.addItem(node)
        
        # Call the parent class mousePressEvent to maintain drag functionality
        else:
            # Call the parent class mousePressEvent for other cases (like panning)
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        item = self.itemAt(event.position().toPoint())
        scene_pos = self.mapToScene(event.position().toPoint())
        if isinstance(item, Node) or (item and isinstance(item.parentItem(), Node)):
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
            self.parent.updateCoords(scene_pos)
        elif self.shift_pressed:
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
            self.parent.updateCoords(scene_pos)
        elif (scene_pos.x() < 0 or scene_pos.x() > 2000 or scene_pos.y() < 0 or scene_pos.y() > 2000):
            QApplication.setOverrideCursor(Qt.CursorShape.ArrowCursor)
        else:
            QApplication.setOverrideCursor(Qt.CursorShape.OpenHandCursor)
            self.parent.updateCoords(scene_pos)

        super().mouseMoveEvent(event)

    def leaveEvent(self, QEvent):
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

    def handleCoords(self, point):
        x = (round(((point.x() / (2000)) - 0.5) * 12**2, 2))
        y = (round(((point.y() / (2000)) - 0.5) * 12**2, 2))
        if not point.isNull():
            self.labelCoords.setText(f'{x}, {y}')
        else:
            self.labelCoords.clear()

    def update_image_path(self, new_path):
        self.image = QPixmap(new_path)
        self.image_item.setPixmap(self.image)
        self.update()
    
    def calculateScurveStuff(self, v_max, a_max, j_max, track_width):
        # Clear lists
        self.all_time_intervals = []
        self.all_positions = []
        self.all_velocities = []
        self.all_accelerations = []
        self.all_headings = []
        self.all_nodes_map = []

        current_position = 0
        segment_data = [[], []]
        segment_length = 0
        for i in range (0, len(self.line_data)):
            line = self.line_data[i][:]

            if (len(line) == 3):
                segments = createCurveSegments(line[0], line[1], line[2])

            else:
                segments = createCurveSegments(line[0], line[1], line[2], line[3])

            segment_data[0].append(line)
            segment_data[1].append(segments)
            segment_length += segments[-1]

            if ((not self.nodes[i+1].isEndNode) and (not self.nodes[i+1].has_action())):
                continue
            print("SEGMENT LENGTH: ", segment_length)
            time_intervals, positions, velocities, accelerations, headings, nodes_map = motion_profile_generator.generate_motion_profile([], segment_data[0], segment_data[1], v_max, a_max, j_max, track_width)

            if (self.all_time_intervals != []):
                self.all_time_intervals.extend([time + self.all_time_intervals[-1] for time in time_intervals])
            else:
                self.all_time_intervals = time_intervals
            self.all_positions.extend([p + current_position for p in positions])
            self.all_velocities.extend(velocities)
            self.all_accelerations.extend(accelerations)
            self.all_headings.extend(headings)
            self.all_nodes_map.extend((mapping+len(self.all_time_intervals)-len(time_intervals)) for mapping in nodes_map)

            current_position += segment_length
            segment_data = [[], []]
            segment_length = 0

        self.all_nodes_map.append(len(self.all_time_intervals))
        return self.all_time_intervals, self.all_positions, self.all_velocities, self.all_accelerations, self.all_headings, self.all_nodes_map

    def update_path(self):
        # This method should update self.path based on self.nodes, self.start_node, and self.end_node
        # Implement your path building logic here
        if self.start_node and self.end_node and len(self.nodes) > 1:
            points = [self.start_node.pos()]
            for node in self.nodes:
                if node.isEndNode or node.isStartNode:
                    continue
                points.append(node.pos())
            points.append(self.end_node.pos())
            self.buildPath(points)

            pen = QPen(QColor("black"), 3)
            self.path_item.setPen(pen)
            self.path_item.setPath(self.path)
        else:
            self.path = QPainterPath()

            pen = QPen(QColor("black"), 3)
            self.path_item.setPen(pen)
            self.path_item.setPath(self.path)
            
        self.viewport().update()  # Request a repaint of the viewport

    # Modified version of path generation logic from @musicamante on stackoverflow
    def buildPath(self, points):
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
            if (self.nodes[p].turn):
                angle = source.angle()
            elif (self.nodes[p].isReverseNode):
                if targetAngle > 180:
                    angle = (source.angle() + 90 + (targetAngle - 180) / 2) % 360
                else:
                    angle = (target.angle() - 90 + (targetAngle) / 2) % 360
            elif targetAngle > 180:
                angle = (source.angle() + source.angleTo(target) / 2) % 360
            else:
                angle = (target.angle() + target.angleTo(source) / 2) % 360

            if (self.nodes[p].isReverseNode):
                revTarget = QLineF.fromPolar(source.length() * factor, angle).translated(current)
            else:
                revTarget = QLineF.fromPolar(source.length() * factor, angle + 180).translated(current)

            cp2 = revTarget.p2()
            
            if p == 1:
                self.line_data.append([self.path.currentPosition(), current, cp2])
                self.path.quadTo(cp2, current)
            else:
                self.line_data.append([self.path.currentPosition(), current, cp1, cp2])
                self.path.cubicTo(cp1, cp2, current)
            revSource = QLineF.fromPolar(target.length() * factor, angle + turnVal).translated(current)
            cp1 = revSource.p2()

        # The final curve, that joins to the last point
        if (cp1 == None):
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

    def add_node(self, point, pos=-1):
        node = Node(point.x(), point.y(), self)
        if (pos == -1 and self.end_node != None):
            pos = len(self.nodes)-1
        if (pos == -1):
                self.nodes.append(node)
        else:
            self.nodes.insert(pos, node)

        node.setPos(point)
        self.scene.addItem(node)

        node.show()
        self.update_path()
        self.auto_save()

        print(f"Node created at ({node.absX}, {node.absY})")
        return node
    
    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)
            if node == self.start_node:
                self.start_node = None
            if node == self.end_node:
                self.end_node = None

        self.update_path()
    
    def set_start_node(self, node):
        if self.start_node:
            self.start_node.isStartNode = False
            self.start_node.update()
            self.update()
        self.start_node = node

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
            self.end_node.isEndNode = False
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

    def load_nodes(self, str):
        nodes_data = json.loads(str)
        self.clear_nodes()
        for node_data in nodes_data:
            if (len(node_data) > 4):
                node = self.add_node(QPointF(node_data[0], node_data[1]))
                self.start_node = node if bool(node_data[2]) else self.start_node
                node.isStartNode = bool(node_data[2])
                self.end_node = node if bool(node_data[3]) else self.end_node
                node.isEndNode = bool(node_data[3])
                node.spinIntake = bool(node_data[4])
                node.clampGoal = bool(node_data[5])
                node.isReverseNode = bool(node_data[6])
                node.turn = node_data[7]
                node.wait_time = node_data[8]

                node.show()

        self.update_path()

    def index_of(self, node):
        return (self.nodes.index(node))
    
    def auto_save(self):
        if (self.parent.current_working_file != None):
            self.parent.auto_save()