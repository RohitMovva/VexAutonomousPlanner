from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPixmap, QPainter, QColor, QPen, QPainterPath
from PyQt6.QtCore import Qt, QLineF, QPointF, Qt
from math import sqrt
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *
from motion_profiling_v2 import motion_profile_generator
from utilities import *

def createCurveSegments(start, end, control1, control2=None):
    numsegments = 1001
    segments = [0]
    ox, oy = None, None
    if (control2):
        ox, oy = cubic_bezier_point(start, control1, control2, end, 0)
    else:
        ox, oy = quadratic_bezier_point(start, control1, end, 0)
    dx, dy = None, None
    currlen = 0
    for i in range(1, numsegments):
        t = (i+1)/numsegments
        cx, cy = None, None
        if (control2):
            cx, cy = cubic_bezier_point(start, control1, control2, end, t)
        else:
            cx, cy = quadratic_bezier_point(start, control1, end, t)
        dx, dy = ox-cx, oy-cy
        currlen += sqrt(dx**2 + dy**2)
        segments.append(currlen*(12/700))

        ox, oy = cx, cy
    return segments

class DrawingWidget(QWidget):
    def __init__(self, parent=None, image_path=resource_path('../assets/V5RC-HighStakes-Match-2000x2000.png')):
        super().__init__(parent)
        self.parent = parent
        self.setGeometry(0, 0, 700, 700)
        self.image = QPixmap(image_path) if image_path else None
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

        self.all_time_intervals = []
        self.all_positions = []
        self.all_velocities = []
        self.all_accelerations = []
        self.all_headings = []
        self.all_nodes_map = [] # Represents index of node n in any of the above lists

        self.path = None
        self.line_data = []

    def update_image_path(self, new_path):
        self.image = QPixmap(new_path)
        self.update()
    
    def calculateScurveStuff(self, v_max=20, a_max=8, j_max=45):
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

            if ((not self.parent.nodes[i+1].isEndNode) and (not self.parent.nodes[i+1].hasAction)):
                continue

            time_intervals, positions, velocities, accelerations, headings, nodes_map = motion_profile_generator.generate_motion_profile([], segment_data[0], segment_data[1], v_max, a_max, j_max)

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

    def paintEvent(self, event):
        gui_instance = self.parent

        painter = QPainter(self)
        painter.drawPixmap(self.rect(), self.image)

        if gui_instance.start_node and gui_instance.end_node and len(gui_instance.nodes) > 1:
            points = [QPointF(gui_instance.start_node.x, gui_instance.start_node.y)]
            for node in gui_instance.nodes:
                if node.isEndNode or node.isStartNode:
                    continue
                points.append(QPointF(node.x, node.y))
            points.append(QPointF(gui_instance.end_node.x, gui_instance.end_node.y))
            self.buildPath(points)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            pen = QPen(QColor("black"), 2)
            painter.setPen(pen)
            painter.drawPath(self.path)
            painter.end()

    # From @musicamante on stackoverflow rewrite later
    def buildPath(self, points):
        factor = 0.25
        self.path = QPainterPath(points[0])
        self.line_data = []
        # self.control_points = []
        cp1 = None
        for p, current in enumerate(points[1:-1], 1):
            # previous segment
            source = QLineF(points[p - 1], current)
            # next segment
            target = QLineF(current, points[p + 1])
            targetAngle = target.angleTo(source)
            if targetAngle > 180:
                angle = (source.angle() + source.angleTo(target) / 2) % 360
            else:
                angle = (target.angle() + target.angleTo(source) / 2) % 360

            revTarget = QLineF.fromPolar(source.length() * factor, angle + 180).translated(current)
            cp2 = revTarget.p2()

            if p == 1:
                self.line_data.append([self.path.currentPosition(), current, cp2])
                
                self.path.quadTo(cp2, current)
            else:
                self.line_data.append([self.path.currentPosition(), current, cp1, cp2])
                self.path.cubicTo(cp1, cp2, current)
            revSource = QLineF.fromPolar(target.length() * factor, angle).translated(current)
            cp1 = revSource.p2()

        # The final curve, that joins to the last point
        if (cp1 == None):
            return
        self.line_data.append([self.path.currentPosition(), points[-1], cp1])
        self.path.quadTo(cp1, points[-1])
