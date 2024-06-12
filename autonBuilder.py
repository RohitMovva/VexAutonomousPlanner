import sys
import json
from PyQt6.QtWidgets import QApplication, QDialog, QLabel, QWidget, QVBoxLayout, QMenu, QMenuBar, QInputDialog, QMainWindow, QStackedLayout, QTextEdit, QPushButton
from PyQt6.QtGui import QPixmap, QMouseEvent, QPainter, QColor, QAction, QPen, QPainterPath
from PyQt6.QtCore import Qt, QPoint, QLineF, QPointF
from scipy.integrate import quad
import numpy as np
import matplotlib.pyplot as plt

# Calculations for length of Bezier curves (quadratic and cubic)
def quad_bezier(t, P0, P1, P2):
    return (1-t)**2 * P0 + 2 * (1-t) * t * P1 + t**2 * P2

# Derivative of the quadratic Bezier curve function
def quad_bezier_derivative(t, P0, P1, P2):
    return 2 * (1-t) * (P1 - P0) + 2 * t * (P2 - P1)

# Integrand for the quadratic arc length calculation
def quad_integrand(t, P0, P1, P2):
    dx_dt, dy_dt = quad_bezier_derivative(t, P0, P1, P2)
    return np.sqrt(dx_dt**2 + dy_dt**2)

# Quadratic Bezier curve function
def cubic_bezier(t, P0, P1, P2, P3):
    return (1-t)**3 * P0 + 3 * (1-t)**2 * t * P1 + 3 * (1-t) * t**2 * P2 + t**3 * P3

# Derivative of the cubic Bezier curve function
def cubic_bezier_derivative(t, P0, P1, P2, P3):
    return 3 * (1-t)**2 * (P1 - P0) + 6 * (1-t) * t * (P2 - P1) + 3 * t**2 * (P3 - P2)

# Integrand for the cubic arc length calculation
def cubic_integrand(t, P0, P1, P2, P3):
    dx_dt, dy_dt = cubic_bezier_derivative(t, P0, P1, P2, P3)
    return np.sqrt(dx_dt**2 + dy_dt**2)


# Click listener object
class ClickableLabel(QLabel):
    def __init__(self, parent=None, gui_instance=None):
        super().__init__(parent)
        self.gui_instance = gui_instance

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            x = int(event.position().x())
            y = int(event.position().y())
            print(f"Mouse clicked at ({x}, {y})")
            self.gui_instance.add_node(x, y)
        super().mousePressEvent(event)

# Node that stores data for auton route
class Node(QWidget):
    def __init__(self, x, y, parent=None, gui_instance=None):
        super().__init__(parent)
        self.x = x
        self.y = y
        self.gui_instance = gui_instance
        self.isStartNode = False
        self.isEndNode = False
        self.spinIntake = False
        self.clampGoal = False
        self.turn = 0
        self.setFixedSize(10, 10)
        self.move(x+5, y+5)
        self.dragging = False
        self.offset = QPoint(0, 0)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        if self.isStartNode:
            painter.setBrush(QColor("green"))
        elif self.isEndNode:
            painter.setBrush(QColor("red"))
        else:
            painter.setBrush(QColor("blue"))

        painter.drawEllipse(0,0,100,140)
        painter.drawEllipse(0, 0, self.width(), self.height())

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.RightButton:
            self.show_context_menu(event.globalPosition().toPoint())
        elif event.button() == Qt.MouseButton.LeftButton:
            self.dragging = True
            self.offset = event.position().toPoint()
        super().mousePressEvent(event)
        self.gui_instance.update_lines()

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.dragging:
            new_pos = self.mapToParent(event.position().toPoint() - self.offset)
            self.x = new_pos.x()-5
            self.y = new_pos.y()-5
            self.move(new_pos)
            self.gui_instance.update_lines()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = False
        super().mouseReleaseEvent(event)

    def show_context_menu(self, pos):
        context_menu = QMenu(self)

        start_action = QAction('Start Node', self, checkable=True)
        start_action.setChecked(self.isStartNode)
        start_action.triggered.connect(self.toggle_start_node)
        context_menu.addAction(start_action)

        end_action = QAction('End Node', self, checkable=True)
        end_action.setChecked(self.isEndNode)
        end_action.triggered.connect(self.toggle_end_node)
        context_menu.addAction(end_action)

        spin_action = QAction('Spin Intake', self, checkable=True)
        spin_action.setChecked(self.spinIntake)
        spin_action.triggered.connect(self.toggle_spin_intake)
        context_menu.addAction(spin_action)

        clamp_action = QAction('Clamp Goal', self, checkable=True)
        clamp_action.setChecked(self.clampGoal)
        clamp_action.triggered.connect(self.toggle_clamp_goal)
        context_menu.addAction(clamp_action)

        turn_action = QAction('Turn Value: ' + str(self.turn), self)
        turn_action.triggered.connect(self.set_turn)
        context_menu.addAction(turn_action)

        delete_action = QAction('Delete Node', self)
        delete_action.triggered.connect(self.delete_node)
        context_menu.addAction(delete_action)

        insert_node_before_action = QAction('Insert Node Before', self)
        insert_node_before_action.triggered.connect(self.insert_node_before)
        context_menu.addAction(insert_node_before_action)

        insert_node_after_action = QAction('Insert Node After', self)
        insert_node_after_action.triggered.connect(self.insert_node_after)
        context_menu.addAction(insert_node_after_action)

        context_menu.exec(pos)

    def toggle_start_node(self):
        self.isStartNode = not self.isStartNode
        if self.isStartNode:
            self.gui_instance.set_start_node(self)
        else:
            self.gui_instance.clear_start_node()
        self.update()
        self.gui_instance.update_lines()
        print(f"Start Node: {self.isStartNode}")

    def toggle_end_node(self):
        self.isEndNode = not self.isEndNode
        if self.isEndNode:
            self.gui_instance.set_end_node(self)
        else:
            self.gui_instance.clear_end_node()
        self.update()
        self.gui_instance.update_lines()
        print(f"End Node: {self.isEndNode}")

    def toggle_spin_intake(self):
        self.spinIntake = not self.spinIntake
        print(f"Spin Intake: {self.spinIntake}")

    def toggle_clamp_goal(self):
        self.clampGoal = not self.clampGoal
        print(f"Clamp Goal: {self.clampGoal}")

    def set_turn(self):
        value, ok = QInputDialog.getInt(self, "Set Turn", "Enter turn (0-360):", self.turn, 0, 360)
        if ok:
            self.turn = value
            print(f"Turn set to: {self.turn}")

    def delete_node(self):
        self.gui_instance.remove_node(self)
        self.close()
        print(f"Node at ({self.x}, {self.y}) deleted")

    def insert_node_before(self):
        self.gui_instance.add_node(self.x+5, self.y+5, self.gui_instance.index_of(self))
    
    def insert_node_after(self):
        self.gui_instance.add_node(self.x+5, self.y+5, self.gui_instance.index_of(self)+1)

# Should be integrated directly with the Gui class
class SaveNodesDialog(QDialog):
    def __init__(self, nodes_string, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Nodes String")

        layout = QVBoxLayout()

        self.text_edit = QTextEdit(self)
        self.text_edit.setReadOnly(True)
        self.text_edit.setText(nodes_string)
        layout.addWidget(self.text_edit)

        copy_button = QPushButton("Copy to Clipboard", self)
        copy_button.clicked.connect(self.copy_to_clipboard)
        layout.addWidget(copy_button)

        self.setLayout(layout)

    def copy_to_clipboard(self):
        clipboard = QApplication.clipboard()
        clipboard.setText(self.text_edit.toPlainText())

class AutonomousPlannerGUIManager(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Image Viewer')
        self.setGeometry(100, 100, 699, 699)

        self.central_widget = DrawingWidget(self)
        self.setCentralWidget(self.central_widget)

        self.nodes = []
        self.start_node = None
        self.end_node = None

        self.layout = QVBoxLayout()

        self.label = ClickableLabel(parent=self.central_widget, gui_instance=self)

        pixmap = QPixmap('assets/top_down_cropped_high_stakes_field.png')  # Replace with your image path

        self.update()
        self.layout.addWidget(self.label)
        self.central_widget.setLayout(self.layout)

        self.create_menu_bar()


    def create_menu_bar(self):
        menu_bar = self.menuBar()

        file_menu = menu_bar.addMenu('File')
        tools_menu = menu_bar.addMenu('Tools')

        save_nodes_action = QAction('Save Nodes as String', self)
        save_nodes_action.triggered.connect(self.save_nodes)
        file_menu.addAction(save_nodes_action)

        create_cpp_action = QAction('Create C++ File from Nodes', self)
        create_cpp_action.triggered.connect(self.create_cpp_file)
        file_menu.addAction(create_cpp_action)

        clear_nodes_action = QAction('Clear All Nodes', self)
        clear_nodes_action.triggered.connect(self.clear_nodes)
        tools_menu.addAction(clear_nodes_action)

        show_acceleration_graph = QAction('Show Acceleration Graph', self)
        show_acceleration_graph.triggered.connect(self.acceleration_graph)
        tools_menu.addAction(show_acceleration_graph)

        show_position_graph = QAction('Show Position Graph', self)
        show_position_graph.triggered.connect(self.position_graph)
        tools_menu.addAction(show_position_graph)

        show_velocity_graph = QAction('Show Velocity Graph', self)
        show_velocity_graph.triggered.connect(self.velocity_graph)
        tools_menu.addAction(show_velocity_graph)


        load_nodes_action = QAction('Load Nodes from String', self)
        load_nodes_action.triggered.connect(self.load_nodes)
        file_menu.addAction(load_nodes_action)

    def index_of(self, node):
        return (self.nodes.index(node))

    def add_node(self, x, y, pos=-1):
        node = Node(x, y, self.central_widget, gui_instance=self)
        if (pos == -1):
            self.nodes.append(node)
        else:
            self.nodes.insert(pos, node)
        node.show()
        self.update_lines()
        print(f"Node created at ({x}, {y})")

    def update_lines(self):
        self.central_widget.update()
        self.central_widget.show()

    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)
            if node == self.start_node:
                self.start_node = None
            if node == self.end_node:
                self.end_node = None
            self.update_lines()

    def set_start_node(self, node):
        if self.start_node:
            self.start_node.isStartNode = False
            self.start_node.update()
            self.central_widget.update()
        self.start_node = node

    def clear_start_node(self):
        self.start_node = None

    def set_end_node(self, node):
        if self.end_node:
            self.end_node.isEndNode = False
            self.end_node.update()
            self.central_widget.update()
        self.end_node = node

    def clear_end_node(self):
        self.end_node = None


    def save_nodes(self):
        nodes_data = [
            [
                node.x,
                node.y,
                int(node.isStartNode),
                int(node.isEndNode),
                int(node.spinIntake),
                int(node.clampGoal),
                node.turn
            ]
            for node in self.nodes
        ]
        nodes_string = json.dumps(nodes_data, separators=(',', ':'))
        print("Nodes saved as string:", nodes_string)
        dialog = SaveNodesDialog(nodes_string, self)
        dialog.exec()


    def create_cpp_file(self):
        print("Creating C++ file from nodes...")

    def clear_nodes(self):
        while self.nodes:
            node = self.nodes.pop()
            node.delete_node()
        self.start_node = None
        self.end_node = None
        self.update_lines()
        print("Clearing all nodes...")

    def load_nodes(self):
        nodes_string, ok = QInputDialog.getText(self, "Load Nodes", "Enter nodes string:")
        if ok and nodes_string:
            nodes_data = json.loads(nodes_string)
            self.clear_nodes()
            for node_data in nodes_data:
                node = Node(node_data[0], node_data[1], self.central_widget, gui_instance=self)
                self.start_node = node if bool(node_data[2]) else self.start_node
                node.isStartNode = bool(node_data[2])
                self.end_node = node if bool(node_data[3]) else self.end_node
                node.isEndNode = bool(node_data[3])
                node.spinIntake = bool(node_data[4])
                node.clampGoal = bool(node_data[5])
                node.turn = node_data[6]
                self.nodes.append(node)
                node.show()

            self.update_lines()

    def acceleration_graph(self):
        self.central_widget.calculateScurveStuff()
        plt.figure(figsize=(12, 8))

        # Acceleration profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_time_intervals, self.central_widget.all_accelerations)
        plt.title('Acceleration Profile')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')

        plt.tight_layout()
        plt.show()

    def position_graph(self):
        self.central_widget.calculateScurveStuff()
        plt.figure(figsize=(12, 8))

        # Position profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_time_intervals, self.central_widget.all_positions)
        plt.title('Position Profile')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')

        plt.tight_layout()
        plt.show()

    def velocity_graph(self):
        self.central_widget.calculateScurveStuff()
        plt.figure(figsize=(12, 8))

        # Velocity profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_time_intervals, self.central_widget.all_velocities)
        plt.title('Velocity Profile')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')


        plt.tight_layout()
        plt.show()



class DrawingWidget(QWidget):
    def __init__(self, parent=None, image_path="assets/top_down_cropped_high_stakes_field.png"):
        super().__init__(parent)
        self.setGeometry(0, 0, 699, 699)
        self.image = QPixmap(image_path) if image_path else None
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

    def calculateScurveStuff(self):
        self.all_time_intervals = []
        self.all_positions = []
        self.all_velocities = []
        self.all_accelerations = []

        current_position = 0
        print(self.distances)
        for arc_length in self.distances:
            print(arc_length*(0.00523261802))
            time_intervals, positions, velocities, accelerations = self.generate_scurve_profile(arc_length*(0.00523261802))
            self.all_time_intervals.extend(time_intervals + (self.all_time_intervals[-1] if self.all_time_intervals else 0))
            self.all_positions.extend([p + current_position for p in positions])
            self.all_velocities.extend(velocities)
            self.all_accelerations.extend(accelerations)
            current_position += arc_length*(0.00523261802)
            print("CURRENT POSITION: " + str(current_position))

    def paintEvent(self, event):
        gui_instance = self.parent()

        painter = QPainter(self)
        painter.drawPixmap(self.rect(), self.image)

        if gui_instance.start_node and gui_instance.end_node and len(gui_instance.nodes) > 1:
            points = []
            for node in gui_instance.nodes:
                if node.isEndNode:
                    continue
                points.append(QPointF(node.x+10, node.y+10))
            points.append(QPointF(gui_instance.end_node.x+10, gui_instance.end_node.y+10))
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
        self.distances = []
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
                P0, P1, P2 = map(np.array, [(self.path.currentPosition().x(), self.path.currentPosition().y()), (cp2.x(), cp2.y()), (current.x(), current.y())])
                arc_length, _ = quad(quad_integrand, 0, 1, args=(P0, P1, P2))
                print(arc_length)
                self.distances.append(arc_length)
                self.path.quadTo(cp2, current)
            else:
                P0, P1, P2, P3 = map(np.array, [(self.path.currentPosition().x(), self.path.currentPosition().y()), (cp1.x(), cp1.y()), (cp2.x(), cp2.y()), (current.x(),
                                                                                                                                                             current.y())])
                arc_length, _ = quad(cubic_integrand, 0, 1, args=(P0, P1, P2, P3))
                print(arc_length)
                self.distances.append(arc_length)
                self.path.cubicTo(cp1, cp2, current)

            revSource = QLineF.fromPolar(target.length() * factor, angle).translated(current)
            cp1 = revSource.p2()

        # The final curve, that joins to the last point
        P0, P1, P2 = map(np.array, [(self.path.currentPosition().x(), self.path.currentPosition().y()), (cp1.x(), cp1.y()), (points[-1].x(), points[-1].y())])
        arc_length, _ = quad(quad_integrand, 0, 1, args=(P0, P1, P2))
        self.distances.append(arc_length)
        print(arc_length)
        self.path.quadTo(cp1, points[-1])

    # Function to calculate the distance covered for given v_max
    def calculate_total_distance(self, v_max, a_max, j_max):
        t_jerk = a_max / j_max
        v_after_jerk1 = 0.5 * j_max * t_jerk**2
        t_acc = (v_max - v_after_jerk1) / a_max
        
        d_jerk1 = (1/6) * j_max * t_jerk**3
        d_acc = (1/2) * a_max * t_acc**2 + v_after_jerk1 * t_acc
        d_jerk2 = (1/6) * j_max * t_jerk**3 + (v_after_jerk1 + a_max * t_acc) * t_jerk - (1/2) * a_max * t_jerk**2
        
        total_distance = 2 * (d_jerk1 + d_acc + d_jerk2)
        return total_distance


    # Bisection method to find the correct v_max
    def find_correct_v_max(self, distance, a_max, j_max, v_max_initial, tolerance=1e-6):
        v_max_low = 0
        v_max_high = v_max_initial
        v_max = (v_max_low + v_max_high) / 2
        
        while abs(self.calculate_total_distance(v_max, a_max, j_max) - distance) > tolerance:
            if self.calculate_total_distance(v_max, a_max, j_max) > distance:
                v_max_high = v_max
            else:
                v_max_low = v_max
            v_max = (v_max_low + v_max_high) / 2
        
        return v_max

    def find_tjerk(self, distance, t_jerk, j_max, dt=0.01, tolerance=1e-9):
        l = 0
        r = t_jerk+1
        c = 0
        while (l <= r):
            mid = l + (r-l)/2
            a_max = j_max*mid
            d_jerk1 = (j_max*mid**3)/6
            d_jerk2 = ((-j_max)*mid**3)/6 + (a_max*mid**2)/2 + ((j_max*mid**2)/2)*mid
            dist = d_jerk1 + d_jerk2
            print(mid, " ", dist, " ", distance)
            if (abs(distance-dist) < tolerance):
                return mid
            elif (dist < distance):
                l = mid
            else:
                r = mid
            c += 1
            if (c > 150):
                break
        return t_jerk

    def generate_scurve_profile(self, distance, v_max=.25, a_max=0.125, j_max=0.08, dt=0.01):
        t_jerk = a_max / j_max
        t_acc = (v_max - 2 * 0.5 * j_max * t_jerk**2) / a_max

        d_jerk1 = (1/6) * j_max * t_jerk**3
        d_acc = (a_max*t_acc**2)/2 + ((j_max*t_jerk**2)/2)*t_acc**1
        d_jerk1 = (j_max*t_jerk**3)/6
        d_jerk2 = ((-j_max)*t_jerk**3)/6 + (a_max*t_jerk**2)/2 + ((j_max*t_jerk**2)/2 + t_acc*a_max)*t_jerk

        t_flat = (distance - 2.0 * (d_jerk1 + d_acc + d_jerk2)) / v_max
        print(t_jerk, " ", t_acc, " ", t_flat)
        if t_flat < 0:
            v_max = self.find_correct_v_max(distance, a_max, j_max, v_max)
            t_jerk = a_max / j_max
            t_acc = (v_max - 2 * 0.5 * j_max * t_jerk**2) / a_max
            t_flat = 0

            d_jerk1 = (j_max*t_jerk**3)/6
            d_jerk2 = ((-j_max)*t_jerk**3)/6 + (a_max*t_jerk**2)/2 + ((j_max*t_jerk**2)/2 + t_acc*a_max)*t_jerk
            if (t_acc < 0):
                t_jerk = self.find_tjerk(distance/2, t_jerk, j_max)

                t_acc = 0
                d_acc = 0
                d_flat = 0
                a_max = t_jerk*j_max
                
            print("Dif: ", (distance / 2 / (1/6 * j_max))**(1/3)-((24*distance)/(j_max/dt))**(1/4))
            print(t_jerk, " ", t_acc, " ", t_flat, " ", v_max)



        total_time = 4 * t_jerk + 2 * t_acc + t_flat
        time_intervals = np.arange(0, total_time, dt)
        positions, velocities, accelerations = [], [], []
        s = 0
        if (t_acc == 0):
            a_max = j_max*t_jerk
            v_max = 0.5 * j_max * t_jerk**2 + a_max*t_acc

        d_jerk1 = (j_max*t_jerk**3)/6
        d_acc = (1/2) * a_max * t_acc**2 + 0.5 * j_max * t_jerk**2 * t_acc
        d_jerk2 = ((-j_max)*t_jerk**3)/6 + (a_max*t_jerk**2)/2 + ((j_max*t_jerk**2)/2 + t_acc*a_max)*t_jerk
        d_flat = v_max * t_flat
        if (d_acc < 0):
            d_acc = 0

        a = 0
        s = 0
        v = 0
        for t in time_intervals:
            if t < t_jerk:
                print("First jerk phase: ", d_jerk1, end=' ')
                # First jerk phase (increasing acceleration)
                a += j_max * dt
                v += a * dt
                s += v * dt
            elif t < t_jerk + t_acc:
                print("First acceleration phase: ", d_jerk1+d_acc, end=' ')
                # First acceleration phase (constant acceleration)
                a = a_max
                v += a * dt
                s += v * dt
            elif t < 2 * t_jerk + t_acc:
                print("Second jerk phase (negative): ", d_jerk1+d_jerk2+d_acc, end=' ')
                # Second jerk phase (decreasing acceleration)
                a -= j_max * dt
                v += a * dt
                s += v * dt
            elif t < 2 * t_jerk + t_acc + t_flat:
                print("Constant velocity phase: ", d_jerk1+d_jerk2+d_acc+d_flat, end=' ')
                # Constant velocity phase
                a = 0
                v = v_max
                s += v * dt
            elif t < 3 * t_jerk + t_acc + t_flat:
                print("Third jerk phase(negative): ", 2*d_jerk2+d_jerk1+d_acc+d_flat, end=' ')
                # Third jerk phase (decreasing acceleration)
                a -= j_max * dt
                v += a * dt
                s += v * dt
            elif t < 3 * t_jerk + 2*t_acc + t_flat:
                print("Second acceleration phase: ", 2*d_jerk2+d_jerk1+2*d_acc+d_flat, end=' ')
                # Second acceleration phase (constant negative acceleration)
                a = -a_max
                v += a * dt
                s += v * dt
            elif t < 4 * t_jerk + 2*t_acc + t_flat:
                print("Fourth jerk phase: ", 2*d_jerk2+2*d_jerk1+2*d_acc+d_flat, end=' ')
                # Fourth jerk phase (increasing acceleration)
                a += j_max * dt
                v += a * dt
                s += v * dt
            
            positions.append(s)
            velocities.append(v)
            accelerations.append(a)

        return time_intervals, positions, velocities, accelerations

                
if __name__ == '__main__':
    app = QApplication(sys.argv)

    window = AutonomousPlannerGUIManager()
    window.show()

    sys.exit(app.exec())
