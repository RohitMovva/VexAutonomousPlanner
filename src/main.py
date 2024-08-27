import sys
import os
import json
import yaml
from pathlib import Path
from PyQt6.QtWidgets import QApplication, QDialog, QLabel, QWidget, QVBoxLayout, QMenu, QInputDialog, QMainWindow, QTextEdit, QPushButton, QFileDialog, QDockWidget, QFormLayout, QSlider, QSpinBox, QLineEdit, QComboBox
from PyQt6.QtGui import QPixmap, QMouseEvent, QPainter, QColor, QAction, QPen, QPainterPath, QFontDatabase
from PyQt6.QtCore import Qt, QPoint, QLineF, QPointF, Qt
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, ceil
from distance_time_calculator import calculate_time_small_increment
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *

def load_fonts():
    font_dir = os.path.join(os.path.dirname(__file__), 'fonts')
    for font_file in os.listdir(font_dir):
        QFontDatabase.addApplicationFont(os.path.join(font_dir, font_file))

def resource_path(relative_path):
    try:
        base_path = sys._MEIPASS
        relative_path = relative_path[3:]
    except Exception:
        base_path = os.path.abspath(".")
    print(os.path.join(base_path, relative_path))
    return os.path.join(base_path, relative_path)

def create_files():
    files = [
        'routes/',
    ]
    for file in files:
        if not os.path.exists(file):
            os.makedirs(file)
            print(f"Created file: {file}")

# Random math function
def cross_product_2d(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]

def solve_quartic(a, b, c, d):
    """
    Solve a quartic equation of the form ax^4 + bx^3 + cx^2 + dx + e = 0.
    This is a simplified version and may not work for all cases.
    """
    # Convert to a depressed quartic
    p = (8*a*c - 3*b**2) / (8*a**2)
    q = (b**3 - 4*a*b*c + 8*a**2*d) / (8*a**3)

    # Solve the resolvent cubic
    D0 = c**2 - 3*b*d + 12*a*e
    D1 = 2*c**3 - 9*b*c*d + 27*b**2*e + 27*a*d**2 - 72*a*c*e
    Q = ((D1 + math.sqrt(D1**2 - 4*D0**3)) / 2) ** (1/3)
    S = 0.5 * math.sqrt(-2*p/3 + (Q + D0/Q)/(3*a))

    # Calculate the roots
    root1 = -b/(4*a) + S + 0.5 * math.sqrt(-4*S**2 - 2*p + q/S)
    root2 = -b/(4*a) + S - 0.5 * math.sqrt(-4*S**2 - 2*p + q/S)
    root3 = -b/(4*a) - S + 0.5 * math.sqrt(-4*S**2 - 2*p - q/S)
    root4 = -b/(4*a) - S - 0.5 * math.sqrt(-4*S**2 - 2*p - q/S)

    return [root1, root2, root3, root4]

# BEZIER CURVE THINGS

def point_to_array(p):
    return np.array([p.x(), p.y()])

def max_speed_based_on_curvature(curvature, V_base, K):
    return V_base / (1 + K * curvature)

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

def distToTime(distance, segments):
    l = 0
    r = len(segments)-1
    mid = None
    while (l <= r):
        mid = int(l + (r-l)/2)
        if (segments[mid] < distance):
            l = mid+1
        elif (segments[mid] > distance):
            r = mid-1
        else:
            break
        
    return mid/1000.0

def getHeading(distance, segments, start, end, cp1, cp2=None):
    t = distToTime(distance, segments)
    if (cp2 == None):
        return quad_bezier_angle(t, start, cp1, end)
    return cubic_bezier_angle(t, start, cp1, cp2, end)

def getConfigValue(keyname):
    with open(resource_path('../config.yaml'), 'r') as file:
        config = yaml.safe_load(file)
    if (config == None):
        config = {}
    return config.get(keyname) # Prevents error if key doesn't exist in dict

def setConfigValue(keyname, value):
    with open(resource_path('../config.yaml'), 'r') as file:
        config = yaml.safe_load(file)
    if (config == None):
        config = {}
    config[keyname] = value
    with open(resource_path('../config.yaml'), 'w') as file:
                yaml.safe_dump(config, file)

# Click listener object
class ClickableLabel(QLabel):
    def __init__(self, parent=None, gui_instance=None):
        super().__init__(parent)
        self.gui_instance = gui_instance

        self.setMouseTracking(True)

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            x = int(event.position().x()) + 7 # I have no clue on God's green earth on why this is needed but it is
            y = int(event.position().y()) + 7
            print(f"Mouse clicked at ({x}, {y})")
            self.gui_instance.add_node(x, y)
        super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event: QMouseEvent):
        x = int(event.position().x()) + 7
        y = int(event.position().y()) + 7
        scale = 700/2000
        x = round(((x * (scale*2000-scale*34*2)/(scale*2000)) / (scale*2000-scale*34*2) - 0.5) * 12**2, 2)
        y = round(((y * (scale*2000-scale*34*2)/(scale*2000)) / (scale*2000-scale*34*2) - 0.5) * 12**2, 2)

        self.gui_instance.update_coordinate_display(x, y)
        super().mouseMoveEvent(event)

# Node that stores data for auton route
class Node(QWidget):
    def __init__(self, x, y, parent=None, gui_instance=None):
        super().__init__(parent)
        self.x = x
        self.y = y
        print(x, y)
        # Scale pixel value down to account for the extra padding that isn't part of the field on either side, scale to between 0-1, subtract 0.5 to center and turn into inches
        self.scale = 700/2000
        self.absX = ((self.x * (self.scale*2000-self.scale*34*2)/(self.scale*2000)) / (self.scale*2000-self.scale*34*2) - 0.5) * 12**2
        self.absY = ((self.y * (self.scale*2000-self.scale*34*2)/(self.scale*2000)) / (self.scale*2000-self.scale*34*2) - 0.5) * 12**2

        self.gui_instance = gui_instance
        self.isStartNode = False
        self.isEndNode = False
        self.spinIntake = False
        self.clampGoal = False
        self.turn = 0
        self.wait_time = 0
        self.hasAction = (self.spinIntake or self.clampGoal or self.turn != 0 or self.wait_time != 0)
        self.setFixedSize(10, 10)
        self.move(x-5, y-5)
        self.dragging = False
        self.offset = QPoint(0, 0)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        if self.isStartNode:
            painter.setBrush(QColor("green"))
        elif self.isEndNode:
            painter.setBrush(QColor("red"))
        elif self.hasAction:
            painter.setBrush(QColor("#1338BE"))
        else:
            painter.setBrush(QColor("#1F456E"))

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
            self.x = new_pos.x()+5
            self.y = new_pos.y()+5
            self.absX = ((self.x * (self.scale*2000-self.scale*34*2)/(self.scale*2000)) / (self.scale*2000-self.scale*34*2) - 0.5) * 12**2
            self.absY = ((self.y * (self.scale*2000-self.scale*34*2)/(self.scale*2000)) / (self.scale*2000-self.scale*34*2) - 0.5) * 12**2
            self.move(new_pos)
            self.gui_instance.update_lines()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = False
        if (self.gui_instance.current_working_file != None):
            self.gui_instance.auto_save()
        super().mouseReleaseEvent(event)

    def show_context_menu(self, pos):
        context_menu = QMenu(self)

        attributes_menu = QMenu("Attributes", self)
        node_menu = QMenu("Node Actions", self)

        start_action = QAction('Start Node', self, checkable=True)
        start_action.setChecked(self.isStartNode)
        start_action.triggered.connect(self.toggle_start_node)
        attributes_menu.addAction(start_action)

        end_action = QAction('End Node', self, checkable=True)
        end_action.setChecked(self.isEndNode)
        end_action.triggered.connect(self.toggle_end_node)
        attributes_menu.addAction(end_action)

        spin_action = QAction('Spin Intake', self, checkable=True)
        spin_action.setChecked(self.spinIntake)
        spin_action.triggered.connect(self.toggle_spin_intake)
        attributes_menu.addAction(spin_action)

        clamp_action = QAction('Clamp Goal', self, checkable=True)
        clamp_action.setChecked(self.clampGoal)
        clamp_action.triggered.connect(self.toggle_clamp_goal)
        attributes_menu.addAction(clamp_action)

        turn_action = QAction('Turn Value: ' + str(self.turn), self)
        turn_action.triggered.connect(self.set_turn)
        attributes_menu.addAction(turn_action)

        wait_action = QAction('Wait time: ' + str(self.wait_time), self)
        wait_action.triggered.connect(self.set_wait)
        attributes_menu.addAction(wait_action)

        delete_action = QAction('Delete Node', self)
        delete_action.triggered.connect(self.delete_node)
        node_menu.addAction(delete_action)

        insert_node_before_action = QAction('Insert Node Before', self)
        insert_node_before_action.triggered.connect(self.insert_node_before)
        node_menu.addAction(insert_node_before_action)

        insert_node_after_action = QAction('Insert Node After', self)
        insert_node_after_action.triggered.connect(self.insert_node_after)
        node_menu.addAction(insert_node_after_action)

        context_menu.addMenu(attributes_menu)
        context_menu.addMenu(node_menu)

        context_menu.exec(pos)

    def toggle_start_node(self):
        self.isStartNode = not self.isStartNode
        if self.isStartNode:
            self.gui_instance.set_start_node(self)
            if self.isEndNode:
                self.gui_instance.clear_end_node()
                self.isEndNode = False
        else:
            self.gui_instance.clear_start_node()
        self.repaint()
        self.gui_instance.update_lines()
        print(f"Start Node: {self.isStartNode}")

    def toggle_end_node(self):
        self.isEndNode = not self.isEndNode
        if self.isEndNode:
            self.gui_instance.set_end_node(self)
            if self.isStartNode:
                self.gui_instance.clear_start_node()
                self.isStartNode = False
        else:
            self.gui_instance.clear_end_node()
        self.update()
        self.gui_instance.update_lines()
        print(f"End Node: {self.isEndNode}")

    def toggle_spin_intake(self):
        self.spinIntake = not self.spinIntake
        self.hasAction = (self.spinIntake or self.clampGoal or self.turn != 0 or self.wait_time != 0)
        print(f"Spin Intake: {self.spinIntake}")

    def toggle_clamp_goal(self):
        self.clampGoal = not self.clampGoal
        self.hasAction = (self.spinIntake or self.clampGoal or self.turn != 0 or self.wait_time != 0)
        print(f"Clamp Goal: {self.clampGoal}")

    def set_turn(self):
        value, ok = QInputDialog.getInt(self, "Set Turn", "Enter turn (0-360):", self.turn, 0, 360)
        if ok:
            self.turn = value
            self.hasAction = (self.spinIntake or self.clampGoal or self.turn != 0 or self.wait_time != 0)
            print(f"Turn set to: {self.turn}")

    def set_wait(self):
        value, ok = QInputDialog.getInt(self, "Set Wait Time", "Enter time (seconds):", self.wait_time, 0)
        if ok:
            self.wait_time = value
            self.hasAction = (self.spinIntake or self.clampGoal or self.turn != 0 or self.wait_time != 0)
            print(f"Wait time set to: {self.wait_time}")

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

        self.setWindowTitle('Path Planner') # Totally not inspired by Pronounce that
        self.layout = QVBoxLayout()

        self.nodes = []
        self.start_node = None
        self.end_node = None

        self.current_working_file = None
        self.routes_header_path = None
        self.routes_folder_path = None
        
        self.clearing_nodes = False


        autonomous_path = getConfigValue('autonomous_repository_path')
        if autonomous_path == None:
            autonomous_path = QFileDialog.getExistingDirectory(self, "Select Autonomous Program Directory", 
                                                      str(Path(os.getcwd()).parent.parent.absolute()))
            
            setConfigValue('autonomous_repository_path', autonomous_path + "/routes.h")
            print(f"Added autonomous repository path: {autonomous_path}/routes.h")


        routes_path = getConfigValue('routes_folder_path')
        if autonomous_path == None:
            routes_path = QFileDialog.getExistingDirectory(self, "Select Routes Folder", 
                                                      str(Path(os.getcwd()).parent.parent.absolute()))
            
            setConfigValue('routes_folder_path', routes_path)
            print(f"Added routes folder path: {routes_path}")
        
        self.routes_header_path = autonomous_path
        self.routes_folder_path = routes_path

        self.max_velocity = getConfigValue('max_velocity')
        self.max_acceleration = getConfigValue('max_acceleration')
        self.max_jerk = getConfigValue('max_jerk')

        # Image and path widget
        self.central_widget = DrawingWidget(self)
        self.setCentralWidget(self.central_widget)
        self.central_widget.setFixedSize(700, 700)

        # Settings Dock Widget
        self.settings_dock_widget = SettingsDockWidget(self.max_velocity, self.max_acceleration, self.max_jerk, self)

        # Click listener
        self.label = ClickableLabel(parent=self.central_widget, gui_instance=self)

        # Add widgets to layout
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        self.layout.addWidget(self.label)

        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.settings_dock_widget)

        self.update()
        self.create_menu_bar()


    def create_menu_bar(self):
        menu_bar = self.menuBar()

        file_menu = menu_bar.addMenu('File')
        tools_menu = menu_bar.addMenu('Tools')

        save_nodes_file_action = QAction('Save Nodes to File', self)
        save_nodes_file_action.triggered.connect(self.save_nodes_to_file)
        file_menu.addAction(save_nodes_file_action)

        load_nodes_file_action = QAction('Load Nodes from File', self)
        load_nodes_file_action.triggered.connect(self.load_nodes_from_file)
        file_menu.addAction(load_nodes_file_action)

        save_nodes_string_action = QAction('Convert to String', self)
        save_nodes_string_action.triggered.connect(self.save_nodes_to_string)
        file_menu.addAction(save_nodes_string_action)

        load_nodes_string_action = QAction('Load Nodes from String', self)
        load_nodes_string_action.triggered.connect(self.load_nodes_from_string)
        file_menu.addAction(load_nodes_string_action)

        set_current_file_action = QAction('Set Current Working File', self)
        set_current_file_action.triggered.connect(self.set_working_file)
        file_menu.addAction(set_current_file_action)

        clear_nodes_action = QAction('Clear All Nodes', self)
        clear_nodes_action.triggered.connect(self.clear_nodes)
        tools_menu.addAction(clear_nodes_action)

        show_position_graph = QAction('Show Position Graph', self)
        show_position_graph.triggered.connect(self.position_graph)
        tools_menu.addAction(show_position_graph)

        show_velocity_graph = QAction('Show Velocity Graph', self)
        show_velocity_graph.triggered.connect(self.velocity_graph)
        tools_menu.addAction(show_velocity_graph)

        show_acceleration_graph = QAction('Show Acceleration Graph', self)
        show_acceleration_graph.triggered.connect(self.acceleration_graph)
        tools_menu.addAction(show_acceleration_graph)

        show_heading_graph = QAction('Show Heading Graph', self)
        show_heading_graph.triggered.connect(self.heading_graph)
        tools_menu.addAction(show_heading_graph)

    def index_of(self, node):
        return (self.nodes.index(node))
    
    def update_coordinate_display(self, x, y):
        self.settings_dock_widget.set_current_coordinates(x, y)
    
    def set_max_velocity(self, new_velocity):
        self.max_velocity = new_velocity
    def set_max_acceleration(self, new_acceleration):
        self.max_acceleration = new_acceleration
    def set_max_jerk(self, new_jerk):
        self.max_jerk = new_jerk

    def add_node(self, x, y, pos=-1):
        node = Node(x, y, self.central_widget, gui_instance=self)
        if (pos == -1):
            self.nodes.append(node)
        else:
            self.nodes.insert(pos, node)
        node.show()
        self.update_lines()
        if (self.current_working_file != None):
            self.auto_save()
        print(f"Node created at ({node.absX}, {node.absY})")

    def update_lines(self):
        self.central_widget.repaint()
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
        if (self.current_working_file != None):
            self.auto_save()

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

    def save_nodes_to_string(self):
        nodes_string = self.convert_nodes()
        print("Nodes saved as string:", nodes_string)
        dialog = SaveNodesDialog(nodes_string, self)
        dialog.exec()

    def load_nodes_from_string(self):
        nodes_string, ok = QInputDialog.getText(self, "Load Nodes", "Enter nodes string:")
        if ok and nodes_string:
            self.load_nodes(nodes_string)

    def save_nodes_to_file(self):
        nodes_string = ""
        if (len(self.nodes) > 0):
            nodes_string = self.convert_nodes()
        full_path = None

        if (self.current_working_file == None):
            file_name, ok = QInputDialog.getText(self, "Save Route to File", "Enter file name (without extension):")
            if (not ok or not file_name):
                return
            
            file_name = file_name.strip() + ".txt"
            folder = QFileDialog.getExistingDirectory(self, "Select Directory to Save File",
                                                      str(Path(os.getcwd()).parent.absolute()))
            if (not folder):
                return
            
            full_path = f"{folder}/{file_name}"
        else:
            full_path = self.routes_folder_path + "/" + self.current_working_file + ".txt"
        nodes_data = []
        nodes_map = []
        if (len(self.nodes) > 2 and self.start_node and self.end_node):
            time_intervals, positions, velocities, accelerations, headings, nodes_map = self.central_widget.calculateScurveStuff()
            for i in range(0, len(time_intervals), 50): # Every 25ms save data
                nodes_data.append([velocities[i], headings[i]])

        nodes_actions = [
                [
                    int(node.spinIntake),
                    int(node.clampGoal),
                    node.turn,
                    node.wait_time
                ]
                for node in self.nodes
            ]
        print(nodes_map)
        print(len(nodes_map), len(nodes_actions))
        for i in range(0, len(nodes_map)):
            nodes_data.insert(int(nodes_map[i]/50), nodes_actions[i])

        self.fill_template(nodes_data)
        with open(full_path, 'w') as file:
            file.write(nodes_string)
        print(f"Route saved to {full_path}")

    def load_nodes_from_file(self, dialogue=True):
        file_name = None
        if (dialogue):
            file_name, _ = QFileDialog.getOpenFileName(self, "Load Route from File", "", "Text Files (*.txt);;All Files (*)")
        else:
            file_name = self.routes_folder_path + "/" + self.current_working_file + ".txt"
        if file_name != None:
            with open(file_name, 'r') as file:
                nodes_string = file.read()
            print(f"Route loaded from {file_name}")
            self.load_nodes(nodes_string)

    def set_working_file(self):
        file_name, ok = QInputDialog.getText(self, "File to save route to", "Enter file name (without extension):")
        if ok and file_name:
            self.current_working_file = file_name
            folder = self.routes_folder_path
            if (folder == None):
                folder = QFileDialog.getExistingDirectory(self, "Select Directory to Save File", 
                                                      str(Path(os.getcwd()).parent.absolute()))
                
            if folder:
                self.routes_folder_path = folder
                full_path = f"{folder}/{self.current_working_file}.txt"
                if not os.path.exists(full_path):
                    with open(full_path, 'w+') as file: # Creates file if it isn't already created
                        pass
                print(f"Set current working file at {full_path}")
                if (len(self.nodes) > 0 or os.stat(full_path).st_size == 0):
                    self.auto_save()
                else:
                    self.load_nodes_from_file(False)

    def clear_nodes(self):
        self.clearing_nodes = True
        while self.nodes:
            node = self.nodes.pop()
            node.delete_node()
        self.start_node = None
        self.end_node = None
        self.clearing_nodes = False
        self.update_lines()
        print("Clearing all nodes...")

    def load_nodes(self, str):
        nodes_data = json.loads(str)
        self.clear_nodes()
        for node_data in nodes_data:
            if (len(node_data) > 4):
                node = Node(node_data[0], node_data[1], self.central_widget, gui_instance=self)
                self.start_node = node if bool(node_data[2]) else self.start_node
                node.isStartNode = bool(node_data[2])
                self.end_node = node if bool(node_data[3]) else self.end_node
                node.isEndNode = bool(node_data[3])
                node.spinIntake = bool(node_data[4])
                node.clampGoal = bool(node_data[5])
                node.turn = node_data[6]
                node.wait_time = node_data[7]
                node.hasAction = (node.spinIntake or node.clampGoal or node.turn != 0 or node.wait_time != 0)
                print(node.hasAction)
                self.nodes.append(node)
                node.show()

        self.update_lines()

    def convert_nodes(self, as_list=False):
        nodes_data = [
            [
                node.x,
                node.y,
                int(node.isStartNode),
                int(node.isEndNode),
                int(node.spinIntake),
                int(node.clampGoal),
                node.turn,
                node.wait_time
            ]
            for node in self.nodes
        ]
        if (as_list):
            return nodes_data
        nodes_string = json.dumps(nodes_data, separators=(',', ':'))
        return nodes_string
    
    def auto_save(self):
        if (self.current_working_file != None and self.start_node and self.end_node and not self.clearing_nodes):
            if (len(self.nodes) > 0):
                self.save_nodes_to_file()
            else:
                self.load_nodes_from_file(False)

    def fill_template(self, nodes_data):  
        # Create the string to insert
        stringified = []
        for i in range(0, len(nodes_data)):
            if (len(nodes_data[i]) > 2):
                stringified.append(f"{{")
                for j in range(0, len(nodes_data[i])):
                    stringified[-1] += f"{nodes_data[i][j]}"
                    if (j < len(nodes_data[i])-1):
                        stringified[-1] += f", "
                stringified[-1] += f"}}"
            else:
                stringified.append(f"{{{nodes_data[i][0]}, {nodes_data[i][1]}}}")

        # pairs = [f"{{{v}, {h}}}" for v, h in nodes_data]
        insertion = f"std::vector<std::vector<float>> {self.current_working_file} = {{{', '.join(stringified)}}};\n"
        
        try:
            # Read the content of routes.h
            with open(self.routes_header_path, "r") as routes_file:
                content = routes_file.readlines()
            
            # Find the line with the specified route name
            inserted = False
            for i, line in enumerate(content):
                if line.strip().startswith(f"std::vector<std::vector<float>> {self.current_working_file} ="):
                    content[i] = insertion
                    inserted = True
                    break
            if not inserted:
                # Insert before the #endif line
                for i, line in enumerate(content):
                    if line.strip() == "#endif":
                        content.insert(i, insertion)
                        break
        
        except FileNotFoundError:
            # If routes.h does not exist, create a new one with the necessary structure
            content = [
                "#ifndef ROUTES_H\n",
                "#define ROUTES_H\n",
                "#include <vector>\n",
                "\n",
                insertion,
                "\n",
                "#endif\n"
            ]
        
        # Write the updated content to routes.h
        with open(self.routes_header_path, "w") as routes_file:
            routes_file.writelines(content)

    def changeField(self, fieldType):
        if (fieldType == "High Stakes Match"):
            self.central_widget.update_image_path(resource_path('../assets/V5RC-HighStakes-Match-2000x2000.png'))
        else:
            self.central_widget.update_image_path(resource_path('../assets/V5RC-HighStakes-Skills-2000x2000.png'))

    def setVelocity(self, newVelocity):
        self.max_velocity = newVelocity
        setConfigValue("max_velocity", self.max_velocity)
        
    def setAcceleration(self, newAcceleration):
        self.max_acceleration = newAcceleration
        setConfigValue("max_acceleration", self.max_acceleration)

    def setJerk(self, newJerk):
        self.max_jerk = newJerk
        setConfigValue("max_jerk", self.max_jerk)


    def position_graph(self):
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk)
        plt.figure(figsize=(12, 8))

        # Position profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_time_intervals, self.central_widget.all_positions)
        plt.title('Position Profile')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (ft)')

        plt.tight_layout()
        plt.show()

    def velocity_graph(self):
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk)
        plt.figure(figsize=(12, 8))

        # Velocity profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_time_intervals, self.central_widget.all_velocities)
        plt.title('Velocity Profile')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (ft/s)')


        plt.tight_layout()
        plt.show()

    def acceleration_graph(self):
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk)
        plt.figure(figsize=(12, 8))

        # Acceleration profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_time_intervals, self.central_widget.all_accelerations)
        plt.title('Acceleration Profile')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (ft/s²)')

        plt.tight_layout()
        plt.show()

    def heading_graph(self):
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk)
        plt.figure(figsize=(12, 8))

        # Heading profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_positions, self.central_widget.all_headings)
        plt.title('Heading Profile')
        plt.xlabel('Position (ft)')
        plt.ylabel('Heading (degrees)')


        plt.tight_layout()
        plt.show()

class SettingsDockWidget(QDockWidget):
    def __init__(self, max_velocity, max_acceleration, max_jerk, parent=None):
        super().__init__("Settings", parent)

        self.parent = parent
        # Create the settings widget
        settings_widget = QWidget()
        settings_layout = QFormLayout()

        # Add Field Type drop-down menu
        self.field_type_combo = QComboBox()
        self.field_type_combo.addItems(["High Stakes Match", "High Stakes Skills"])
        self.field_type_combo.setCurrentIndex(0)
        settings_layout.addRow("Field Type:", self.field_type_combo)
        self.field_type_combo.currentIndexChanged.connect(self.on_field_type_changed)

        # Add max jerk, acceleration, and velocity inputs

        self.velocity_input = QSpinBox()
        self.velocity_input.setRange(0, 100)  # Adjust range as needed
        self.velocity_input.setValue(max_velocity)
        settings_layout.addRow("Max Velocity (ft/s):", self.velocity_input)
        self.velocity_input.valueChanged.connect(self.on_velocity_changed)

        self.acceleration_input = QSpinBox()
        self.acceleration_input.setRange(0, 100)  # Adjust range as needed
        self.acceleration_input.setValue(max_acceleration)
        settings_layout.addRow("Max Acceleration (ft/s²):", self.acceleration_input)
        self.acceleration_input.valueChanged.connect(self.on_acceleration_changed)

        self.jerk_input = QSpinBox()
        self.jerk_input.setRange(0, 100)  # Adjust range as needed
        self.jerk_input.setValue(max_jerk)
        settings_layout.addRow("Max Jerk (ft/s³):", self.jerk_input)
        self.jerk_input.valueChanged.connect(self.on_jerk_changed)

        # Add labels to display current x and y coordinates
        self.current_x_label = QLabel("0")
        settings_layout.addRow("Current X:", self.current_x_label)

        self.current_y_label = QLabel("0")
        settings_layout.addRow("Current Y:", self.current_y_label)

        # Set the layout for the settings widget
        settings_widget.setLayout(settings_layout)
        self.setWidget(settings_widget)

    def set_current_coordinates(self, x, y):
        self.current_x_label.setText(str(x))
        self.current_y_label.setText(str(y))

    def on_field_type_changed(self):
        field_type = self.field_type_combo.currentText()
        print(f"Field Type changed: {field_type}")
        self.parent.changeField(field_type)

    def on_velocity_changed(self):
        max_velocity = self.velocity_input.value()
        print(f"Max Velocity changed: {max_velocity} ft/s")
        self.parent.setVelocity(max_velocity)

    def on_acceleration_changed(self):
        max_acceleration = self.acceleration_input.value()
        print(f"Max Acceleration changed: {max_acceleration} ft/s^2")
        self.parent.setAcceleration(max_acceleration)

    def on_jerk_changed(self):
        max_jerk = self.jerk_input.value()
        print(f"Max Jerk changed: {max_jerk} ft/s^3")
        self.parent.setJerk(max_jerk)


class DrawingWidget(QWidget):
    def __init__(self, parent=None, image_path=resource_path('../assets/V5RC-HighStakes-Match-2000x2000.png')):
        super().__init__(parent)
        self.parent = parent
        self.setGeometry(0, 0, 700, 700)
        self.image = QPixmap(image_path) if image_path else None
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

    def update_image_path(self, new_path):
        self.image = QPixmap(new_path)
        self.update()

    # def generate_maximimum_velocities(self, v_max, a_max, j_max, dd):

        

    def distance_to_max_velocity(self, v_max, distance, segments, p0, p1, p2, p3=None, K=10.0):
        t_along_curve = distToTime(distance, segments)
        curvature = None
        if (p3 == None): # Quadratic
            curvature = quadratic_bezier_curvature(p0, p1, p2, t_along_curve)
        else:
            curvature = cubic_bezier_curvature(p0, p1, p2, p3, t_along_curve)
            
        adjusted_vmax = max_speed_based_on_curvature(curvature, v_max, K)
        return adjusted_vmax
    
    def convert_velocity_parameterization(self, velocities, distance_interval, time_interval):
        """
        Convert a list of velocities from constant distance parameterization to constant time parameterization.
        
        Args:
        velocities (list): List of velocities, each measured after a constant distance interval.
        distance_interval (float): The constant distance interval between velocity measurements.
        time_interval (float): The desired constant time interval for the new parameterization.
        
        Returns:
        list: A new list of velocities parameterized by the constant time interval.
        """
        # Convert input to numpy array for vectorized operations
        v = np.array(velocities)
        
        # Handle zero velocities to avoid division by zero
        v = np.maximum(v, 1e-2)  # Replace zeros with a small positive number
        
        # Calculate the times at which the original velocities were measured
        times = np.cumsum(distance_interval / v)
        
        # Remove any potential inf or nan values
        valid_indices = np.isfinite(times)
        times = times[valid_indices]
        v = v[valid_indices]
        
        if len(times) == 0:
            return []  # Return empty list if all times were invalid
        
        # Create a new time array with constant time intervals
        new_times = np.arange(0, times[-1], time_interval)
        # Interpolate velocities at the new time points
        new_velocities = np.interp(new_times, times, v)
        
        return new_velocities.tolist()
    
    def mushingprufil(self, v_max, a_max, j_max, bezier_info, dd=0.0025):
        # Calculate path length
        curve_segments = []
        total_path_length = 0
        for curve in bezier_info:
            if (len(curve) == 3):
                curve_segments.append(createCurveSegments(curve[0], curve[2], curve[1]))
            else:
                curve_segments.append(createCurveSegments(curve[0], curve[3], curve[1], curve[2]))
            total_path_length += curve_segments[-1][-1]
        # Generate list of everything based on the path length
        print("Path length: ", np.ceil(total_path_length/dd))
        positions = []
        velocities = [0] * ceil(total_path_length/dd)
        accelerations = [0] * ceil(total_path_length/dd)
        headings = []
        times = []
        nodes_map = [0] # Calculate nodes map (after start node) after we finish reparametizing the list to time
        # Triple pass curve shi
        current_curve = 0
        current_points = bezier_info[current_curve]
        current_segments = curve_segments[current_curve]

        current_dist = 0
        past_curve_distance_sum = 0

        past_acceleration = 0

        # TODO remove redundant current distance variable

        # First smoothing pass, makes velocities continous when increasing.
        timesum = 0
        for i in range(0, ceil(total_path_length/dd)):
            maximum_velocity = v_max
            if (len(current_points) == 3): # Quadratic
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                             current_points[0], current_points[1], current_points[2])
            else: # Cubic
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                current_points[0], current_points[1], current_points[2], current_points[3])
            # print(maximum_velocity)
            direction = np.sign(maximum_velocity - velocities[i-1])
            dt = calculate_time_small_increment(dd, velocities[i-1], accelerations[i-1], direction*j_max,
                                                maximum_velocity, a_max)
            timesum += dt
            desired_acceleration = (maximum_velocity - velocities[i-1]) / dt
            max_accel_change = j_max * dt
            if (desired_acceleration < 0):
                accelerations[i] = max(max(desired_acceleration, past_acceleration - max_accel_change), -1*a_max)
            else:
                accelerations[i] = min(min(desired_acceleration, past_acceleration + max_accel_change), a_max)

            past_acceleration = accelerations[i-1]
            velocities[i] = (velocities[i-1] + accelerations[i-1] * dt)
            current_dist += velocities[i]*dt
            # if (i < 500):
                # print(velocities[i], " ", accelerations[i], " ", max_accel_change, " ", j_max, " ", dt)
            # print(velocities[i]*dt, " ", dd)
            # print("INFO: ", current_dist, " ", velocities[i], " ", accelerations[i])
            # print("Acceleration info: ", desired_acceleration, " ", accelerations[i])
            # print("Time: ", timesum, " ", "Velocity: ", velocities[i])
            
            # Reset variables for new bezier curve
            if (current_dist > current_segments[-1] + past_curve_distance_sum and
                current_curve < len(bezier_info)-1): # 
                current_curve += 1
                current_points = bezier_info[current_curve]
                current_segments = curve_segments[current_curve]

                past_curve_distance_sum = current_dist

        print("Velocity lengths: ", len(velocities))
        # Reset variables for second smoothing pass
        current_dist = past_curve_distance_sum + current_segments[-1]
        current_curve = len(bezier_info)-1
        past_curve_distance_sum = past_curve_distance_sum

        current_segments = createCurveSegments(bezier_info[current_curve][0], bezier_info[current_curve][2], bezier_info[current_curve][1])
        # IMPORTANT Velocity MUST end at 0 
        velocities[-1] = 0
        accelerations[-1] = 0


        # Second smoothing path to make velocities continous going downwards
        for i in range(len(velocities) - 2, -1, -1): # TODO This causes times to be off, fix that
            # Set dt to a value based on how long it will take to cover x distance
            maximum_velocity = v_max
            if (len(bezier_info[current_curve]) == 3):
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                             current_points[0], current_points[1], current_points[2])
            else:
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                current_points[0], current_points[1], current_points[2], current_points[3])
            direction = np.sign(maximum_velocity - velocities[i-1])
            dt = calculate_time_small_increment(dd, velocities[i+1], accelerations[i+1], direction*j_max,
                                                maximum_velocity, a_max)
            
            desired_accel = (velocities[i]-velocities[i+1]) / dt
            if (1750 < i < 3000):
                print(desired_accel, " ", velocities[i], " ", velocities[i+1])
            max_accel_change = j_max * dt
            if (desired_accel < 0):
                accelerations[i] = max(max(desired_accel, accelerations[i+1] - max_accel_change), -1*a_max)
            else:
                accelerations[i] = min(min(desired_accel, accelerations[i+1] + max_accel_change), a_max)

            velocities[i] = velocities[i+1] + accelerations[i] * dt
            current_dist -= velocities[i+1]*dt
            # print(velocities[i+1]*dt, " ", dd)
            # if (abs(i-len(velocities)) < 500):
            #     print("INFO: ", current_dist, " ", past_curve_distance_sum, " ", accelerations[i], " ", velocities[i]-velocities[i+1])
            # print("INFO: ", current_dist, " ", velocities[i], " ", accelerations[i])

            # Reset variables
            if (current_dist < past_curve_distance_sum
                and current_curve > 0):
                # print("Recomputing vars: ", past_curve_distance_sum, " ", current_dist, " ", current_curve)
                current_curve -= 1
                current_points = bezier_info[current_curve]
                current_segments = curve_segments[current_curve]
                past_curve_distance_sum -= current_segments[-1]
                # print("Res: ", past_curve_distance_sum, " ", current_dist, " ", current_curve)
                # current_dist = 0

        # TODO Add loops here to smooth out acceleration list

        # TODO Reparametize velocities and accelerations array based on time rather than distance
        # Included, resize all lists
        # velocities = self.convert_velocity_parameterization(velocities, dd, 0.0005)
        print("Reparametized: ", len(velocities))
        num_setpoints = len(velocities)
        positions = [0]*num_setpoints
        accelerations = [0]*num_setpoints
        headings = [0]*num_setpoints
        times = [0]*num_setpoints

        # Reset variables for final loop
        current_curve = 0
        past_curve_distance_sum = 0

        current_segments = createCurveSegments(bezier_info[current_curve][0], bezier_info[current_curve][2], bezier_info[current_curve][1])

        # Final loops for calculating the following lists: positions, headings, and time_intervals
        dt = 0.0005 # Standardize dt
        for i in range(0, num_setpoints):
            # TODO Replace these with the proper velocity and acceleration formulas based on jerk
            positions[i] = (positions[i-1] + velocities[i-1]*dt)
            accelerations[i] = ((velocities[i]-velocities[i-1])/dt)
            times[i] = (times[i-1] + dt)
            # print("Time: ", times[i])


            if (len(bezier_info[current_curve]) == 3):
                headings[i] = (getHeading(positions[i], current_segments, \
                current_points[0], current_points[2], current_points[1]))
            else:
                headings[i] = (getHeading(positions[i], current_segments, \
                current_points[0], current_points[2], current_points[3], current_points[1]))

            current_dist = positions[i]

            if (current_dist > current_segments[-1] + past_curve_distance_sum
                and current_curve < len(bezier_info) - 1):
                current_curve += 1
                nodes_map.append(i)
                past_curve_distance_sum = current_dist

                current_points = bezier_info[current_curve]
                current_segments = curve_segments[current_curve]

        nodes_map.append(len(velocities))

        return positions, velocities, accelerations, headings, times, nodes_map
        

    def triple_pass_curve(self, v_max, a_max, j_max, dt, bezier_info):
        # Bezier info represents a list of bezier curve points with their length prolly
        positions = [0]
        headings = []
        times = [0]
        velocities = [0]
        accelerations = [0]
        nodes_map = [0]
        
        # Forward pass for acceleration
        current_dist = 0
        current_curve = 0
        past_curve_distance_sum = 0

        current_segments = createCurveSegments(bezier_info[current_curve][0], bezier_info[current_curve][2], bezier_info[current_curve][1])

        i = 0
        past_acceleration = 0
        while(current_curve < len(bezier_info) - 1):
            maximum_velocity = v_max
            if (len(bezier_info[current_curve]) == 3):
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                             bezier_info[current_curve][0], bezier_info[current_curve][1], 
                                                             bezier_info[current_curve][2])
            else:
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                             bezier_info[current_curve][0], bezier_info[current_curve][1], 
                                                             bezier_info[current_curve][2], bezier_info[current_curve][3])
            desired_accel = (maximum_velocity - velocities[i-1]) / dt
            max_accel_change = j_max * dt
            if (desired_accel < 0):
                accelerations.append(max(desired_accel, past_acceleration - max_accel_change))
            else:
                accelerations.append(min(desired_accel, past_acceleration + max_accel_change))

            print(current_dist, " ", past_acceleration, " ", accelerations[i-1])
            past_acceleration = accelerations[i-1]
            velocities.append(velocities[i-1] + accelerations[i-1] * dt)
            current_dist += velocities[i]*dt

            if (current_dist > current_segments[-1] + past_curve_distance_sum):
                current_curve += 1
                past_curve_distance_sum = current_dist
                if (len(bezier_info[current_curve]) == 3):
                    current_segments = createCurveSegments(bezier_info[current_curve][0], 
                                                           bezier_info[current_curve][2], 
                                                           bezier_info[current_curve][1])
                else:
                    current_segments = createCurveSegments(bezier_info[current_curve][0], 
                                                           bezier_info[current_curve][2], 
                                                           bezier_info[current_curve][3], 
                                                           bezier_info[current_curve][1])
            i += 1
        
        current_dist = past_curve_distance_sum + current_segments[-1]
        current_curve = len(bezier_info)-1
        past_curve_distance_sum = past_curve_distance_sum

        current_segments = createCurveSegments(bezier_info[current_curve][0], bezier_info[current_curve][2], bezier_info[current_curve][1])

        # IMPORTANT Velocity MUST end at 0
        velocities[-1] = 0
        accelerations[-1] = 0

        # Backward pass for acceleration
        for i in range(len(velocities) - 2, -1, -1):
            # Set dt to a value based on how long it will take to cover x distance
            maximum_velocity = v_max
            if (len(bezier_info[current_curve]) == 3):
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                             bezier_info[current_curve][0], bezier_info[current_curve][1], 
                                                             bezier_info[current_curve][2])
            else:
                maximum_velocity = self.distance_to_max_velocity(v_max, current_dist-past_curve_distance_sum, current_segments,
                                                             bezier_info[current_curve][0], bezier_info[current_curve][1], 
                                                             bezier_info[current_curve][2], bezier_info[current_curve][3])
            desired_accel = (velocities[i]-velocities[i+1]) / dt
            max_accel_change = j_max * dt
            if (desired_accel < 0):
                accelerations[i] = max(max(desired_accel, accelerations[i+1] - max_accel_change), -a_max)
            else:
                accelerations[i] = min(min(desired_accel, accelerations[i+1] + max_accel_change), a_max)
            print(velocities[i+1], " ", velocities[i], " ", accelerations[i])
            # BRING DOWN ACCELERATION DONT HAVE MAX CHANGE OR ANYTHING LFIKE THAT
            # print(desired_accel, " ", accelerations[i])
            # accelerations[i] = min(max(desired_accel, 
            #                         accelerations[i+1] - max_accel_change if i < len(accelerations)-1 else -a_max, 
            #                         -a_max), 
            #                     accelerations[i+1] + max_accel_change if i < len(accelerations)-1 else a_max, 
            #                     a_max)
            velocities[i] = velocities[i+1] + accelerations[i] * dt
            current_dist -= velocities[i+1]*dt
            if (current_dist < past_curve_distance_sum
                and current_curve < len(bezier_info) - 1):
                current_curve -= 1
                past_curve_distance_sum = current_dist
                if (len(bezier_info[current_curve]) == 3):
                    current_segments = createCurveSegments(bezier_info[current_curve][0], 
                                                           bezier_info[current_curve][2], 
                                                           bezier_info[current_curve][1])
                else:
                    current_segments = createCurveSegments(bezier_info[current_curve][0], 
                                                           bezier_info[current_curve][2], 
                                                           bezier_info[current_curve][3], 
                                                           bezier_info[current_curve][1])
        
        # Final forward pass to smooth velocities

        current_dist = 0
        current_curve = 0
        past_curve_distance_sum = 0

        current_segments = createCurveSegments(bezier_info[current_curve][0], bezier_info[current_curve][2], bezier_info[current_curve][1])

        for i in range(1, len(velocities)):
            # TODO Replace these with the proper velocity and acceleration formulas based on jerk
            # velocities[i] = velocities[i-1] - accelerations[i-1] * dt
            positions.append(positions[i-1] + velocities[i-1]*dt)
            # print(positions[-1], " ", velocities[i-1]*dt)
            times.append(times[-1] + dt)


            if (len(bezier_info[current_curve]) == 3):
                headings.append(getHeading(positions[i], current_segments, \
                bezier_info[current_curve][0], bezier_info[current_curve][2], bezier_info[current_curve][1]))
            else:
                headings.append(getHeading(positions[i], current_segments, \
                bezier_info[current_curve][0], bezier_info[current_curve][2], bezier_info[current_curve][3], bezier_info[current_curve][1]))

            current_dist = positions[i]

            if (current_dist > current_segments[-1] + past_curve_distance_sum
                and current_curve < len(bezier_info) - 1):
                current_curve += 1
                nodes_map.append(i)
                past_curve_distance_sum = current_dist
                if (len(bezier_info[current_curve]) == 3):
                    current_segments = createCurveSegments(bezier_info[current_curve][0], 
                                                           bezier_info[current_curve][2], 
                                                           bezier_info[current_curve][1])
                else:
                    current_segments = createCurveSegments(bezier_info[current_curve][0], 
                                                           bezier_info[current_curve][2], 
                                                           bezier_info[current_curve][3], 
                                                           bezier_info[current_curve][1])
        nodes_map.append(len(velocities))
        return positions, velocities, accelerations, headings, times, nodes_map
    
    # def generate_all(self, velocities, dt):
    #     positions = [0]
    #     headings = [0]
    #     times = [0]
    #     for i in range(0, len(velocities)-1): # -1 to make sure all lists have an equal size
    #         positions.append(positions[-1] + velocities[i]*dt)
    #         headings.append(getHeading())
    #         times.append(times[-1] + dt)


    def calculateMotionProfile(self, v_max=20, a_max=8, j_max=45):
        # use triple pass curve function to curve out profile
        bezier_info = []
        for data in self.line_data:
            if (len(data) == 3): # Quadratic bezier curve
                bezier_info.append([data[0], data[2], data[1]])
            else: # Cubic bezier curve
                bezier_info.append([data[0], data[2], data[3], data[1]])

        positions, velocities, accelerations, headings, times, nodes_map\
        = self.mushingprufil(v_max, a_max, j_max, bezier_info)

        self.all_positions = positions
        self.all_velocities = velocities
        self.all_accelerations = accelerations
        self.all_headings = headings
        self.all_time_intervals = times
        self.all_nodes_map = nodes_map
        return self.all_time_intervals, self.all_positions, self.all_velocities, self.all_accelerations, self.all_headings, \
        self.all_nodes_map
        # positions, headings, times = self.generate_all(velocities, 0.0005)

    def calculateScurveStuff(self, v_max=20, a_max=8, j_max=45):
        # return self.calculateMotionProfile()
        self.all_time_intervals = []
        self.all_positions = []
        self.all_velocities = []
        self.all_velocity_limits = [] # Max velocity based on curvature at that point
        self.all_accelerations = []
        self.all_headings = []
        self.all_nodes_map = [] # Represents index of node n in any of the above lists
        
        current_position = 0
        segment_data = [[], []]
        segment_length = 0
        print("POINT LEN: ", len(self.line_data))
        for i in range (0, len(self.line_data)):
            print(i)
            line = self.line_data[i]
            line.append(None) # Prevents out of range error
            segments = createCurveSegments(line[0], line[1], line[2], line[3])

            segment_data[0].append(line)
            segment_data[1].append(segments)
            segment_length += segments[-1]
            print("SEGMENT LENGTH: ", len(segment_data[0]))
            print(segment_data[1])
            print(segment_data[0])
            print(self.parent.nodes[i+1].hasAction, " ", self.parent.nodes[i+1].isEndNode)
            if ((not self.parent.nodes[i+1].isEndNode) and (not self.parent.nodes[i+1].hasAction)):
                continue
            time_intervals, positions, velocities, velocity_limits, accelerations, headings, nodes_map = self.generate_scurve_profile(segment_length, segment_data[1], segment_data[0],
                                                                                                                     v_max, a_max, j_max)
            # self.all_time_intervals.extend(time_intervals + (self.all_time_intervals[-1] if self.all_time_intervals else 0))
            # print(time_intervals)
            if (self.all_time_intervals != []):
                self.all_time_intervals.extend([time + self.all_time_intervals[-1] for time in time_intervals])
            else:
                self.all_time_intervals = time_intervals
            self.all_positions.extend([p + current_position for p in positions])
            self.all_velocities.extend(velocities)
            self.all_velocity_limits.extend(velocity_limits)
            self.all_accelerations.extend(accelerations)
            self.all_headings.extend(headings)
            self.all_nodes_map.extend((mapping+len(self.all_time_intervals)-len(time_intervals)) for mapping in nodes_map)
            current_position += segment_length
            segment_data = [[], []]
            segment_length = 0
            # print(self.all_time_intervals)
        self.all_nodes_map.append(len(self.all_time_intervals))

        # self.all_positions, self.all_velocities, self.all_accelerations, self.all_time_intervals = \
        # self.combineVelocities(self.all_velocities, self.all_velocity_limits, v_max, a_max, j_max)

        return self.all_time_intervals, self.all_positions, self.all_velocities, self.all_accelerations, self.all_headings, self.all_nodes_map

    def combineVelocities(self, velocities, velocity_limits, v_max, a_max, j_max, dt=0.0005):
        adjusted_velocities = []
        for i in range(0, len(velocities)):
            adjusted_velocities.append(min(velocities[i], velocity_limits[i]))
        adjusted_velocities = self.adjust_velocities(adjusted_velocities, 0.0005, a_max, j_max)
        positions = [0]
        accelerations = [0]
        for i in range(0, len(adjusted_velocities)-1):
            positions.append(positions[-1] + adjusted_velocities[i])
            accelerations.append(adjusted_velocities[i+1]-adjusted_velocities[i])

        time_intervals = [0]
        for i in range(0, len(adjusted_velocities)-1):
            time_intervals.append(time_intervals[-1] + dt)
        return positions, adjusted_velocities, accelerations, time_intervals
    
    def check_constraints(self, dt, a_max, j_max, velocities, accelerations, jerks):
        

        print(accelerations, " ", a_max)
        # for a in accelerations:
            # print(a, end=" ")
        print(jerks, " ", j_max)
        

        a_max_violations = np.abs(accelerations) - a_max > 0.01
        j_max_violations = np.abs(jerks) - j_max > 0.01
        # print(a_max_violations)
        
        return a_max_violations, j_max_violations

    def recompute_velocities(self, velocities, start_index, end_index, previous_velocity, dt, a_max, j_max):
        # if (end_index-start_index > 2):
            # print(end_index-start_index)
        current_velocity = velocities[start_index]
        new_velocities = [current_velocity]
        
        for i in range(start_index + 1, end_index + 1):
            desired_velocity = velocities[i]
            # desired_velocity = velocities[end_index]
            delta_v = desired_velocity - current_velocity

            if np.abs(delta_v / dt) > a_max:
                delta_v = np.sign(delta_v) * a_max * dt
            # print(delta_v)
            
            new_velocity = current_velocity + delta_v
            
            if len(new_velocities) > 1:
                previous_velocity = new_velocities[-2]
            delta_a = \
            ((new_velocity - current_velocity)/dt - (current_velocity - previous_velocity)/dt)/dt
            # delta_a = (new_velocity - 2 * current_velocity + previous_velocity) / (dt ** 2)
            # print(delta_a)
            if np.abs(delta_a) > j_max:
                delta_a = np.sign(delta_a) * j_max * dt ** 2
                new_velocity = 2 * current_velocity - previous_velocity + delta_a * dt ** 2
            
            new_velocities.append(new_velocity)
            current_velocity = new_velocity
        # print()
        return new_velocities
    
    def recompoot_velocities(self, sign, target_velocity, previous_velocity, curr_acc, t_acc_1, t_const_acc, dt, j_max):
        # Iterate through the three phases, applying the conditions at each timestep
        idx = 0
        new_velocities = [previous_velocity]
        print("FIRST ACC: ", curr_acc)
        while (new_velocities[-1] < target_velocity or (len(new_velocities) > 1 and abs(new_velocities[-1]-new_velocities[-2]) > 0.00001)):
            if (idx < t_acc_1): # Phase 1 increase acceleration
                print("INCREASE: ", idx, end=" ")
                curr_acc += sign * j_max * dt
            elif (idx > t_acc_1 + t_const_acc): # Phase 3 decrease acceleration
                print("DECREASE: ", 2*t_acc_1-idx + dt/2, end=" ")
                curr_acc -= sign * j_max * dt
            new_velocities.append(new_velocities[-1] + (curr_acc*dt)) # Update new velocity in list
            print("Curracc: ", curr_acc, " ",  new_velocities[-1]-new_velocities[-2], " ", new_velocities[-1], " ", target_velocity)
            idx += dt
        return new_velocities

    def adjust_velocities(self, velocities, dt, a_max, j_max):
        velocities = np.array(velocities)
        npvelocities = np.array(velocities)
        npaccelerations = np.diff(npvelocities) / dt
        npjerks = np.diff(npaccelerations) / dt
        a_max_violations, j_max_violations = self.check_constraints(dt, a_max, j_max,
                                                                    npvelocities, npaccelerations, npjerks)
        
        # Shift j_max_violations by one index to align with a_max_violations
        j_max_violations = np.roll(j_max_violations, 1)
        
        # Add a padding false at the end to make the lengths equal
        a_max_violations = np.append(a_max_violations, False)
        j_max_violations = np.append(j_max_violations, [False, False])
        
        violation_indices = np.where(a_max_violations | j_max_violations)[0]
        # print(violation_indices)
        
        adjusted_velocities = velocities.copy()
        
        corrected_until = len(adjusted_velocities)  # Track the furthest back correction point
        offset = 0
        for violation_index in reversed(violation_indices):
            if violation_index >= corrected_until:
                continue  # Skip already corrected points
            
            # TODO
            # - Split the segments if we switch from too high of a jerk to too low of a jerk (same with acc)
            # - Make sure if we need to go down to a velocity we reverse our phases and change signs in recompoot_velocities()
            # - Why are the values ending up so high?

            # - Use distance instead of time to generate minicurves
            # - If time flat is negative recalculate max acceleration

            start_index = int(max(0, violation_index - 1))
            
            t_acc_1 = None
            t_acc_2 = None
            t_const_acc = None

            start_acc = None
            past_violation_type = np.sign(npaccelerations[violation_index])
            print("SIGN: ", past_violation_type)
            while (start_index > 0 and start_index < len(velocities)-1):
                sp = min(start_index, violation_index)
                ep = max(start_index, violation_index)
                start_velocity = adjusted_velocities[sp]
                start_acc = (adjusted_velocities[sp+1] - adjusted_velocities[sp])/dt

                target_velocity = adjusted_velocities[ep]

                sign = past_violation_type
                #                          _
                # Acceleration looks like / \
                # Change acceleration till we reach +- max accleration
                # Constant acceleration phase
                # Change acceleration back to 0
                
                # Calculate end velocity of above phases
                t_acc_1 = (a_max-start_acc)/j_max # Time for phase 1
                t_acc_2 = (a_max)/j_max # Time for phase 2

                # (0.5 * jerk * time**2) + (initial_acceleration * time)
                v_acc_1 = ((sign*j_max)*t_acc_1**2)/2 + (start_acc*t_acc_1)
                v_acc_2 = ((sign*-1*j_max)*t_acc_2**2)/2 + ((sign*a_max)*t_acc_2)

                t_const_acc = (target_velocity - (start_velocity + v_acc_1 + v_acc_2))/a_max
                # TODO 
                # - if negative recalculate a new max acceleration
                # - calculate t const based on distance and correct based on distance as well
                print("INFER: ", sp, " ", ep, " ", start_velocity, " ", target_velocity, " ", 
                      v_acc_1, " ", v_acc_2)
                # print("INFO: ", t_acc_1 + t_acc_2, " ", v_acc_1 + v_acc_2, " ", 
                #       target_velocity, " ", start_velocity + v_acc_1 + v_acc_2, " ", t_const_acc)
                if (t_const_acc >= 0 and 
                    not((a_max_violations[sp - 1] or j_max_violations[sp - 1]))):
                    break
                violation_type = np.sign(npaccelerations[sp])
                if ((a_max_violations[sp] or j_max_violations[sp]) and past_violation_type != violation_type):
                    print("DIF TYPE: ", npaccelerations[sp], " ", npaccelerations[ep])
                    break
                start_index += int(sign)
            # print("TIME INTERVALS: ", t_acc_1, " ", t_acc_2, " ", t_const_acc)
            if (t_const_acc < 0 or abs(violation_index-start_index) < 2): # Impossible to fix that issue
                continue
            # print(violation_index-start_index, " ", start_index, " ", violation_index, " "
            #       , adjusted_velocities[start_index], " ", adjusted_velocities[violation_index])
            previous_velocity = 0
            if (start_index > 0):
                previous_velocity = adjusted_velocities[min(start_index, violation_index)] # -1
            
            # new_velocities = self.recompute_velocities(adjusted_velocities, start_index, violation_index,
            #                                            previous_velocity, dt, a_max, j_max)
            print("SHABOOF: ", (min(start_index, violation_index), " ", max(start_index, violation_index)), 
                   " ", t_acc_1, " ", t_acc_2, " ", t_const_acc)
            new_velocities = self.recompoot_velocities(sign, target_velocity
                                                       , previous_velocity, start_acc, t_acc_1, t_const_acc, dt, j_max)
            
            print(len(new_velocities), " ", max(start_index, violation_index)-min(start_index, violation_index))
            
            adjusted_velocities = np.concatenate((adjusted_velocities[:min(start_index, violation_index)], new_velocities, adjusted_velocities[max(start_index, violation_index) + 1:]))

            # adjusted_velocities[min(start_index, violation_index):max(start_index, violation_index) + 1] = new_velocities
            offset = len(velocities) - max(start_index, violation_index)-min(start_index, violation_index)
            corrected_until = min(start_index, violation_index)  # Update the furthest back correction point
        
        # for a in adjusted_velocities:
        #     print(a, end=" ")
        # print()
        return adjusted_velocities

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

    # Function to calculate the distance covered for given v_max
    def calculate_total_distance(self, v_max, a_max, j_max):
        t_jerk = a_max / j_max
        v_after_jerk1 = 0.5 * j_max * t_jerk**2
        t_acc = (v_max - v_after_jerk1*2) / a_max
        
        d_jerk1 = (j_max*t_jerk**3)/6
        d_acc = (a_max*t_acc**2)/2 + ((j_max*t_jerk**2)/2)*t_acc**1
        d_jerk2 = ((-j_max)*t_jerk**3)/6 + (a_max*t_jerk**2)/2 + ((j_max*t_jerk**2)/2 + t_acc*a_max)*t_jerk
        
        total_distance = 2 * (d_jerk1 + d_acc + d_jerk2)
        return total_distance


    # Bisection method to find the correct v_max
    def find_correct_v_max(self, distance, a_max, j_max, v_max_initial, tolerance=1e-11):
        v_max_low = 0
        v_max_high = 1e9
        v_max = v_max_low + (v_max_high - v_max_low) / 2
        
        while abs(self.calculate_total_distance(v_max, a_max, j_max) - distance) > tolerance:
            if self.calculate_total_distance(v_max, a_max, j_max) > distance:
                v_max_high = v_max
            else:
                v_max_low = v_max
            v_max = v_max_low + (v_max_high - v_max_low) / 2
        
        return v_max

    def find_tjerk(self, distance, t_jerk, j_max, tolerance=1e-11):
        l = 0
        r = 1e9
        c = 0
        while (l <= r):
            mid = l + (r-l)/2
            a_max = j_max*mid
            d_jerk1 = (j_max*mid**3)/6
            d_jerk2 = ((-j_max)*mid**3)/6 + (a_max*mid**2)/2 + ((j_max*mid**2)/2)*mid
            dist = d_jerk1 + d_jerk2
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

    def generate_scurve_profile(self, distance, segments, line_data, v_max, a_max, j_max, K=10.0, dt=0.0005):
        original_vmax = v_max
        # v_acc_2 = ((sign*-1*j_max)*t_acc_2**2)/2 + ((sign*a_max)*t_acc_2)
        t_jerk = a_max / j_max
        t_acc = (v_max - j_max * t_jerk**2) / a_max

        d_acc = (a_max*t_acc**2)/2 + ((j_max*t_jerk**2)/2)*t_acc**1
        d_jerk1 = (j_max*t_jerk**3)/6
        d_jerk2 = ((-j_max)*t_jerk**3)/6 + (a_max*t_jerk**2)/2 + ((j_max*t_jerk**2)/2 + t_acc*a_max)*t_jerk
        # print("TJERK: ", t_jerk)
        t_flat = (distance - 2.0 * (d_jerk1 + d_acc + d_jerk2)) / v_max
        if t_flat < 0:
            v_max = self.find_correct_v_max(distance, a_max, j_max, v_max)
            t_jerk = a_max / j_max
            t_acc = (v_max - (j_max*t_jerk**2)) / a_max
            t_flat = 0

            d_jerk1 = (j_max*t_jerk**3)/6
            d_jerk2 = ((-j_max)*t_jerk**3)/6 + (a_max*t_jerk**2)/2 + ((j_max*t_jerk**2)/2 + t_acc*a_max)*t_jerk
            if (t_acc < 0):
                t_jerk = self.find_tjerk(distance/2, t_jerk, j_max)
                t_acc = 0
                d_acc = 0
                d_flat = 0
                a_max = t_jerk*j_max*2


        total_time = 4 * t_jerk + 2 * t_acc + t_flat
        time_intervals = np.arange(0, total_time, dt)
        positions, velocities, velocity_limits, accelerations, headings, nodes_map = [], [], [], [], [], []
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
        debug = False

        curr_segment = 0
        nodes_map.append(0)
        prev_dist = 0
        time_intervals = [0]
        # Make sure you stay in the stage until the velocity is correctamundo
        # for i in range(len(time_intervals)): # Switch to a while loop
            # print(i)
        while (time_intervals[-1] < 4 * t_jerk + 2*t_acc + t_flat):
        # while (abs(distance-s) > 0.01):
            
            current_length = s-prev_dist
            t = time_intervals[-1]

            pts = line_data[curr_segment]
            if (pts[-1] == None):
                pts = pts[0:len(pts)-1]
            t_along_curve = distToTime(current_length, segments[curr_segment])
            curvature = None
            if (len(pts) < 4): # Quadratic
                curvature = quadratic_bezier_curvature(pts[0], pts[2], pts[1], t_along_curve)
            else:
                curvature = cubic_bezier_curvature(pts[0], pts[2], pts[3], pts[1], t_along_curve)

            adjusted_vmax = max_speed_based_on_curvature(curvature, original_vmax, K)
            # print(curvature, " " , adjusted_vmax)
            # print(curvature, " ", t_along_curve, " ", adjusted_vmax, end=" ")
            # print(bezier_derivative(t_along_curve, pts), " ", bezier_second_derivative(t_along_curve, pts))
            velocity_limits.append(adjusted_vmax)
            # adjusted_vmax = 1e9
            # if (abs(v_max-adjusted_vmax) > 0.5):
            # print(current_length,  " ", adjusted_vmax, " ", v_max, " ", t_along_curve, " ", curvature)

            if t < t_jerk:
                # First jerk phase (increasing acceleration)
                if (debug):
                    print("First jerk phase: ", d_jerk1, end=' ')
                s += (j_max*dt**3)/6 + (a*dt**2)/2 + v*dt
                v += (j_max*dt**2)/2 + a*dt
                # if (v > adjusted_vmax):
                #     v = adjusted_vmax
                a += j_max * dt
            elif t < t_jerk + t_acc:
                # First acceleration phase (constant acceleration)
                if (debug):
                    print("First acceleration phase: ", d_jerk1+d_acc, end=' ')
                s += (a*dt**2)/2 + v*dt
                v += a*dt
                # if (v > adjusted_vmax):
                #     v = adjusted_vmax
                a = a_max
            elif t < 2 * t_jerk + t_acc:
                # Second jerk phase (decreasing acceleration)
                if (debug):
                    print("Second jerk phase (negative): ", d_jerk1+d_jerk2+d_acc, end=' ')
                s += ((-j_max)*dt**3)/6 + (a*dt**2)/2 + v*dt
                v += ((-j_max)*dt**2)/2 + a*dt
                # if (v > adjusted_vmax):
                #     v = adjusted_vmax
                a += -j_max * dt
            elif t < 2 * t_jerk + t_acc + t_flat:
                # Constant velocity phase
                if (debug):
                    print("Constant velocity phase: ", d_jerk1+d_jerk2+d_acc+d_flat, end=' ')
                s += v*dt
                v = v_max
                # if (v > adjusted_vmax):
                #     v = adjusted_vmax
                a = 0
            elif t < 3 * t_jerk + t_acc + t_flat:
                # Third jerk phase (decreasing acceleration)
                if (debug):
                    print("Third jerk phase(negative): ", 2*d_jerk2+d_jerk1+d_acc+d_flat, end=' ')
                s += ((-j_max)*dt**3)/6 + (a*dt**2)/2 + v*dt
                v += ((-j_max)*dt**2)/2 + a*dt
                # if (v > adjusted_vmax):
                #     v = adjusted_vmax
                a += -j_max * dt
            elif t < 3 * t_jerk + 2*t_acc + t_flat:
                # Second acceleration phase (constant negative acceleration)
                if (debug):
                    print("Second acceleration phase: ", 2*d_jerk2+d_jerk1+2*d_acc+d_flat, end=' ')
                s += (a*dt**2)/2 + v*dt
                v += a*dt
                # if (v > adjusted_vmax):
                #     v = adjusted_vmax
                a = -a_max
            elif t < 4 * t_jerk + 2*t_acc + t_flat:
                # Fourth jerk phase (increasing acceleration)
                if (debug):
                    print("Fourth jerk phase: ", 2*d_jerk2+2*d_jerk1+2*d_acc+d_flat, end=' ')
                s += (j_max*dt**3)/6 + (a*dt**2)/2 + v*dt # Derive posiotion based on jerk as well
                v += (j_max*dt**2)/2 + a*dt # Derive velocity based on jerk
                # if (v > adjusted_vmax):
                #     v = adjusted_vmax
                a += j_max * dt
            if (debug):
                print(s, " ", v, " ", a, " ", t)
            positions.append(s)
            velocities.append(v)
            accelerations.append(a)

            # MAKE DEPENDENT ON DISTANCE
            if (s-prev_dist > segments[curr_segment][-1] and curr_segment < len(segments)-1):
                nodes_map.append(len(time_intervals))
                # nodes_map.append(i)
                curr_segment += 1
                prev_dist = s
            headings.append(getHeading(s-prev_dist, segments[curr_segment], 
                                       line_data[curr_segment][0], line_data[curr_segment][1], line_data[curr_segment][2], line_data[curr_segment][3]))
            
            time_intervals.append(time_intervals[-1]+dt)

        # nodes_map.append(len(line_data))
        time_intervals = time_intervals[1:]
        # print(time_intervals)
        return time_intervals, positions, velocities, velocity_limits, accelerations, headings, nodes_map

                
if __name__ == '__main__':
    app = QApplication(sys.argv)
    if getattr(sys, 'frozen', False):
        create_files()
        load_fonts()

    window = AutonomousPlannerGUIManager()
    window.show()

    sys.exit(app.exec())
