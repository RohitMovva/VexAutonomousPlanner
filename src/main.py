import sys
import os
import json
import yaml
from pathlib import Path
from PyQt6.QtWidgets import QApplication, QDialog, QLabel, QWidget, QVBoxLayout, QMenu, QInputDialog, QMainWindow, QTextEdit, QPushButton, QFileDialog
from PyQt6.QtGui import QPixmap, QMouseEvent, QPainter, QColor, QAction, QPen, QPainterPath, QFontDatabase
from PyQt6.QtCore import Qt, QPoint, QLineF, QPointF, Qt
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

def load_fonts():
    font_dir = os.path.join(os.path.dirname(__file__), 'fonts')
    print(font_dir)
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

# QUADRATIC
def quad_bezier_angle(t, P0, P1, P2):
    tangentX = (2*(1-t)*(P1.x()-P0.x())) + (2*t*(P2.x()-P1.x()))
    tangentY = (2*(1-t)*(P1.y()-P0.y())) + (2*t*(P2.y()-P1.y()))
    return -1*np.arctan2(tangentY, tangentX)*(180/np.pi)

def quadratic_bezier_point(P0, P1, P2, t):
    x = (1 - t)**2 * P0.x() + 2 * (1 - t) * t * P1.x() + t**2 * P2.x()
    y = (1 - t)**2 * P0.y() + 2 * (1 - t) * t * P1.y() + t**2 * P2.y()
    return x, y

# CUBIC
def cubic_bezier_point(P0, P1, P2, P3, t):
    x = (1 - t)**3 * P0.x() + 3 * (1 - t)**2 * t * P1.x() + 3 * (1 - t) * t**2 * P2.x() + t**3 * P3.x()
    y = (1 - t)**3 * P0.y() + 3 * (1 - t)**2 * t * P1.y() + 3 * (1 - t) * t**2 * P2.y() + t**3 * P3.y()
    return x, y

# 3(1- t)^2(P1 - P0) + 6(1 - t)t(P2 - P1) + 3t^2(P3 - P2)
def cubic_bezier_angle(t, P0, P1, P2, P3):
    tangentX = 3*(1-t)**2*(P1.x()-P0.x()) + 6*(1-t)*t*(P2.x()-P1.x()) + 3*t**2*(P3.x()-P2.x())
    tangentY = 3*(1-t)**2*(P1.y()-P0.y()) + 6*(1-t)*t*(P2.y()-P1.y()) + 3*t**2*(P3.y()-P2.y())
    return -1*np.arctan2(tangentY, tangentX)*(180/np.pi)

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
        segments.append(currlen*(12/699))

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
        self.hasAction = (self.spinIntake or self.clampGoal)
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
        elif self.hasAction:
            painter.setBrush(QColor("#1338BE"))
        else:
            painter.setBrush(QColor("#1F456E"))


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
        self.update()
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
        self.hasAction = (self.spinIntake or self.clampGoal)
        print(f"Spin Intake: {self.spinIntake}")

    def toggle_clamp_goal(self):
        self.clampGoal = not self.clampGoal
        self.hasAction = (self.spinIntake or self.clampGoal)
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

        self.setWindowTitle('Path Planner') # Totally not inspired by Pronounce that

        self.central_widget = DrawingWidget(self)
        self.setCentralWidget(self.central_widget)
        self.central_widget.setFixedSize(699, 699)

        self.nodes = []
        self.start_node = None
        self.end_node = None

        self.current_working_file = None
        self.routes_header_path = None
        self.routes_folder_path = None
        with open(resource_path('../config.yaml'), 'r') as file:
            config = yaml.safe_load(file)
        if (config == None):
            config = {}
        # Check if the repository_path exists in the config
        if 'autonomous_repository_path' not in config:
            autonomous_path = QFileDialog.getExistingDirectory(self, "Select Autonomous Program Directory", 
                                                      str(Path(os.getcwd()).parent.parent.absolute()))
            
            config['autonomous_repository_path'] = autonomous_path + "/routes.h"
            print(f"Added autonomous repository path: {autonomous_path}/routes.h")

        if 'routes_folder_path' not in config:
            routes_path = QFileDialog.getExistingDirectory(self, "Select Routes Folder", 
                                                      str(Path(os.getcwd()).parent.parent.absolute()))
            
            config['routes_folder_path'] = routes_path
            print(f"Added routes folder path: {routes_path}")
        
        self.routes_header_path = config['autonomous_repository_path']
        self.routes_folder_path = config['routes_folder_path']
        with open(resource_path('../config.yaml'), 'w') as file:
                yaml.safe_dump(config, file)

        self.layout = QVBoxLayout()

        self.label = ClickableLabel(parent=self.central_widget, gui_instance=self)

        pixmap = QPixmap(resource_path('../assets/top_down_cropped_high_stakes_field.png'))
        # self.label.setPixmap(pixmap)

        self.update()
        self.layout.addWidget(self.label)
        self.central_widget.setLayout(self.layout)

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

        create_cpp_action = QAction('Create C++ File from Nodes', self)
        create_cpp_action.triggered.connect(self.create_cpp_file)
        file_menu.addAction(create_cpp_action)

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
            full_path = self.routes_folder_path + self.current_working_file
        nodes_data = []
        time_intervals, positions, velocities, accelerations, headings, nodes_map = self.central_widget.calculateScurveStuff()
        for i in range(0, len(time_intervals), 200): # Every 100ms save data
            nodes_data.append([velocities[i], headings[i]])

        nodes_actions = [
                [
                    int(node.spinIntake),
                    int(node.clampGoal),
                    node.turn
                ]
                for node in self.nodes
            ]
        for i in range(0, len(nodes_map)):
            nodes_data.insert(int(nodes_map[i]/200), nodes_actions[i])

        self.fill_template(nodes_data)
        with open(full_path, 'w') as file:
            file.write(nodes_string)
        print(f"Route saved to {full_path}")

    def load_nodes_from_file(self, dialogue=True):
        file_name = None
        if (dialogue):
            file_name, _ = QFileDialog.getOpenFileName(self, "Load Route from File", "", "Text Files (*.txt);;All Files (*)")
        else:
            file_name = self.routes_folder_path + self.current_working_file
        if file_name != None:
            with open(file_name, 'r') as file:
                nodes_string = file.read()
            print(f"Route loaded from {file_name}")
            self.load_nodes(nodes_string)

    def set_working_file(self):
        file_name, ok = QInputDialog.getText(self, "File to save route to", "Enter file name (without extension):")
        if ok and file_name:
            self.current_working_file = file_name + ".txt"
            folder = self.routes_folder_path
            if (folder == None):
                folder = QFileDialog.getExistingDirectory(self, "Select Directory to Save File", 
                                                      str(Path(os.getcwd()).parent.absolute()))
                
            if folder:
                self.routes_folder_path = folder
                full_path = f"{folder}/{self.current_working_file}"
                if not os.path.exists(full_path):
                    with open(full_path, 'w+') as file: # Creates file if it isn't already created
                        pass
                print(f"Set current working file at {full_path}")
                if (len(self.nodes) > 0 or os.stat(full_path).st_size == 0):
                    self.auto_save()
                else:
                    self.load_nodes_from_file(False)

    def create_cpp_file(self):
        print("Creating C++ file from nodes...") # This is a lie

    def clear_nodes(self):
        while self.nodes:
            node = self.nodes.pop()
            node.delete_node()
        self.start_node = None
        self.end_node = None
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
                node.turn
            ]
            for node in self.nodes
        ]
        if (as_list):
            return nodes_data
        nodes_string = json.dumps(nodes_data, separators=(',', ':'))
        return nodes_string
    
    def auto_save(self):
        if (self.current_working_file != None and self.start_node and self.end_node):
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
        insertion = f"std::vector<std::vector<double>> {self.current_working_file} = {{{', '.join(stringified)}}};\n"
        
        try:
            # Read the content of routes.h
            with open(self.routes_header_path, "r") as routes_file:
                content = routes_file.readlines()
            
            # Find the line with the specified route name
            inserted = False
            for i, line in enumerate(content):
                if line.strip().startswith(f"std::vector<std::vector<double>> {self.current_working_file} ="):
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

    def position_graph(self):
        self.central_widget.calculateScurveStuff()
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
        self.central_widget.calculateScurveStuff()
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
        self.central_widget.calculateScurveStuff()
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
        self.central_widget.calculateScurveStuff()
        plt.figure(figsize=(12, 8))

        # Heading profile
        plt.subplot(3, 1, 1)
        plt.plot(self.central_widget.all_positions, self.central_widget.all_headings)
        plt.title('Heading Profile')
        plt.xlabel('Position (ft)')
        plt.ylabel('Heading (degrees)')


        plt.tight_layout()
        plt.show()

class DrawingWidget(QWidget):
    def __init__(self, parent=None, image_path=resource_path('../assets/top_down_cropped_high_stakes_field.png')):
        super().__init__(parent)
        self.parent = parent
        self.setGeometry(0, 0, 699, 699)
        self.image = QPixmap(image_path) if image_path else None
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)

    def calculateScurveStuff(self):
        self.all_time_intervals = []
        self.all_positions = []
        self.all_velocities = []
        self.all_accelerations = []
        self.all_headings = []
        self.all_nodes_map = [] # Represents index of node n in any of the above lists
        
        current_position = 0
        segment_data = [[], []]
        segment_length = 0
        for i in range (0, len(self.line_data)):
            line = self.line_data[i]
            line.append(None) # Prevents out of range error
            segments = createCurveSegments(line[0], line[1], line[2], line[3])

            segment_data[0].append(line)
            segment_data[1].append(segments)
            segment_length += segments[-1]
            if ((not self.parent.nodes[i+1].hasAction) and i < len(self.line_data)-1):
                continue
            time_intervals, positions, velocities, accelerations, headings, nodes_map = self.generate_scurve_profile(segment_length, segment_data[1], segment_data[0])
            self.all_time_intervals.extend(time_intervals + (self.all_time_intervals[-1] if self.all_time_intervals else 0))
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
        self.line_data = []
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

    def generate_scurve_profile(self, distance, segments, line_data, v_max=.25, a_max=0.125, j_max=0.08, dt=0.0005):
        t_jerk = a_max / j_max
        t_acc = (v_max - j_max * t_jerk**2) / a_max

        d_acc = (a_max*t_acc**2)/2 + ((j_max*t_jerk**2)/2)*t_acc**1
        d_jerk1 = (j_max*t_jerk**3)/6
        d_jerk2 = ((-j_max)*t_jerk**3)/6 + (a_max*t_jerk**2)/2 + ((j_max*t_jerk**2)/2 + t_acc*a_max)*t_jerk

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
        positions, velocities, accelerations, headings, nodes_map = [], [], [], [], []
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
        for i in range(len(time_intervals)):
            t = time_intervals[i]
            if t < t_jerk:
                # First jerk phase (increasing acceleration)
                if (debug):
                    print("First jerk phase: ", d_jerk1, end=' ')
                s += (j_max*dt**3)/6 + (a*dt**2)/2 + v*dt
                v += (j_max*dt**2)/2 + a*dt
                a += j_max * dt
            elif t < t_jerk + t_acc:
                # First acceleration phase (constant acceleration)
                if (debug):
                    print("First acceleration phase: ", d_jerk1+d_acc, end=' ')
                s += (a*dt**2)/2 + v*dt
                v += a*dt
                a = a_max
            elif t < 2 * t_jerk + t_acc:
                # Second jerk phase (decreasing acceleration)
                if (debug):
                    print("Second jerk phase (negative): ", d_jerk1+d_jerk2+d_acc, end=' ')
                s += ((-j_max)*dt**3)/6 + (a*dt**2)/2 + v*dt
                v += ((-j_max)*dt**2)/2 + a*dt
                a += -j_max * dt
            elif t < 2 * t_jerk + t_acc + t_flat:
                # Constant velocity phase
                if (debug):
                    print("Constant velocity phase: ", d_jerk1+d_jerk2+d_acc+d_flat, end=' ')
                s += v*dt
                v = v_max
                a = 0
            elif t < 3 * t_jerk + t_acc + t_flat:
                # Third jerk phase (decreasing acceleration)
                if (debug):
                    print("Third jerk phase(negative): ", 2*d_jerk2+d_jerk1+d_acc+d_flat, end=' ')
                s += ((-j_max)*dt**3)/6 + (a*dt**2)/2 + v*dt
                v += ((-j_max)*dt**2)/2 + a*dt
                a += -j_max * dt
            elif t < 3 * t_jerk + 2*t_acc + t_flat:
                # Second acceleration phase (constant negative acceleration)
                if (debug):
                    print("Second acceleration phase: ", 2*d_jerk2+d_jerk1+2*d_acc+d_flat, end=' ')
                s += (a*dt**2)/2 + v*dt
                v += a*dt
                a = -a_max
            elif t < 4 * t_jerk + 2*t_acc + t_flat:
                # Fourth jerk phase (increasing acceleration)
                if (debug):
                    print("Fourth jerk phase: ", 2*d_jerk2+2*d_jerk1+2*d_acc+d_flat, end=' ')
                s += (j_max*dt**3)/6 + (a*dt**2)/2 + v*dt # Derive posiotion based on jerk as well
                v += (j_max*dt**2)/2 + a*dt # Derive velocity based on jerk
                a += j_max * dt
            if (debug):
                print(s, " ", v, " ", a, " ", t)
            positions.append(s)
            velocities.append(v)
            accelerations.append(a)

            # MAKE DEPENDENT ON DISTANCE
            if (s > segments[curr_segment][-1] and len(segments)-curr_segment > 1):
                nodes_map.append(i)
                curr_segment += 1
            headings.append(getHeading(s, segments[curr_segment], 
                                       line_data[curr_segment][0], line_data[curr_segment][1], line_data[curr_segment][2], line_data[curr_segment][3]))

        # nodes_map.append(len(line_data))
        return time_intervals, positions, velocities, accelerations, headings, nodes_map

                
if __name__ == '__main__':
    app = QApplication(sys.argv)
    if getattr(sys, 'frozen', False):
        create_files()
        load_fonts()

    window = AutonomousPlannerGUIManager()
    window.show()

    sys.exit(app.exec())
