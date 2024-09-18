import os
import json
from pathlib import Path
from PyQt6.QtWidgets import QApplication, QDialog, QVBoxLayout, QInputDialog, QMainWindow, QTextEdit, QPushButton, QFileDialog, QGraphicsScene, QGraphicsView
from PyQt6.QtGui import QAction, QIcon, QKeySequence, QTransform, QShortcut
from PyQt6.QtCore import Qt, pyqtSlot
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *
from gui.click_listener import *
from gui.path import *
from gui.settings import *
from gui.node import *
from utilities import *

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
    factor = 1.5
    def __init__(self, parent=None):
        super(AutonomousPlannerGUIManager, self).__init__(parent)

        self.setWindowTitle('Path Planner') # Totally not inspired by Pronounce that
        self.setWindowIcon(QIcon(resource_path('../assets/windows_icon.ico')))
        self.layout = QVBoxLayout()

        # self.nodes = []
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
        if routes_path == None:
            routes_path = QFileDialog.getExistingDirectory(self, "Select Routes Folder", 
                                                      str(Path(os.getcwd()).parent.parent.absolute()))
            
            setConfigValue('routes_folder_path', routes_path)
            print(f"Added routes folder path: {routes_path}")
        
        self.routes_header_path = autonomous_path
        self.routes_folder_path = routes_path

        self.max_velocity = getConfigValue('max_velocity')
        self.max_acceleration = getConfigValue('max_acceleration')
        self.max_jerk = getConfigValue('max_jerk')

        self.track_width = getConfigValue('track_width')

        # Image and path widget
        self.central_widget = PathWidget(self)

        self.setCentralWidget(self.central_widget)
        self.central_widget.show()

        # Settings Dock Widget
        self.settings_dock_widget = SettingsDockWidget(self.max_velocity, self.max_acceleration, self.max_jerk, self)
        
        # Add widgets to layout
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        # self.layout.addWidget(self.label)

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

    # def index_of(self, node):
    #     return (self.nodes.index(node))
    
    def update_coordinate_display(self, x, y):
        self.settings_dock_widget.set_current_coordinates(x, y)
    
    def set_max_velocity(self, new_velocity):
        self.max_velocity = new_velocity
    def set_max_acceleration(self, new_acceleration):
        self.max_acceleration = new_acceleration
    def set_max_jerk(self, new_jerk):
        self.max_jerk = new_jerk

    def save_nodes_to_string(self):
        nodes_string = self.convert_nodes()
        print("Nodes saved as string:", nodes_string)
        dialog = SaveNodesDialog(nodes_string, self)
        dialog.exec()

    def load_nodes_from_string(self):
        nodes_string, ok = QInputDialog.getText(self, "Load Nodes", "Enter nodes string:")
        if ok and nodes_string:
            self.central_widget.load_nodes(nodes_string)

    def save_nodes_to_file(self):
        nodes_string = ""
        nodes = self.central_widget.get_nodes()
        if (len(nodes) > 0):
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
        if (len(nodes) > 2 and self.start_node and self.end_node):
            time_intervals, positions, velocities, accelerations, headings, nodes_map = self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width)
            for i in range(0, len(time_intervals), 5): # Every 25ms save data
                nodes_data.append([velocities[i], headings[i]])

        nodes_actions = [
                [
                    int(node.spinIntake),
                    int(node.clampGoal),
                    int(node.isReverseNode),
                    node.turn,
                    node.wait_time
                ]
                for node in nodes
            ]
        print(len(nodes_map), len(nodes_actions))
        for i in range(0, len(nodes_map)):
            nodes_data.insert(int(nodes_map[i]/5)+i, nodes_actions[i])
        print(nodes_data)
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
            self.central_widget.load_nodes(nodes_string)

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
                if (len(self.central_widget.get_nodes()) > 0 or os.stat(full_path).st_size == 0):
                    self.auto_save()
                else:
                    self.load_nodes_from_file(False)

    def clear_nodes(self):
        self.central_widget.clear_nodes()
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
                node.isReverseNode = bool(node_data[6])
                node.turn = node_data[7]
                node.wait_time = node_data[8]
                self.nodes.append(node)
                node.show()

        self.central_widget.update_path()

    def convert_nodes(self, as_list=False):
        nodes = self.central_widget.get_nodes()
        print(nodes)
        nodes_data = [
            [
                node.x,
                node.y,
                int(node.isStartNode),
                int(node.isEndNode),
                int(node.spinIntake),
                int(node.clampGoal),
                int(node.isReverseNode),
                node.turn,
                node.wait_time
            ]
            for node in nodes
        ]
        if (as_list):
            return nodes_data
        nodes_string = json.dumps(nodes_data, separators=(',', ':'))
        return nodes_string
    
    def auto_save(self):
        print("AUTO SAVING")
        print(self.current_working_file, " ", self.central_widget.start_node, " ", self.central_widget.end_node)
        if (self.current_working_file != None and self.central_widget.start_node and self.central_widget.end_node and not self.clearing_nodes):
            if (len(self.nodes) > 0):
                print("SAVING FR")
                self.save_nodes_to_file()
            else:
                print("LOADING AWOERO")
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
        # print(stringified)
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
                    # print("INSERTED: ", content[i])
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
        # print(content, " ", self.routes_header_path)
        # f = open(self.routes_header_path, "a")
        # f.writelines(content)
        # f.close()

        # f = open(self.routes_header_path, "r")
        # print(f.read())
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
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width)

        create_mpl_plot(self.central_widget.all_time_intervals, self.central_widget.all_positions, 12, 8, "Position Profile", "Time (s)", "Position (ft)")

    def velocity_graph(self):
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width)

        create_mpl_plot(self.central_widget.all_time_intervals, self.central_widget.all_velocities, 12, 8, "Velocity Profile", "Time (s)", "Velocity (ft/s)")

    def acceleration_graph(self):
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width)

        create_mpl_plot(self.central_widget.all_time_intervals, self.central_widget.all_accelerations, 12, 8, "Acceleration Profile", "Time (s)", "Acceleration (ft/s²)")

    def heading_graph(self):
        self.central_widget.calculateScurveStuff(self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width)

        create_mpl_plot(self.central_widget.all_time_intervals, self.central_widget.all_headings, 12, 8, "Heading Profile", "Time (s)", "Heading (degrees)")
