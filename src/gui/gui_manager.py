import json
import logging
import os
from pathlib import Path

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QAction, QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QInputDialog,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
)

import utilities.file_management
from gui import action_point, dock_widget, node, path
from utilities import config_manager

logger = logging.getLogger(__name__)


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

        self.setWindowTitle("Path Planner")  # Totally not inspired by Pronounce that
        self.setWindowIcon(
            QIcon(utilities.file_management.resource_path("../assets/flip_logo.ico"))
        )
        # Set default window size
        self.resize(700, 400)  # Width: 1280px, Height: 720px
        self.layout = QVBoxLayout()
        self.start_node = None
        self.end_node = None

        self.current_working_file = None
        self.routes_header_path = None
        self.routes_folder_path = None

        self.clearing_nodes = False

        # Get parent of current working directory
        config_file_path = os.path.join(os.getcwd(), "..", "config.yaml")
        self.config_manager: config_manager.ConfigManager = config_manager.ConfigManager(config_file_path)

        autonomous_path = (
            self.config_manager.get_value("files", "header_folder") + "/routes.h"
        )
        routes_path = self.config_manager.get_value("files", "routes_folder")

        self.routes_header_path = autonomous_path
        self.routes_folder_path = routes_path

        self.max_velocity = self.config_manager.get_value("motion", "max_velocity")
        self.max_acceleration = self.config_manager.get_value(
            "motion", "max_acceleration"
        )
        self.max_jerk = self.config_manager.get_value("motion", "max_jerk")

        self.track_width = self.config_manager.get_value("robot", "track_width") / 12

        # Image and path widget
        self.central_widget = path.PathWidget(self.config_manager, self)

        self.setCentralWidget(self.central_widget)
        self.central_widget.show()

        # Settings Dock Widget
        self.settings_dock_widget = dock_widget.MultiPageDockWidget(
            self.config_manager, self
        )

        # Add widgets to layout
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        self.addDockWidget(
            Qt.DockWidgetArea.RightDockWidgetArea, self.settings_dock_widget
        )

        # Optional: you can also set it to a specific height on the screen
        # This will position the dock at its preferred size
        self.resizeDocks([self.settings_dock_widget], [300], Qt.Orientation.Horizontal)
        self.resizeDocks([self.settings_dock_widget], [400], Qt.Orientation.Vertical)

        self.update()
        self.create_menu_bar()

    def update_coords(self, p):
        # p = self.mapToScene(event.position().toPoint())
        x = round(((p.x() / (2000)) - 0.5) * 12.3266567842 * 12, 3)
        y = round(((p.y() / (2000)) - 0.5) * 12.3266567842 * 12, 3)

        self.settings_dock_widget.set_current_coordinates(x, y)

    def create_menu_bar(self):
        menu_bar = self.menuBar()

        file_menu = menu_bar.addMenu("File")
        tools_menu = menu_bar.addMenu("Tools")

        save_nodes_file_action = QAction("Save Nodes to File", self)
        save_nodes_file_action.triggered.connect(self.save_nodes_to_file)
        file_menu.addAction(save_nodes_file_action)

        load_nodes_file_action = QAction("Load Nodes from File", self)
        load_nodes_file_action.triggered.connect(self.load_nodes_from_file)
        file_menu.addAction(load_nodes_file_action)

        save_nodes_string_action = QAction("Convert to String", self)
        save_nodes_string_action.triggered.connect(self.save_nodes_to_string)
        file_menu.addAction(save_nodes_string_action)

        load_nodes_string_action = QAction("Load Nodes from String", self)
        load_nodes_string_action.triggered.connect(self.load_nodes_from_string)
        file_menu.addAction(load_nodes_string_action)

        set_current_file_action = QAction("Set Current Working File", self)
        set_current_file_action.triggered.connect(self.set_working_file)
        file_menu.addAction(set_current_file_action)

        clear_nodes_action = QAction("Clear All Nodes", self)
        clear_nodes_action.triggered.connect(self.clear_nodes)
        tools_menu.addAction(clear_nodes_action)

        show_position_graph = QAction("Show Position Graph", self)
        show_position_graph.triggered.connect(self.position_graph)
        tools_menu.addAction(show_position_graph)

        show_velocity_graph = QAction("Show Velocity Graph", self)
        show_velocity_graph.triggered.connect(self.velocity_graph)
        tools_menu.addAction(show_velocity_graph)

        show_acceleration_graph = QAction("Show Acceleration Graph", self)
        show_acceleration_graph.triggered.connect(self.acceleration_graph)
        tools_menu.addAction(show_acceleration_graph)

        show_heading_graph = QAction("Show Heading Graph", self)
        show_heading_graph.triggered.connect(self.heading_graph)
        tools_menu.addAction(show_heading_graph)

        show_angular_velocity_graph = QAction("Show Angular Velocity Graph", self)
        show_angular_velocity_graph.triggered.connect(self.angular_velocity_graph)
        tools_menu.addAction(show_angular_velocity_graph)

        show_coords_graph = QAction("Show Coords Graph", self)
        show_coords_graph.triggered.connect(self.coords_graph)
        tools_menu.addAction(show_coords_graph)

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
        logger.info(f"Nodes saved as string: {nodes_string}")
        dialog = SaveNodesDialog(nodes_string, self)
        dialog.exec()

    def load_nodes_from_string(self):
        nodes_string, ok = QInputDialog.getText(
            self, "Load Nodes", "Enter nodes string:"
        )
        if ok and nodes_string:
            self.central_widget.load_nodes(nodes_string)

    def mirror_nodes(self):
        self.central_widget.mirror_nodes()

    def save_nodes_to_file(self):
        nodes_string = ""
        nodes = self.central_widget.get_nodes()
        if len(nodes) > 0:
            nodes_string = self.convert_nodes()
        full_path = None

        if self.current_working_file is None:
            file_name, ok = QInputDialog.getText(
                self, "Save Route to File", "Enter file name (without extension):"
            )
            if not ok or not file_name:
                return

            file_name = file_name.strip() + ".txt"
            folder = QFileDialog.getExistingDirectory(
                self,
                "Select Directory to Save File",
                str(Path(os.getcwd()).parent.absolute()),
            )
            if not folder:
                return

            full_path = f"{folder}/{file_name}"
        else:
            full_path = (
                self.routes_folder_path + "/" + self.current_working_file + ".txt"
            )
        nodes_data = []
        nodes_map = []
        actions_map = []
        if (
            len(self.central_widget.get_nodes()) > 2
            and self.central_widget.start_node
            and self.central_widget.end_node
        ):
            (
                time_intervals,
                positions,
                velocities,
                accelerations,
                headings,
                angular_velocities,
                nodes_map,
                actions_map,
                coords,
            ) = self.central_widget.generate_motion_profile_lists(
                self.max_velocity,
                self.max_acceleration,
                self.max_jerk,
                self.track_width,
            )
            for i in range(0, len(time_intervals), 1):  # Every 10ms save data
                nodes_data.append(
                    [
                        0,
                        time_intervals[i],
                        coords[i][0] * 12,
                        coords[i][1] * -12,
                        headings[i],
                        velocities[i] * 12,
                        angular_velocities[i],
                    ]
                )
        logger.info(f"nodes map: {nodes_map} traj length {len(nodes_data)}")
        nodes_actions = [
            [1] + cur_node.get_action_values()
            for cur_node in nodes
        ]

        actions_data = [
            [1] + cur_action.get_action_values()
            for cur_action in self.central_widget.action_points
        ]

        logger.info(
            f"nodes map length {len(nodes_map)}, node actions length {len(nodes_actions)}"
        )
        for i in range(0, len(nodes_map)):
            nodes_data.insert(int(nodes_map[i] / 1) + i, nodes_actions[i])
        for i in range(0, len(actions_map)):
            nodes_data.insert(int(actions_map[i] / 1) + i, actions_data[i])

        self.fill_template(nodes_data)
        with open(full_path, "w") as file:
            file.write(nodes_string)
        logger.info(f"Route saved to {full_path}")

    def load_nodes_from_file(self, dialogue=True):
        file_name = None
        if dialogue:
            file_name, _ = QFileDialog.getOpenFileName(
                self, "Load Route from File", "", "Text Files (*.txt);;All Files (*)"
            )
        else:
            file_name = (
                self.routes_folder_path + "/" + self.current_working_file + ".txt"
            )
        if file_name is not None:
            with open(file_name, "r") as file:
                nodes_string = file.read()
            logger.info(f"Route loaded from {file_name}")
            self.central_widget.load_nodes(nodes_string)

    def load_route_file(self, file_path):
        """Load a route file"""
        
        if (not file_path.endswith(".txt")
                or not os.path.exists(os.path.join(self.routes_folder_path, file_path))):
            logger.error(f"Invalid file path: {file_path}")
            return
        self.current_working_file = file_path[:-4]  # cut off the .txt
        file_path = os.path.join(self.routes_folder_path, file_path)
        with open(file_path, "r") as file:
            nodes_string = file.read()

        logger.info(f"Route loaded from {file_path}")
        self.central_widget.load_nodes(nodes_string)

    def set_working_file(self):
        file_name, ok = QInputDialog.getText(
            self, "File to save route to", "Enter file name (without extension):"
        )
        if ok and file_name:
            self.current_working_file = file_name
            folder = self.routes_folder_path
            if folder is None:
                folder = QFileDialog.getExistingDirectory(
                    self,
                    "Select Directory to Save File",
                    str(Path(os.getcwd()).parent.absolute()),
                )

            if folder:
                self.routes_folder_path = folder
                full_path = f"{folder}/{self.current_working_file}.txt"
                if not os.path.exists(full_path):
                    with open(
                        full_path, "w+"
                    ):  # Creates file if it isn't already created
                        pass
                logger.info(f"Set current working file at {full_path}")
                if (
                    len(self.central_widget.get_nodes()) > 0
                    or os.stat(full_path).st_size == 0
                ):
                    self.auto_save()
                else:
                    self.load_nodes_from_file(False)

    def clear_nodes(self):
        logger.info("Clearing all nodes...")
        self.central_widget.clear_nodes()

    def convert_nodes(self, as_list=False):
        nodes: list[node.Node] = self.central_widget.get_nodes()
        logger.info("Converting nodes...")
        logger.debug(f"Nodes: {nodes}")
        nodes_data = [
            [
                cur_node.get_abs_x(),
                cur_node.get_abs_y(),
                int(cur_node.is_start_node),
                int(cur_node.is_end_node),
                int(cur_node.is_reverse_node),
                int(cur_node.stop),
                cur_node.turn,
                cur_node.wait_time,
            ] + cur_node.get_action_values()
            for cur_node in nodes
        ]
        action_data = [
            [
                cur_action.get_abs_x(),
                cur_action.get_abs_y(),
                cur_action.t,
                int(cur_action.stop),
                cur_action.wait_time,
            ] + cur_action.get_action_values()
            for cur_action in self.central_widget.action_points
        ]
        if as_list:
            return [nodes_data, action_data]
        nodes_string = json.dumps([nodes_data, action_data], separators=(",", ":"))
        return nodes_string

    def auto_save(self):
        if (
            self.current_working_file is not None
            and self.central_widget.start_node
            and self.central_widget.end_node
            and not self.clearing_nodes
        ):
            logger.info("Auto Saving...")
            if len(self.central_widget.get_nodes()) > 0:
                self.save_nodes_to_file()
            else:
                self.load_nodes_from_file(False)

    def fill_template(self, nodes_data):
        # Create the string to insert
        stringified = []
        for i in range(0, len(nodes_data)):
            if len(nodes_data[i]) > 2:
                stringified.append("{")
                for j in range(0, len(nodes_data[i])):
                    stringified[-1] += f"{nodes_data[i][j]}"
                    if j < len(nodes_data[i]) - 1:
                        stringified[-1] += ", "
                stringified[-1] += "}"
            else:
                stringified.append(f"{{{nodes_data[i][0]}, {nodes_data[i][1]}}}")
        insertion = f"std::vector<std::vector<double>> {self.current_working_file} = {{{', '.join(stringified)}}};\n"

        try:
            # Read the content of routes.h
            with open(self.routes_header_path, "r") as routes_file:
                content = routes_file.readlines()

            # Find the line with the specified route name
            inserted = False
            for i, line in enumerate(content):
                if line.strip().startswith(
                    f"std::vector<std::vector<double>> {self.current_working_file} ="
                ):
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
                "#endif\n",
            ]

        if (
            not os.path.exists(self.routes_header_path)
            or os.stat(self.routes_header_path).st_size == 0
        ):
            # If the file is empty, write the content
            content = [
                "#ifndef ROUTES_H\n",
                "#define ROUTES_H\n",
                "#include <vector>\n",
                "\n",
                insertion,
                "\n",
                "#endif\n",
            ]

        logger.info(f"Saving motion profile to {self.routes_header_path}")
        with open(self.routes_header_path, "w") as routes_file:
            routes_file.writelines(content)

    def switch_field(self, fieldType: str):
        if fieldType == "High Stakes Match":
            self.central_widget.update_image_path(
                utilities.file_management.resource_path(
                    "../assets/V5RC-PushBack-Match-2000x2000.png"
                )
            )
        else:
            self.central_widget.update_image_path(
                utilities.file_management.resource_path(
                    "../assets/V5RC-PushBack-Skills-2000x2000.png"
                )
            )

    def set_velocity(self, new_velocity):
        self.max_velocity = new_velocity
        utilities.file_management.set_config_value("max_velocity", self.max_velocity)

    def set_acceleration(self, new_acceleration):
        self.max_acceleration = new_acceleration
        utilities.file_management.set_config_value(
            "max_acceleration", self.max_acceleration
        )

    def set_jerk(self, new_jerk):
        self.max_jerk = new_jerk
        utilities.file_management.set_config_value("max_jerk", self.max_jerk)

    def set_track_width(self, new_track_width):
        self.track_width = new_track_width
        utilities.file_management.set_config_value("track_width", self.track_width)

    def toggle_robot_visualization(self, state):
        self.central_widget.toggle_visualization(state)

    def position_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.positions,
            12,
            8,
            "Position Profile",
            "Time (s)",
            "Position (ft)",
        )

    def velocity_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.velocities,
            12,
            8,
            "Velocity Profile",
            "Time (s)",
            "Velocity (ft/s)",
        )

    def acceleration_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.accelerations,
            12,
            8,
            "Acceleration Profile",
            "Time (s)",
            "Acceleration (ft/s²)",
        )

    def heading_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.headings,
            12,
            8,
            "Heading Profile",
            "Time (s)",
            "Heading (degrees)",
        )

    def angular_velocity_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.angular_velocities,
            12,
            8,
            "Angular Velocity Profile",
            "Time (s)",
            "Angular Velocity (radians/s)",
        )

    def coords_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width
        )

        x, y = zip(*self.central_widget.coords)

        utilities.file_management.create_mpl_plot(
            x,
            y,
            8,
            8,
            "Coordinate Profile",
            "X",
            "Y",
            ((-6), (6)),
            ((-6), (6)),
            1,
        )
