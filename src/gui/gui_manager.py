import json
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

import utilities
from gui import node, path, settings


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
        self.setWindowIcon(QIcon(utilities.resource_path("../assets/flip_logo.ico")))
        self.layout = QVBoxLayout()

        # self.nodes = []
        self.start_node = None
        self.end_node = None

        self.current_working_file = None
        self.routes_header_path = None
        self.routes_folder_path = None

        self.clearing_nodes = False

        autonomous_path = utilities.get_config_value("autonomous_repository_path")
        if autonomous_path is None:
            autonomous_path = QFileDialog.getExistingDirectory(
                self,
                "Select Autonomous Program Directory",
                str(Path(os.getcwd()).parent.parent.absolute()),
            )

            utilities.set_config_value(
                "autonomous_repository_path", autonomous_path + "/routes.h"
            )
            print(f"Added autonomous repository path: {autonomous_path}/routes.h")

        routes_path = utilities.get_config_value("routes_folder_path")
        if routes_path is None:
            routes_path = QFileDialog.getExistingDirectory(
                self,
                "Select Routes Folder",
                str(Path(os.getcwd()).parent.parent.absolute()),
            )

            utilities.set_config_value("routes_folder_path", routes_path)
            print(f"Added routes folder path: {routes_path}")

        self.routes_header_path = autonomous_path
        self.routes_folder_path = routes_path

        self.max_velocity = utilities.get_config_value("max_velocity")
        self.max_acceleration = utilities.get_config_value("max_acceleration")
        self.max_jerk = utilities.get_config_value("max_jerk")

        self.track_width = utilities.get_config_value("track_width")

        # Image and path widget
        self.central_widget = path.PathWidget(self)

        self.setCentralWidget(self.central_widget)
        self.central_widget.show()

        # Settings Dock Widget
        self.settings_dock_widget = settings.SettingsDockWidget(
            self.max_velocity, self.max_acceleration, self.max_jerk, self
        )

        # Add widgets to layout
        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        # self.layout.addWidget(self.label)

        self.addDockWidget(
            Qt.DockWidgetArea.RightDockWidgetArea, self.settings_dock_widget
        )

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
                coords,
            ) = self.central_widget.generate_motion_profile_lists(
                self.max_velocity,
                self.max_acceleration,
                self.max_jerk,
                self.track_width,
            )
            for i in range(0, len(time_intervals), 1):  # Every 25ms save data
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
        print("NM")
        print(nodes_map, len(nodes_data))
        nodes_actions = [
            [
                1,
                int(cur_node.spin_intake),
                int(cur_node.clamp_goal),
                int(cur_node.doink),
                int(cur_node.is_reverse_node),
                cur_node.turn,
                cur_node.lb,
            ]
            for cur_node in nodes
        ]

        print(len(nodes_map), len(nodes_actions))
        for i in range(0, len(nodes_map)):
            nodes_data.insert(int(nodes_map[i] / 1) + i, nodes_actions[i])
        self.fill_template(nodes_data)
        with open(full_path, "w") as file:
            file.write(nodes_string)
        print(f"Route saved to {full_path}")

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
            print(f"Route loaded from {file_name}")
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
                print(f"Set current working file at {full_path}")
                if (
                    len(self.central_widget.get_nodes()) > 0
                    or os.stat(full_path).st_size == 0
                ):
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
            if len(node_data) > 4:
                print(node_data)
                new_node = node.Node(
                    node_data[0], node_data[1], self.central_widget, gui_instance=self
                )
                self.start_node = new_node if bool(node_data[2]) else self.start_node
                new_node.is_start_node = bool(node_data[2])
                self.end_node = new_node if bool(node_data[3]) else self.end_node
                new_node.is_end_node = bool(node_data[3])
                new_node.spin_intake = bool(node_data[4])
                new_node.clamp_goal = bool(node_data[5])
                new_node.doink = bool(node_data[6])
                new_node.is_reverse_node = bool(node_data[7])
                new_node.stop = bool(node_data[8])
                new_node.turn = node_data[9]
                new_node.lb = node_data[11]
                new_node.wait_time = node_data[12]
                self.nodes.append(new_node)
                new_node.show()

        self.central_widget.update_path()

    def convert_nodes(self, as_list=False):
        nodes: list[node.Node] = self.central_widget.get_nodes()
        print(nodes)
        nodes_data = [
            [
                cur_node.get_abs_x(),
                cur_node.get_abs_y(),
                int(cur_node.is_start_node),
                int(cur_node.is_end_node),
                int(cur_node.spin_intake),
                int(cur_node.clamp_goal),
                int(cur_node.doink),
                int(cur_node.is_reverse_node),
                int(cur_node.stop),
                cur_node.turn,
                cur_node.lb,
                cur_node.wait_time,
            ]
            for cur_node in nodes
        ]
        if as_list:
            return nodes_data
        nodes_string = json.dumps(nodes_data, separators=(",", ":"))
        return nodes_string

    def auto_save(self):
        if (
            self.current_working_file is not None
            and self.central_widget.start_node
            and self.central_widget.end_node
            and not self.clearing_nodes
        ):
            print("Auto Saving...")
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
        # print(stringified)
        # pairs = [f"{{{v}, {h}}}" for v, h in nodes_data]
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

        print("Saving motion profile to " + self.routes_header_path)
        with open(self.routes_header_path, "w") as routes_file:
            routes_file.writelines(content)

    def switch_field(self, fieldType: str):
        if fieldType == "High Stakes Match":
            self.central_widget.update_image_path(
                utilities.resource_path("../assets/V5RC-HighStakes-Match-2000x2000.png")
            )
        else:
            self.central_widget.update_image_path(
                utilities.resource_path(
                    "../assets/V5RC-HighStakes-Skills-2000x2000.png"
                )
            )

    def set_velocity(self, new_velocity):
        self.max_velocity = new_velocity
        utilities.set_config_value("max_velocity", self.max_velocity)

    def set_acceleration(self, new_acceleration):
        self.max_acceleration = new_acceleration
        utilities.set_config_value("max_acceleration", self.max_acceleration)

    def set_jerk(self, new_jerk):
        self.max_jerk = new_jerk
        utilities.set_config_value("max_jerk", self.max_jerk)

    def set_track_width(self, new_track_width):
        self.track_width = new_track_width
        utilities.set_config_value("track_width", self.track_width)

    def toggle_robot_visualization(self, state):
        self.central_widget.toggle_visualization(state)

    def position_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity, self.max_acceleration, self.max_jerk, self.track_width
        )

        utilities.create_mpl_plot(
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

        utilities.create_mpl_plot(
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

        utilities.create_mpl_plot(
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
        # for velo in self.central_widget.all_headings:
        #     print(velo)
        # print()

        utilities.create_mpl_plot(
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
        # for velo in self.central_widget.all_angular_velocities:
        #     print(velo)
        # print()
        # print(self.central_widget.all_angular_velocities)

        utilities.create_mpl_plot(
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

        utilities.create_mpl_plot(
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
