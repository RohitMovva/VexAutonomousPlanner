import logging

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)

logger = logging.getLogger(__name__)


class SettingsWidget(QWidget):
    def __init__(self, config_manager, parent=None):
        super().__init__()
        self.parent = parent
        self.config_manager = config_manager

        # Create a scroll area to contain all the settings
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        # Create the settings widget that will be inside the scroll area
        settings_content = QWidget()
        main_layout = QVBoxLayout(settings_content)
        main_layout.setSpacing(10)

        # Field Settings Group
        field_group = QGroupBox("Field Settings")
        field_layout = QFormLayout()

        # Add Field Type drop-down menu
        self.field_type_combo = QComboBox()
        self.field_type_combo.addItems(["Push Back Match", "Push Back Skills"])
        current_field_type = self.config_manager.get_value(
            "field", "type", "Push Back Match"
        )
        index = self.field_type_combo.findText(current_field_type)
        self.field_type_combo.setCurrentIndex(index if index >= 0 else 0)
        field_layout.addRow("Field Type:", self.field_type_combo)
        self.on_field_type_changed()
        self.field_type_combo.currentIndexChanged.connect(self.on_field_type_changed)

        # Mirror button (moved to field settings)
        self.mirror_button = QPushButton("Mirror")
        self.mirror_button.clicked.connect(self.on_mirror_clicked)
        field_layout.addRow(self.mirror_button)

        field_group.setLayout(field_layout)
        main_layout.addWidget(field_group)

        # Motion Constraints Group
        motion_group = QGroupBox("Motion Constraints")
        motion_layout = QFormLayout()

        # Add max velocity, acceleration, and jerk inputs
        self.velocity_input = QDoubleSpinBox()
        self.velocity_input.setRange(0.0, 100.0)
        self.velocity_input.setValue(
            self.config_manager.get_value("motion", "max_velocity", 4.0)
        )
        motion_layout.addRow("Max Velocity (ft/s):", self.velocity_input)
        self.velocity_input.valueChanged.connect(self.on_velocity_changed)

        self.acceleration_input = QDoubleSpinBox()
        self.acceleration_input.setRange(0, 100)
        self.acceleration_input.setValue(
            self.config_manager.get_value("motion", "max_acceleration", 8.0)
        )
        motion_layout.addRow("Max Acceleration (ft/s²):", self.acceleration_input)
        self.acceleration_input.valueChanged.connect(self.on_acceleration_changed)

        self.jerk_input = QDoubleSpinBox()
        self.jerk_input.setRange(0, 100)
        self.jerk_input.setValue(
            self.config_manager.get_value("motion", "max_jerk", 16.0)
        )
        motion_layout.addRow("Max Jerk (ft/s³):", self.jerk_input)
        self.jerk_input.valueChanged.connect(self.on_jerk_changed)

        motion_group.setLayout(motion_layout)
        main_layout.addWidget(motion_group)

        # Robot Properties Group
        robot_group = QGroupBox("Robot Properties")
        robot_layout = QFormLayout()

        # Robot dimensions
        self.robot_width_input = QDoubleSpinBox()
        self.robot_width_input.setRange(0.1, 50.0)
        self.robot_width_input.setValue(
            self.config_manager.get_value("robot", "width", 18.0)
        )
        self.robot_width_input.setSuffix(" in")
        robot_layout.addRow("Robot Width:", self.robot_width_input)
        self.robot_width_input.valueChanged.connect(self.on_robot_width_changed)

        self.robot_length_input = QDoubleSpinBox()
        self.robot_length_input.setRange(0.1, 50.0)
        self.robot_length_input.setValue(
            self.config_manager.get_value("robot", "length", 18.0)
        )
        self.robot_length_input.setSuffix(" in")
        robot_layout.addRow("Robot Length:", self.robot_length_input)
        self.robot_length_input.valueChanged.connect(self.on_robot_length_changed)

        self.track_width_input = QDoubleSpinBox()
        self.track_width_input.setRange(0.1, 40.0)
        self.track_width_input.setValue(
            self.config_manager.get_value("robot", "track_width", 12.5)
        )
        self.track_width_input.setSuffix(" in")
        robot_layout.addRow("Track Width:", self.track_width_input)
        self.track_width_input.valueChanged.connect(self.on_track_width_changed)

        # Toggle for the robot visualization feature
        self.robot_visualization_toggle = QComboBox()
        self.robot_visualization_toggle.addItems(["On", "Off"])
        visualization_enabled = self.config_manager.get_value(
            "robot", "visualization", False
        )
        self.parent.toggle_robot_visualization(visualization_enabled)
        self.robot_visualization_toggle.setCurrentIndex(
            0 if visualization_enabled else 1
        )
        robot_layout.addRow("Robot Visualization:", self.robot_visualization_toggle)
        self.robot_visualization_toggle.currentIndexChanged.connect(
            self.on_robot_visualization_change
        )

        robot_group.setLayout(robot_layout)
        main_layout.addWidget(robot_group)

        # File Settings Group
        file_group = QGroupBox("File Settings")
        file_layout = QFormLayout()

        # Header file folder selection
        self.header_file_path = QLineEdit()
        self.header_file_path.setReadOnly(True)
        header_folder = self.config_manager.get_value("files", "header_folder", "")
        self.header_file_path.setText(header_folder)
        self.header_file_path.setPlaceholderText("Select header file folder...")

        browse_button = QPushButton("Browse...")
        browse_button.clicked.connect(self.browse_header_folder)

        header_path_layout = QHBoxLayout()
        header_path_layout.addWidget(self.header_file_path)
        header_path_layout.addWidget(browse_button)

        file_layout.addRow("Header File Folder:", header_path_layout)

        # Routes folder selection
        self.routes_file_path = QLineEdit()
        self.routes_file_path.setReadOnly(True)
        routes_folder = self.config_manager.get_value("files", "routes_folder", "")
        self.routes_file_path.setText(routes_folder)
        self.routes_file_path.setPlaceholderText("Select routes folder...")

        routes_browse_button = QPushButton("Browse...")
        routes_browse_button.clicked.connect(self.browse_routes_folder)

        routes_path_layout = QHBoxLayout()
        routes_path_layout.addWidget(self.routes_file_path)
        routes_path_layout.addWidget(routes_browse_button)

        file_layout.addRow("Routes Folder:", routes_path_layout)

        file_group.setLayout(file_layout)
        main_layout.addWidget(file_group)

        # Spacer to push coordinate display to the bottom
        main_layout.addItem(
            QSpacerItem(
                20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
            )
        )

        # Coordinates Display Group
        coord_group = QGroupBox("Current Position")
        coord_layout = QFormLayout()

        self.current_x_label = QLabel("0")
        coord_layout.addRow("Current X:", self.current_x_label)
        self.current_y_label = QLabel("0")
        coord_layout.addRow("Current Y:", self.current_y_label)

        coord_group.setLayout(coord_layout)
        main_layout.addWidget(coord_group)

        # Add the settings widget to the scroll area
        scroll_area.setWidget(settings_content)

        # Set up the layout for this widget
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(scroll_area)
        self.setLayout(layout)

    def set_current_coordinates(self, x, y):
        self.current_x_label.setText(str(x))
        self.current_y_label.setText(str(y))

    def on_field_type_changed(self):
        field_type = self.field_type_combo.currentText()
        logger.info(f"Field Type changed: {field_type}")
        self.config_manager.set_value("field", "type", field_type)
        self.parent.switch_field(field_type)

    def on_velocity_changed(self):
        max_velocity = self.velocity_input.value()
        logger.info(f"Max Velocity changed: {max_velocity} ft/s")
        self.config_manager.set_value("motion", "max_velocity", max_velocity)
        self.parent.set_velocity(max_velocity)

    def on_acceleration_changed(self):
        max_acceleration = self.acceleration_input.value()
        logger.info(f"Max Acceleration changed: {max_acceleration} ft/s^2")
        self.config_manager.set_value("motion", "max_acceleration", max_acceleration)
        self.parent.set_acceleration(max_acceleration)

    def on_jerk_changed(self):
        max_jerk = self.jerk_input.value()
        logger.info(f"Max Jerk changed: {max_jerk} ft/s^3")
        self.config_manager.set_value("motion", "max_jerk", max_jerk)
        self.parent.set_jerk(max_jerk)

    def on_robot_visualization_change(self):
        visualization_state = self.robot_visualization_toggle.currentText()
        is_enabled = visualization_state == "On"
        logger.info(f"Robot Visualization changed: {visualization_state}")
        self.config_manager.set_value("robot", "visualization", is_enabled)
        self.parent.toggle_robot_visualization(is_enabled)

    def on_mirror_clicked(self):
        logger.info("Mirror button clicked")
        self.parent.mirror_nodes()

    def on_robot_width_changed(self):
        width = self.robot_width_input.value()
        logger.info(f"Robot Width changed: {width} inches")
        self.config_manager.set_value("robot", "width", width)
        # Implement in parent class if needed
        if hasattr(self.parent, "set_robot_width"):
            self.parent.set_robot_width(width)

    def on_robot_length_changed(self):
        length = self.robot_length_input.value()
        logger.info(f"Robot Length changed: {length} inches")
        self.config_manager.set_value("robot", "length", length)
        # Implement in parent class if needed
        if hasattr(self.parent, "set_robot_length"):
            self.parent.set_robot_length(length)

    def on_track_width_changed(self):
        track_width = self.track_width_input.value()
        logger.info(f"Track Width changed: {track_width} inches")
        self.config_manager.set_value("robot", "track_width", track_width)
        # Implement in parent class if needed
        if hasattr(self.parent, "set_track_width"):
            self.parent.set_track_width(track_width / 12.0)

    def browse_header_folder(self):
        folder_path = QFileDialog.getExistingDirectory(
            self,
            "Select Header File Folder",
            "",
            QFileDialog.Option.ShowDirsOnly | QFileDialog.Option.DontResolveSymlinks,
        )

        if folder_path:
            self.header_file_path.setText(folder_path)
            logger.info(f"Header file folder selected: {folder_path}")
            self.config_manager.set_value("files", "header_folder", folder_path)
            # Implement in parent class if needed
            if hasattr(self.parent, "set_header_folder"):
                self.parent.set_header_folder(folder_path)

    def browse_routes_folder(self):
        folder_path = QFileDialog.getExistingDirectory(
            self,
            "Select Routes Folder",
            "",
            QFileDialog.Option.ShowDirsOnly | QFileDialog.Option.DontResolveSymlinks,
        )

        if folder_path:
            self.routes_file_path.setText(folder_path)
            logger.info(f"Routes folder selected: {folder_path}")
            self.config_manager.set_value("files", "routes_folder", folder_path)
            # Implement in parent class if needed
            if hasattr(self.parent, "set_routes_folder"):
                self.parent.set_routes_folder(folder_path)
