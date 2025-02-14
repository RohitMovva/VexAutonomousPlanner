import logging

from PyQt6.QtWidgets import (
    QComboBox,
    QDockWidget,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QPushButton,  # Added for the Mirror button
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)

logger = logging.getLogger(__name__)


class SettingsDockWidget(QDockWidget):
    def __init__(self, max_velocity, max_acceleration, max_jerk, parent=None):
        super().__init__("Settings", parent)
        self.parent = parent

        # Create the settings widget
        settings_widget = QWidget()
        main_layout = QVBoxLayout()
        settings_layout = QFormLayout()

        # Add Field Type drop-down menu
        self.field_type_combo = QComboBox()
        self.field_type_combo.addItems(["High Stakes Match", "High Stakes Skills"])
        self.field_type_combo.setCurrentIndex(0)
        settings_layout.addRow("Field Type:", self.field_type_combo)
        self.field_type_combo.currentIndexChanged.connect(self.on_field_type_changed)

        # Add max jerk, acceleration, and velocity inputs
        self.velocity_input = QDoubleSpinBox()
        self.velocity_input.setRange(0.0, 100.0)  # Adjust range as needed
        self.velocity_input.setValue(max_velocity)
        settings_layout.addRow("Max Velocity (ft/s):", self.velocity_input)
        self.velocity_input.valueChanged.connect(self.on_velocity_changed)

        self.acceleration_input = QDoubleSpinBox()
        self.acceleration_input.setRange(0, 100)  # Adjust range as needed
        self.acceleration_input.setValue(max_acceleration)
        settings_layout.addRow("Max Acceleration (ft/s²):", self.acceleration_input)
        self.acceleration_input.valueChanged.connect(self.on_acceleration_changed)

        self.jerk_input = QDoubleSpinBox()
        self.jerk_input.setRange(0, 100)  # Adjust range as needed
        self.jerk_input.setValue(max_jerk)
        settings_layout.addRow("Max Jerk (ft/s³):", self.jerk_input)
        self.jerk_input.valueChanged.connect(self.on_jerk_changed)

        # Toggle for the robot visualization feature
        self.robot_visualization_toggle = QComboBox()
        self.robot_visualization_toggle.addItems(["On", "Off"])
        self.robot_visualization_toggle.setCurrentIndex(1)
        settings_layout.addRow("Robot Visualization:", self.robot_visualization_toggle)
        self.robot_visualization_toggle.currentIndexChanged.connect(
            self.on_robot_visualization_change
        )

        # Add the Mirror button
        self.mirror_button = QPushButton("Mirror")
        self.mirror_button.clicked.connect(self.on_mirror_clicked)
        settings_layout.addRow(self.mirror_button)

        # Add the form layout to the main layout
        main_layout.addLayout(settings_layout)

        # Add a spacer to push the following widgets to the bottom
        main_layout.addItem(
            QSpacerItem(
                20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
            )
        )

        # Add labels to display current x and y coordinates
        coord_layout = QFormLayout()
        self.current_x_label = QLabel("0")
        coord_layout.addRow("Current X:", self.current_x_label)
        self.current_y_label = QLabel("0")
        coord_layout.addRow("Current Y:", self.current_y_label)

        # Add the coordinate layout to the main layout
        main_layout.addLayout(coord_layout)

        # Set the main layout for the settings widget
        settings_widget.setLayout(main_layout)
        self.setWidget(settings_widget)

    def set_current_coordinates(self, x, y):
        self.current_x_label.setText(str(x))
        self.current_y_label.setText(str(y))

    def on_field_type_changed(self):
        field_type = self.field_type_combo.currentText()
        logger.info(f"Field Type changed: {field_type}")
        self.parent.switch_field(field_type)

    def on_velocity_changed(self):
        max_velocity = self.velocity_input.value()
        logger.info(f"Max Velocity changed: {max_velocity} ft/s")
        self.parent.set_velocity(max_velocity)

    def on_acceleration_changed(self):
        max_acceleration = self.acceleration_input.value()
        logger.info(f"Max Acceleration changed: {max_acceleration} ft/s^2")
        self.parent.set_acceleration(max_acceleration)

    def on_jerk_changed(self):
        max_jerk = self.jerk_input.value()
        logger.info(f"Max Jerk changed: {max_jerk} ft/s^3")
        self.parent.set_jerk(max_jerk)

    def on_robot_visualization_change(self):
        visualization_state = self.robot_visualization_toggle.currentText()
        logger.info(f"Robot Visualization changed: {visualization_state}")
        self.parent.toggle_robot_visualization(visualization_state == "On")

    def on_mirror_clicked(self):
        logger.info("Mirror button clicked")
        self.parent.mirror_nodes()
        