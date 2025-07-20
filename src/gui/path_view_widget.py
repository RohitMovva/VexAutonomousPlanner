import logging

import numpy as np
from PyQt6.QtCore import QSize, Qt
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

import utilities
from gui import node

logger = logging.getLogger(__name__)


class PathViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.gui_manager = parent

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        path_view_content = QWidget()
        main_layout = QVBoxLayout(path_view_content)
        main_layout.setSpacing(10)

        # Scroll area for node list
        self.node_area = QScrollArea(self)
        self.node_area.setWidgetResizable(True)
        self.node_area.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )
        # self.node_area.setHorizontalScroll
        self.node_content = QWidget()
        self.node_layout = QVBoxLayout(self.node_content)

        self.node_content.setLayout(self.node_layout)
        self.node_area.setWidget(self.node_content)
        main_layout.addWidget(self.node_area)

        self.selected_node: node.Node = None

        # Position Section
        self.position_group = QGroupBox("Position")
        self.position_form = QFormLayout()

        self.position_x = QDoubleSpinBox()
        self.position_x.setRange(-10000.0, 10000.0)
        self.position_x.setDecimals(3)

        self.position_y = QDoubleSpinBox()
        self.position_y.setRange(-10000.0, 10000.0)
        self.position_y.setDecimals(3)

        self.position_form.addRow("X:", self.position_x)
        self.position_form.addRow("Y:", self.position_y)

        self.position_group.setLayout(self.position_form)
        main_layout.addWidget(self.position_group)

        self.position_y.valueChanged.connect(self.on_position_changed)
        self.position_x.valueChanged.connect(self.on_position_changed)

        # Ending Tangent Section
        self.tangent_group = QGroupBox("Tangent")
        self.tangent_form = QFormLayout()

        self.tangent_dx = QDoubleSpinBox()
        self.tangent_dx.setRange(-10000.0, 10000.0)
        self.tangent_dx.setDecimals(3)

        self.tangent_dy = QDoubleSpinBox()
        self.tangent_dy.setRange(-10000.0, 10000.0)
        self.tangent_dy.setDecimals(3)

        self.magnitude_incoming = QDoubleSpinBox()
        self.magnitude_incoming.setRange(0.0, 10000.0)
        self.magnitude_incoming.setDecimals(3)

        self.magnitude_outgoing = QDoubleSpinBox()
        self.magnitude_outgoing.setRange(0.0, 10000.0)
        self.magnitude_outgoing.setDecimals(3)

        self.tangent_form.addRow("dX:", self.tangent_dx)
        self.tangent_form.addRow("dY:", self.tangent_dy)
        self.tangent_form.addRow("Incoming Magnitude:", self.magnitude_incoming)
        self.tangent_form.addRow("Outgoing Magnitude:", self.magnitude_outgoing)

        self.reset_button = QPushButton("Reset Tangent")
        self.reset_button.setToolTip("Reset tangent values to default")
        self.reset_button.clicked.connect(
            self.reset_tangent
        )  # Connect to reset_tangent function
        self.tangent_form.addRow(self.reset_button)

        self.tangent_group.setLayout(self.tangent_form)
        main_layout.addWidget(self.tangent_group)

        # Connect tangent spinboxes to update handler
        self.tangent_dx.valueChanged.connect(self.on_tangent_changed)
        self.tangent_dy.valueChanged.connect(self.on_tangent_changed)

        self.magnitude_incoming.valueChanged.connect(self.on_tangent_changed)
        self.magnitude_outgoing.valueChanged.connect(self.on_tangent_changed)

        self.changing_node = False

        self.lock_pixmap = QIcon(utilities.file_management.resource_path("../assets/lock.png"))
        self.unlock_pixmap = QIcon(utilities.file_management.resource_path("../assets/unlock.png"))
        self.eye_pixmap = QIcon(utilities.file_management.resource_path("../assets/eye.png"))
        self.noeye_pixmap = QIcon(utilities.file_management.resource_path("../assets/noeye.png"))
        self.trash_pixmap = QIcon(utilities.file_management.resource_path("../assets/trash.png"))

        scroll_area.setWidget(path_view_content)

        # Pin spline options to bottom
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(scroll_area)
        self.setLayout(layout)
        self.update_view()

    def update_view(self):
        # Clear existing widgets
        while self.node_layout.count():
            item = self.node_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        # Get nodes from gui_manager
        nodes = self.gui_manager.get_nodes()

        for node in nodes:
            node_widget = QWidget()
            node_layout = QHBoxLayout(node_widget)
            node_layout.setContentsMargins(0, 0, 0, 0)

            node_label = QLabel(node.get_name())
            node_label.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred
            )
            node_layout.addWidget(node_label)

            node_layout.addStretch()

            # Lock button
            lock_button = QPushButton()
            lock_button.setIcon(self.lock_pixmap if node.is_locked() else self.unlock_pixmap)
            lock_button.setIconSize(QSize(24, 24))
            lock_button.setFixedSize(24, 24)
            lock_button.setFlat(True)
            lock_button.setToolTip("Lock/Unlock Node")
            lock_button.clicked.connect(
                lambda checked, n=node, b=lock_button: self.on_lock_clicked(n, b)
            )
            node_layout.addWidget(lock_button)

            # Eye (visibility) button
            eye_button = QPushButton()
            eye_button.setIcon(self.eye_pixmap if node.is_visible() else self.noeye_pixmap)
            eye_button.setIconSize(QSize(24, 24))
            eye_button.setFixedSize(24, 24)
            eye_button.setFlat(True)
            eye_button.setToolTip("Show/Hide Node")
            eye_button.clicked.connect(
                lambda checked, n=node, b=eye_button: self.on_eye_clicked(n, b)
            )
            node_layout.addWidget(eye_button)

            # Trash (delete) button
            trash_button = QPushButton()
            trash_button.setIcon(self.trash_pixmap)
            trash_button.setIconSize(QSize(24, 24))
            trash_button.setFixedSize(24, 24)
            trash_button.setFlat(True)
            trash_button.setToolTip("Delete Node")
            trash_button.clicked.connect(
                lambda checked, n=node: self.on_trash_clicked(n)
            )
            node_layout.addWidget(trash_button)

            node_widget.setLayout(node_layout)
            if node is self.selected_node:
                node_widget.setStyleSheet("background-color: #000000;")
            self.node_layout.addWidget(node_widget)

        self.node_layout.addStretch()

    def reset_tangent(self):
        if not self.selected_node:
            return
        self.changing_node = True
        self.selected_node.set_tangent(None)
        self.selected_node.set_outgoing_magnitude(None)
        self.selected_node.set_incoming_magnitude(None)

        self.gui_manager.update_path()

        self.update_selected_node()
        self.changing_node = False

    # Placeholder callback methods
    def on_lock_clicked(self, node, button):
        node.set_locked(not node.is_locked())
        # Update icon
        if node.is_locked():
            icon_path = utilities.file_management.resource_path("../assets/lock.png")
        else:
            icon_path = utilities.file_management.resource_path("../assets/unlock.png")
        button.setIcon(QIcon(icon_path))

    def set_selected_node(self, node):
        self.changing_node = True
        self.selected_node = node

        self.update_selected_node()

        self.changing_node = False
        self.update_view()

    def update_selected_node(self):
        if (
            self.gui_manager.get_incoming_magnitude_at_node(self.selected_node) != 0
            or self.gui_manager.get_outgoing_magnitude_at_node(self.selected_node) != 0
        ):
            tangent = self.gui_manager.get_tangent_at_node(self.selected_node)
            if self.gui_manager.get_outgoing_magnitude_at_node(self.selected_node) == 0:
                self.tangent_dx.setValue(
                    tangent[0]
                    / self.gui_manager.get_incoming_magnitude_at_node(
                        self.selected_node
                    )
                )
                self.tangent_dy.setValue(
                    tangent[1]
                    / self.gui_manager.get_incoming_magnitude_at_node(
                        self.selected_node
                    )
                )
            else:
                self.tangent_dx.setValue(
                    tangent[0]
                    / self.gui_manager.get_outgoing_magnitude_at_node(
                        self.selected_node
                    )
                )
                self.tangent_dy.setValue(
                    tangent[1]
                    / self.gui_manager.get_outgoing_magnitude_at_node(
                        self.selected_node
                    )
                )

            self.magnitude_incoming.setValue(
                self.gui_manager.get_incoming_magnitude_at_node(self.selected_node)
            )
            self.magnitude_outgoing.setValue(
                self.gui_manager.get_outgoing_magnitude_at_node(self.selected_node)
            )

        pos = (self.selected_node.get_abs_x(), self.selected_node.get_abs_y())
        self.position_x.setValue(pos[0])
        self.position_y.setValue(pos[1])

        self.update_view()

    def on_eye_clicked(self, node, button):
        node.set_visible(not node.is_visible())
        # Update icon
        if node.is_visible():
            icon_path = utilities.file_management.resource_path("../assets/eye.png")
        else:
            icon_path = utilities.file_management.resource_path("../assets/noeye.png")
        button.setIcon(QIcon(icon_path))

    def on_trash_clicked(self, node):
        node.delete_node()

    def on_tangent_changed(self):
        """Handle changes to the tangent form values"""
        if not self.changing_node and (
            self.magnitude_outgoing.value() > 0 or self.magnitude_incoming.value() > 0
        ):
            tangent = np.array([self.tangent_dx.value(), self.tangent_dy.value()])
            self.selected_node.set_tangent(tangent)
            self.selected_node.set_incoming_magnitude(self.magnitude_incoming.value())
            self.selected_node.set_outgoing_magnitude(self.magnitude_outgoing.value())
            self.gui_manager.update_path()

    def on_position_changed(self):
        if self.selected_node is not None:
            position = (self.position_x.value(), self.position_y.value())
            self.selected_node.set_position(position[0], position[1])
