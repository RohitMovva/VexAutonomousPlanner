from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QHBoxLayout, QPushButton, QScrollArea, QSizePolicy, QGroupBox, QFormLayout, QDoubleSpinBox, QSpacerItem
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import QSize, Qt
import numpy as np

from gui import node

import utilities
import logging

logger = logging.getLogger(__name__)

class PathViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.gui_manager = parent  # Assuming parent is the gui manager

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        path_view_content = QWidget()
        main_layout = QVBoxLayout(path_view_content)
        main_layout.setSpacing(10)

        # Scroll area for node list
        self.node_area = QScrollArea(self)
        self.node_area.setWidgetResizable(True)
        self.node_content = QWidget()
        self.node_layout = QVBoxLayout(self.node_content)

        self.node_content.setLayout(self.node_layout)
        self.node_area.setWidget(self.node_content)
        main_layout.addWidget(self.node_area)

        main_layout.addItem(
            QSpacerItem(
                20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
            )
        )

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
        
        # Connect position spinboxes to update handler



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
        self.tangent_group.setLayout(self.tangent_form)
        main_layout.addWidget(self.tangent_group)

        # Connect tangent spinboxes to update handler
        self.tangent_dx.valueChanged.connect(self.on_tangent_changed)
        self.tangent_dy.valueChanged.connect(self.on_tangent_changed)

        scroll_area.setWidget(path_view_content)

        # Pin spline options to bottom
        # self.layout.addWidget(self.spline_options_widget, alignment=Qt.AlignmentFlag.AlignBottom)
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
            node_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            node_layout.addWidget(node_label)

            node_layout.addStretch()

            # Lock button
            lock_button = QPushButton()
            if node.is_locked():
                lock_icon_path = utilities.file_management.resource_path("../assets/lock.png")
            else:
                lock_icon_path = utilities.file_management.resource_path("../assets/unlock.png")
            lock_button.setIcon(QIcon(lock_icon_path))
            lock_button.setIconSize(QSize(24, 24))
            lock_button.setFixedSize(24, 24)
            lock_button.setFlat(True)
            lock_button.setToolTip("Lock/Unlock Node")
            lock_button.clicked.connect(lambda checked, n=node, b=lock_button: self.on_lock_clicked(n, b))
            node_layout.addWidget(lock_button)

            # Eye (visibility) button
            eye_button = QPushButton()
            if node.is_visible():
                eye_icon_path = utilities.file_management.resource_path("../assets/eye.png")
            else:
                eye_icon_path = utilities.file_management.resource_path("../assets/noeye.png")
            eye_button.setIcon(QIcon(eye_icon_path))
            eye_button.setIconSize(QSize(24, 24))
            eye_button.setFixedSize(24, 24)
            eye_button.setFlat(True)
            eye_button.setToolTip("Show/Hide Node")
            eye_button.clicked.connect(lambda checked, n=node, b=eye_button: self.on_eye_clicked(n, b))
            node_layout.addWidget(eye_button)

            # Trash (delete) button
            trash_button = QPushButton()
            trash_icon_path = utilities.file_management.resource_path("../assets/trash.png")
            trash_button.setIcon(QIcon(trash_icon_path))
            trash_button.setIconSize(QSize(24, 24))
            trash_button.setFixedSize(24, 24)
            trash_button.setFlat(True)
            trash_button.setToolTip("Delete Node")
            trash_button.clicked.connect(lambda checked, n=node: self.on_trash_clicked(n))
            node_layout.addWidget(trash_button)

            node_widget.setLayout(node_layout)
            if node is self.selected_node:
                # highlight the node
                node_widget.setStyleSheet("background-color: #000000;")
            self.node_layout.addWidget(node_widget)

        self.node_layout.addStretch()

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
        self.selected_node = node
        tangent = self.gui_manager.get_tangent_at_node(node)
        self.tangent_dx.setValue(tangent[0])
        self.tangent_dy.setValue(tangent[1])

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
        if self.selected_node is not None:
            print(f"Tangent changed: {self.tangent_dx.value()}, {self.tangent_dy.value()}")
            tangent = np.array([self.tangent_dx.value(), self.tangent_dy.value()])
            self.gui_manager.set_tangent_at_node(self.selected_node, tangent)

    def on_position_changed(self):
        if (self.selected_node is not None):
            position = (self.position_x.value(), self.position_y.value())
            self.selected_node.set_position(position[0], position[1])
            