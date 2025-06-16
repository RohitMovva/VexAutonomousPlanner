from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QHBoxLayout, QPushButton, QScrollArea, QSizePolicy, QGroupBox, QFormLayout, QDoubleSpinBox
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import QSize, Qt
import numpy as np

import utilities

class PathViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.gui_manager = parent  # Assuming parent is the gui manager

        self.layout = QVBoxLayout(self)

        # Scroll area for node list
        self.scroll_area = QScrollArea(self)
        self.scroll_area.setWidgetResizable(True)
        self.scroll_content = QWidget()
        self.scroll_layout = QVBoxLayout(self.scroll_content)

        self.scroll_content.setLayout(self.scroll_layout)
        self.scroll_area.setWidget(self.scroll_content)
        self.layout.addWidget(self.scroll_area)

        # --- Spline Options View ---
        self.spline_options_widget = QWidget()
        self.spline_options_layout = QVBoxLayout(self.spline_options_widget)
        self.spline_options_layout.setContentsMargins(0, 0, 0, 0)

        self.selected_node = None


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

        self.tangent_form.addRow("dx:", self.tangent_dx)
        self.tangent_form.addRow("dy:", self.tangent_dy)
        self.tangent_form.addRow("Incoming Magnitude:", self.magnitude_incoming)
        self.tangent_form.addRow("Outgoing Magnitude:", self.magnitude_outgoing)
        self.tangent_group.setLayout(self.tangent_form)
        self.spline_options_layout.addWidget(self.tangent_group)

        # Connect tangent spinboxes to update handler
        self.tangent_dx.valueChanged.connect(self.on_tangent_changed)
        self.tangent_dy.valueChanged.connect(self.on_tangent_changed)

        # Pin spline options to bottom
        self.layout.addWidget(self.spline_options_widget, alignment=Qt.AlignmentFlag.AlignBottom)

        self.update_view()

    def update_view(self):
        # Clear existing widgets
        while self.scroll_layout.count():
            item = self.scroll_layout.takeAt(0)
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
            self.scroll_layout.addWidget(node_widget)

        self.scroll_layout.addStretch()

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
        self.tangent_dx.setValue(self.gui_manager.get_tangent_at_node(node)[0])
        self.tangent_dy.setValue(self.gui_manager.get_tangent_at_node(node)[1])

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