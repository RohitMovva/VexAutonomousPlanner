import logging

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QDockWidget,
    QHBoxLayout,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from gui.routes_widget import RouteFilesWidget
from gui.settings_widget import SettingsWidget
from gui.path_view_widget import PathViewWidget

logger = logging.getLogger(__name__)


class MultiPageDockWidget(QDockWidget):
    def __init__(self, config_manager, parent=None):
        super().__init__("Control Panel", parent)
        self.parent = parent
        self.config_manager = config_manager

        # Create main container widget
        self.main_widget = QWidget()
        self.main_layout = QVBoxLayout(self.main_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # Create navigation bar
        self.nav_bar = QWidget()
        self.nav_layout = QHBoxLayout(self.nav_bar)
        self.nav_layout.setContentsMargins(2, 0, 2, 0)
        self.nav_layout.setSpacing(2)

        # Create navigation buttons
        self.settings_button = self.create_nav_button("Settings", 0)
        self.routes_button = self.create_nav_button("Routes", 1)
        self.path_button = self.create_nav_button("Path", 2)  # Add Path button

        # Add buttons to nav bar
        self.nav_layout.addWidget(self.settings_button)
        self.nav_layout.addWidget(self.routes_button)
        self.nav_layout.addWidget(self.path_button)  # Add Path button to nav bar

        # Add nav bar to main layout
        self.main_layout.addWidget(self.nav_bar)

        # Create stacked widget to hold different pages
        self.stacked_widget = QStackedWidget()

        # Create page widgets
        self.settings_page = SettingsWidget(self.config_manager, self.parent)
        self.routes_page = RouteFilesWidget(self.parent)
        self.path_page = PathViewWidget(self.parent)  # Add Path page

        # Add pages to stacked widget
        self.stacked_widget.addWidget(self.settings_page)
        self.stacked_widget.addWidget(self.routes_page)
        self.stacked_widget.addWidget(self.path_page)  # Add Path page to stack

        # Add stacked widget to main layout
        self.main_layout.addWidget(self.stacked_widget)

        # Set the main widget as the dock widget's content
        self.setWidget(self.main_widget)

        # Default to settings page
        self.stacked_widget.setCurrentIndex(0)
        self.update_button_styles(0)

    def create_nav_button(self, text, page_index):
        """Create a navigation button that switches to the specified page index"""
        button = QPushButton(text)
        button.setCheckable(True)
        button.setMinimumHeight(30)
        button.clicked.connect(lambda: self.switch_page(page_index))
        return button

    def switch_page(self, page_index):
        """Switch to the specified page and update button styles"""
        self.stacked_widget.setCurrentIndex(page_index)
        self.update_button_styles(page_index)
        logger.info(f"Switched to page {page_index}")

    def update_path_page(self):
        """Update the path page view"""
        self.path_page.update_view()
        logger.info("Path page updated")

    def get_tangent_at_node(self, node):
        """Get the tangent at the selected node"""
        return self.parent.get_tangent_at_node(node)
    
    def set_tangent_at_node(self, node, tangent):
        """Set the tangent at the selected node"""
        self.parent.set_tangent_at_node(node, tangent)

    def set_selected_node(self, node):
        """Set the selected node in the path page"""
        self.path_page.set_selected_node(node)

    def update_button_styles(self, active_index):
        """Update button styles to show which one is active"""
        buttons = [self.settings_button, self.routes_button, self.path_button]  # Add path_button

        for i, button in enumerate(buttons):
            if i == active_index:
                button.setChecked(True)
            else:
                button.setChecked(False)

    def create_placeholder_page(self, title):
        """Create a placeholder page with black background"""
        page = QWidget()
        layout = QVBoxLayout(page)

        # Set black background
        page.setStyleSheet("background-color: black;")

        return page

    # Forward methods to the appropriate pages
    def set_current_coordinates(self, x, y):
        self.settings_page.set_current_coordinates(x, y)

    def refresh_route_files(self):
        """Refresh the route files list"""
        if self.stacked_widget.currentIndex() == 1:  # If routes page is active
            self.routes_page.refresh_files()

    def load_route_file(self, file_path):
        """Load a route file"""
        if hasattr(self.parent, "load_route_file"):
            self.parent.load_route_file(file_path)
