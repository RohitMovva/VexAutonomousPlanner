import logging

from PyQt6.QtWidgets import (
    QFrame,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from utilities.file_management import get_paths_files

logger = logging.getLogger(__name__)


class RouteFilesWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent

        # Create main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Add header
        header = QLabel("Route Files")
        header.setStyleSheet("font-size: 16px; font-weight: bold;")
        main_layout.addWidget(header)

        # Create list widget for files
        self.file_list = QListWidget()
        self.file_list.setAlternatingRowColors(True)

        # Enable file selection
        self.file_list.itemDoubleClicked.connect(self.on_file_double_clicked)

        # Create scroll area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(self.file_list)

        # Add scroll area to main layout
        main_layout.addWidget(
            scroll_area, 1
        )  # 1 is the stretch factor to make it expand

        # Add separator
        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        main_layout.addWidget(separator)

        # Add refresh button
        refresh_button = QPushButton("Refresh Files")
        refresh_button.clicked.connect(self.refresh_files)
        main_layout.addWidget(refresh_button)

        # Initial file load
        self.refresh_files()

    def refresh_files(self):
        """Refresh the list of route files"""
        logger.info("Refreshing route files list")
        self.file_list.clear()

        try:
            # Call the parent's get_path_files method
            try:
                file_paths = get_paths_files()
                if file_paths:
                    # Add each file to the list
                    for file_path in file_paths:
                        item = QListWidgetItem(file_path[:-4])
                        self.file_list.addItem(item)
                else:
                    self.file_list.addItem("No route files found")

            except Exception as e:
                logger.error(f"Error loading route files: {str(e)}")
                self.file_list.addItem(f"Error loading route files: {str(e)}")
        except Exception as e:
            error_msg = f"Error loading route files: {str(e)}"
            self.file_list.addItem(error_msg)
            logger.error(error_msg)

    def on_file_double_clicked(self, item):
        """Handle double-click on a file"""
        file_path = item.text() + ".txt"
        logger.info(f"File double-clicked: {file_path}")
        item.setSelected(True)

        # Pass up calls
        if hasattr(self.parent, "load_route_file"):
            self.parent.load_route_file(file_path)
