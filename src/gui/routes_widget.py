import logging
import os

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QFrame,
    QLabel,
    QPushButton,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from utilities import sd_card

logger = logging.getLogger(__name__)

ROUTE_ROLE = Qt.ItemDataRole.UserRole


class RouteFilesWidget(QWidget):
    """The routes folder as a tree: one level of subfolders ("suite", "archive")
    with their routes underneath, and root routes at the top level. Double-click a
    route to load it. The folder holding the current route starts expanded, the
    others collapsed, so a long archive does not bury the routes in use."""

    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        header = QLabel("Route Files")
        header.setStyleSheet("font-size: 16px; font-weight: bold;")
        main_layout.addWidget(header)

        self.file_tree = QTreeWidget()
        self.file_tree.setHeaderHidden(True)
        self.file_tree.setAlternatingRowColors(True)
        self.file_tree.itemDoubleClicked.connect(self.on_item_double_clicked)
        main_layout.addWidget(self.file_tree, 1)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setFrameShadow(QFrame.Shadow.Sunken)
        main_layout.addWidget(separator)

        refresh_button = QPushButton("Refresh Files")
        refresh_button.clicked.connect(self.refresh_files)
        main_layout.addWidget(refresh_button)

        self.refresh_files()

    def routes_folder(self):
        """The routes folder from config, as held by the GUI manager."""
        return getattr(self.parent, "routes_folder_path", None)

    def current_route(self):
        return getattr(self.parent, "current_working_file", None)

    def refresh_files(self):
        """Rebuild the tree from the configured routes folder."""
        logger.info("Refreshing route files list")
        self.file_tree.clear()

        folder = self.routes_folder()
        if not folder:
            self._message("No routes folder set (Settings > Routes Folder)")
            return
        if not os.path.isdir(folder):
            self._message(f"Routes folder not found: {folder}")
            return

        names = sd_card.route_names(folder)
        if not names:
            self._message(f"No route files found in {folder}")
            return

        current = self.current_route() or ""
        current_folder = current.split("/", 1)[0] if "/" in current else None

        folders = {}
        bold = QFont()
        bold.setBold(True)
        for name in names:
            if "/" in name:
                sub, route = name.split("/", 1)
                if sub not in folders:
                    item = QTreeWidgetItem([sub])
                    item.setFont(0, bold)
                    item.setData(0, ROUTE_ROLE, None)
                    folders[sub] = item
                    self.file_tree.addTopLevelItem(item)
                child = QTreeWidgetItem([route])
                child.setData(0, ROUTE_ROLE, name)
                folders[sub].addChild(child)
            else:
                item = QTreeWidgetItem([name])
                item.setData(0, ROUTE_ROLE, name)
                self.file_tree.addTopLevelItem(item)

        for sub, item in folders.items():
            item.setExpanded(sub == current_folder)

    def _message(self, text):
        item = QTreeWidgetItem([text])
        item.setData(0, ROUTE_ROLE, None)
        item.setFlags(Qt.ItemFlag.ItemIsEnabled)
        self.file_tree.addTopLevelItem(item)

    def on_item_double_clicked(self, item, column=0):
        name = item.data(0, ROUTE_ROLE)
        if name is None:
            item.setExpanded(not item.isExpanded())
            return
        file_path = name + ".txt"
        logger.info(f"File double-clicked: {file_path}")
        item.setSelected(True)
        if hasattr(self.parent, "load_route_file"):
            self.parent.load_route_file(file_path)
