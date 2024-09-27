# Click listener object
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QMouseEvent
from PyQt6.QtWidgets import QLabel


class ClickableLabel(QLabel):
    def __init__(self, parent=None, gui_instance=None):
        super().__init__(parent)
        self.gui_instance = gui_instance

        self.setMouseTracking(True)

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            x = (
                int(event.position().x()) + 10
            )  # I have no clue on God's green earth on why this is needed but it is
            y = int(event.position().y()) + 10
            print(f"Mouse clicked at ({x}, {y})")
            self.gui_instance.add_node(x, y)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent):
        x = int(event.position().x()) + 10
        y = int(event.position().y()) + 10
        scale = 700
        x = round(((x / (scale)) - 0.5) * 12**2, 2)
        y = round(((y / (scale)) - 0.5) * 12**2, 2)
        # a*b/c/b =
        self.gui_instance.update_coordinate_display(x, y)
        super().mouseMoveEvent(event)
