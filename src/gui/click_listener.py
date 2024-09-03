# Click listener object
from PyQt6.QtWidgets import QLabel
from PyQt6.QtGui import QMouseEvent
from PyQt6.QtCore import Qt
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *
from utilities import *


class ClickableLabel(QLabel):
    def __init__(self, parent=None, gui_instance=None):
        super().__init__(parent)
        self.gui_instance = gui_instance

        self.setMouseTracking(True)

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            x = int(event.position().x()) + 7 # I have no clue on God's green earth on why this is needed but it is
            y = int(event.position().y()) + 7
            print(f"Mouse clicked at ({x}, {y})")
            self.gui_instance.add_node(x, y)
        super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event: QMouseEvent):
        x = int(event.position().x()) + 7
        y = int(event.position().y()) + 7
        scale = 700/2000
        x = round(((x * (scale*2000-scale*34*2)/(scale*2000)) / (scale*2000-scale*34*2) - 0.5) * 12**2, 2)
        y = round(((y * (scale*2000-scale*34*2)/(scale*2000)) / (scale*2000-scale*34*2) - 0.5) * 12**2, 2)

        self.gui_instance.update_coordinate_display(x, y)
        super().mouseMoveEvent(event)