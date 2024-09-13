from PyQt6.QtWidgets import QWidget, QMenu, QInputDialog
from PyQt6.QtGui import QMouseEvent, QPainter, QColor, QAction
from PyQt6.QtCore import Qt, QPoint
from bezier.quadratic_bezier import *
from bezier.cubic_bezier import *

# Node that stores data for auton route
class Node(QWidget):
    def __init__(self, x, y, parent=None, gui_instance=None):
        super().__init__(parent)
        self.x = x
        self.y = y
        print(x, y)
        # Scale pixel value down to account for the extra padding that isn't part of the field on either side, scale to between 0-1, subtract 0.5 to center and turn into inches
        self.scale = 700
        self.absX = ((self.x / (self.scale)) - 0.5) * 12**2
        self.absY = ((self.y / (self.scale)) - 0.5) * 12**2

        self.gui_instance = gui_instance
        self.isStartNode = False
        self.isEndNode = False
        self.spinIntake = False
        self.clampGoal = False
        self.isReverseNode = False
        self.turn = 0
        self.wait_time = 0
        self.setFixedSize(10, 10)
        self.move(x-5, y-5)
        self.dragging = False
        self.offset = QPoint(0, 0)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        if self.isStartNode:
            painter.setBrush(QColor("green"))
        elif self.isEndNode:
            painter.setBrush(QColor("red"))
        elif self.has_action():
            painter.setBrush(QColor("#1338BE"))
        else:
            painter.setBrush(QColor("#1F456E"))

        painter.drawEllipse(0, 0, self.width(), self.height())

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.RightButton:
            self.show_context_menu(event.globalPosition().toPoint())
        elif event.button() == Qt.MouseButton.LeftButton:
            self.dragging = True
            self.offset = event.position().toPoint()
        super().mousePressEvent(event)
        self.gui_instance.update_lines()

    def mouseMoveEvent(self, event: QMouseEvent):
        if self.dragging:
            new_pos = self.mapToParent(event.position().toPoint() - self.offset)
            self.x = new_pos.x()+5
            self.y = new_pos.y()+5
            self.absX = ((self.x * (self.scale*2000-self.scale*34*2)/(self.scale*2000)) / (self.scale*2000-self.scale*34*2) - 0.5) * 12**2
            self.absY = ((self.y * (self.scale*2000-self.scale*34*2)/(self.scale*2000)) / (self.scale*2000-self.scale*34*2) - 0.5) * 12**2
            self.move(new_pos)
            self.gui_instance.update_lines()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            self.dragging = False
        if (self.gui_instance.current_working_file != None):
            self.gui_instance.auto_save()
        super().mouseReleaseEvent(event)

    def show_context_menu(self, pos):
        context_menu = QMenu(self)

        attributes_menu = QMenu("Attributes", self)
        node_menu = QMenu("Node Actions", self)

        start_action = QAction('Start Node', self, checkable=True)
        start_action.setChecked(self.isStartNode)
        start_action.triggered.connect(self.toggle_start_node)
        attributes_menu.addAction(start_action)

        end_action = QAction('End Node', self, checkable=True)
        end_action.setChecked(self.isEndNode)
        end_action.triggered.connect(self.toggle_end_node)
        attributes_menu.addAction(end_action)

        spin_action = QAction('Spin Intake', self, checkable=True)
        spin_action.setChecked(self.spinIntake)
        spin_action.triggered.connect(self.toggle_spin_intake)
        attributes_menu.addAction(spin_action)

        clamp_action = QAction('Clamp Goal', self, checkable=True)
        clamp_action.setChecked(self.clampGoal)
        clamp_action.triggered.connect(self.toggle_clamp_goal)
        attributes_menu.addAction(clamp_action)

        reverse_action = QAction('Reverse', self, checkable=True)
        reverse_action.setChecked(self.isReverseNode)
        reverse_action.triggered.connect(self.toggle_reverse)
        attributes_menu.addAction(reverse_action)

        turn_action = QAction('Turn Value: ' + str(self.turn), self)
        turn_action.triggered.connect(self.set_turn)
        attributes_menu.addAction(turn_action)

        wait_action = QAction('Wait time: ' + str(self.wait_time), self)
        wait_action.triggered.connect(self.set_wait)
        attributes_menu.addAction(wait_action)

        delete_action = QAction('Delete Node', self)
        delete_action.triggered.connect(self.delete_node)
        node_menu.addAction(delete_action)

        insert_node_before_action = QAction('Insert Node Before', self)
        insert_node_before_action.triggered.connect(self.insert_node_before)
        node_menu.addAction(insert_node_before_action)

        insert_node_after_action = QAction('Insert Node After', self)
        insert_node_after_action.triggered.connect(self.insert_node_after)
        node_menu.addAction(insert_node_after_action)

        context_menu.addMenu(attributes_menu)
        context_menu.addMenu(node_menu)

        context_menu.exec(pos)

    def toggle_start_node(self):
        self.isStartNode = not self.isStartNode
        if self.isStartNode:
            self.gui_instance.set_start_node(self)
            if self.isEndNode:
                self.gui_instance.clear_end_node()
                self.isEndNode = False
        else:
            self.gui_instance.clear_start_node()
        self.repaint()
        self.gui_instance.update_lines()
        print(f"Start Node: {self.isStartNode}")

    def toggle_end_node(self):
        self.isEndNode = not self.isEndNode
        if self.isEndNode:
            self.gui_instance.set_end_node(self)
            if self.isStartNode:
                self.gui_instance.clear_start_node()
                self.isStartNode = False
        else:
            self.gui_instance.clear_end_node()
        self.update()
        self.gui_instance.update_lines()
        print(f"End Node: {self.isEndNode}")

    def has_action(self):
        return self.spinIntake or self.clampGoal or self.isReverseNode or self.turn != 0 or self.wait_time != 0

    def toggle_spin_intake(self):
        self.spinIntake = not self.spinIntake
        print(f"Spin Intake: {self.spinIntake}")

    def toggle_clamp_goal(self):
        self.clampGoal = not self.clampGoal
        print(f"Clamp Goal: {self.clampGoal}")

    def toggle_reverse(self):
        self.isReverseNode = not self.isReverseNode
        print(f"Reverse Node: {self.isReverseNode}")

    def set_turn(self):
        value, ok = QInputDialog.getInt(self, "Set Turn", "Enter turn (0-360):", self.turn, 0, 360)
        if ok:
            self.turn = value
            print(f"Turn set to: {self.turn}")

    def set_wait(self):
        value, ok = QInputDialog.getInt(self, "Set Wait Time", "Enter time (seconds):", self.wait_time, 0)
        if ok:
            self.wait_time = value
            print(f"Wait time set to: {self.wait_time}")

    def delete_node(self):
        self.gui_instance.remove_node(self)
        self.close()
        print(f"Node at ({self.x}, {self.y}) deleted")

    def insert_node_before(self):
        self.gui_instance.add_node(self.x+5, self.y+5, self.gui_instance.index_of(self))
    
    def insert_node_after(self):
        self.gui_instance.add_node(self.x+5, self.y+5, self.gui_instance.index_of(self)+1)

    def __str__(self):
        return "[" + str(self.isStartNode) + " " + str(self.isEndNode) + " " + str(self.has_action) + "]"
