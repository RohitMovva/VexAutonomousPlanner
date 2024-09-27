from PyQt6.QtCore import QPoint, QPointF, QRectF, Qt
from PyQt6.QtGui import QAction, QBrush, QColor, QFont, QMouseEvent, QPainter, QPen
from PyQt6.QtWidgets import QGraphicsItem, QInputDialog, QMenu, QWidget

from bezier.cubic_bezier import *
from bezier.quadratic_bezier import *


# Node that stores data for auton route
class Node(QGraphicsItem):
    def __init__(self, x, y, parent=None, radius=15.0):
        super().__init__()
        self.widget = QWidget()

        # Scale pixel value down to account for the extra padding that isn't part of the field on either side, scale to between 0-1, subtract 0.5 to center and turn into inches
        self.imageSize = 2000
        self.absX = ((self.x() / (self.imageSize)) - 0.5) * 12.3266567842 * 12
        self.absY = ((self.y() / (self.imageSize)) - 0.5) * 12.3266567842 * 12

        self.parent = parent
        self.isStartNode = False
        self.isEndNode = False
        self.spinIntake = False
        self.clampGoal = False
        self.isReverseNode = False
        self.turn = 0
        self.wait_time = 0
        self.dragging = False
        self.offset = QPoint(0, 0)
        self.radius = radius
        self.setPos(x, y)
        self.setAcceptHoverEvents(True)

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.drag_start_position = None
        # self.setWidget(self.widget)

    def boundingRect(self):
        return QRectF(-self.radius, -self.radius, 2 * self.radius, 2 * self.radius)

    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        brushColor = None
        if self.isStartNode:
            brushColor = QColor("green")
        elif self.isEndNode:
            brushColor = QColor("red")
        elif self.has_action():
            brushColor = QColor("#1338BE")
        else:
            brushColor = QColor("#1F456E")

        brushColor.setAlphaF(0.65)
        painter.setBrush(brushColor)

        # painter.setOpacity(.5)
        painter.drawEllipse(self.boundingRect())

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drag_start_position = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MouseButton.LeftButton and self.drag_start_position:
            drag_distance = event.pos() - self.drag_start_position
            self.setPos(self.pos() + drag_distance)
            self.drag_start_position = event.pos()
            self.parent.update_path()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drag_start_position = None
            self.parent.update_path()
            self.parent.save()
        super().mouseReleaseEvent(event)

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self.update()  # Trigger a repaint to update the coordinates
        return super().itemChange(change, value)

    def contextMenuEvent(self, event):
        context_menu = QMenu()

        attributes_menu = QMenu("Attributes")
        node_menu = QMenu("Node Actions")

        start_action = QAction("Start Node", checkable=True)
        start_action.setChecked(self.isStartNode)
        start_action.triggered.connect(self.toggle_start_node)
        attributes_menu.addAction(start_action)

        end_action = QAction("End Node", checkable=True)
        end_action.setChecked(self.isEndNode)
        end_action.triggered.connect(self.toggle_end_node)
        attributes_menu.addAction(end_action)

        spin_action = QAction("Spin Intake", checkable=True)
        spin_action.setChecked(self.spinIntake)
        spin_action.triggered.connect(self.toggle_spin_intake)
        attributes_menu.addAction(spin_action)

        clamp_action = QAction("Clamp Goal", checkable=True)
        clamp_action.setChecked(self.clampGoal)
        clamp_action.triggered.connect(self.toggle_clamp_goal)
        attributes_menu.addAction(clamp_action)

        reverse_action = QAction("Reverse", checkable=True)
        reverse_action.setChecked(self.isReverseNode)
        reverse_action.triggered.connect(self.toggle_reverse)
        attributes_menu.addAction(reverse_action)

        turn_action = QAction("Turn Value: " + str(self.turn))
        turn_action.triggered.connect(self.set_turn)
        attributes_menu.addAction(turn_action)

        wait_action = QAction("Wait time: " + str(self.wait_time))
        wait_action.triggered.connect(self.set_wait)
        attributes_menu.addAction(wait_action)

        delete_action = QAction("Delete Node")
        delete_action.triggered.connect(self.delete_node)
        node_menu.addAction(delete_action)

        insert_node_before_action = QAction("Insert Node Before")
        insert_node_before_action.triggered.connect(self.insert_node_before)
        node_menu.addAction(insert_node_before_action)

        insert_node_after_action = QAction("Insert Node After")
        insert_node_after_action.triggered.connect(self.insert_node_after)
        node_menu.addAction(insert_node_after_action)

        context_menu.addMenu(attributes_menu)
        context_menu.addMenu(node_menu)

        context_menu.exec(event.screenPos())

    def toggle_start_node(self):
        self.isStartNode = not self.isStartNode
        if self.isStartNode:
            self.parent.set_start_node(self)
            if self.isEndNode:
                self.parent.clear_end_node()
                self.isEndNode = False
        else:
            self.parent.clear_start_node()

        self.parent.update_path()
        print(f"Start Node: {self.isStartNode}")

    def toggle_end_node(self):
        self.isEndNode = not self.isEndNode
        if self.isEndNode:
            self.parent.set_end_node(self)
            if self.isStartNode:
                self.parent.clear_start_node()
                self.isStartNode = False
        else:
            self.parent.clear_end_node()
        self.update()
        self.parent.update_path()
        print(f"End Node: {self.isEndNode}")

    def has_action(self):
        return (
            self.spinIntake
            or self.clampGoal
            or self.isReverseNode
            or self.turn != 0
            or self.wait_time != 0
        )

    def toggle_spin_intake(self):
        self.spinIntake = not self.spinIntake
        print(f"Spin Intake: {self.spinIntake}")

    def toggle_clamp_goal(self):
        self.clampGoal = not self.clampGoal
        print(f"Clamp Goal: {self.clampGoal}")

    def toggle_reverse(self):
        self.isReverseNode = not self.isReverseNode
        self.parent.update_path()
        print(f"Reverse Node: {self.isReverseNode}")

    def set_turn(self):
        value, ok = QInputDialog.getInt(
            self, "Set Turn", "Enter turn (0-360):", self.turn, 0, 360
        )
        if ok:
            self.turn = value
            self.parent.update_path()
            print(f"Turn set to: {self.turn}")

    def set_wait(self):
        value, ok = QInputDialog.getInt(
            self, "Set Wait Time", "Enter time (seconds):", self.wait_time, 0
        )
        if ok:
            self.wait_time = value
            print(f"Wait time set to: {self.wait_time}")

    def delete_node(self):
        self.parent.remove_node(self)
        self.scene().removeItem(self)
        print(f"Node at ({self.x}, {self.y}) deleted")

    def insert_node_before(self):
        new_point = QPointF(self.pos().x() + 5, self.pos().y() + 5)
        self.parent.add_node(new_point, self.parent.index_of(self))

    def insert_node_after(self):
        print(self.pos())
        new_point = QPointF(self.pos().x() + 5, self.pos().y() + 5)
        self.parent.add_node(new_point, self.parent.index_of(self) + 1)

    def __str__(self):
        return (
            "["
            + str(self.isStartNode)
            + " "
            + str(self.isEndNode)
            + " "
            + str(self.isReverseNode)
            + " "
            + str(self.turn)
            + " "
            + str(self.wait_time)
            + " "
            + str(self.has_action())
            + " "
            + "]"
        )
