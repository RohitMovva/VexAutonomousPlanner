import logging

from PyQt6.QtCore import QPoint, QPointF, QRectF, Qt, QObject
from PyQt6.QtGui import QAction, QActionGroup, QColor, QPainter
from PyQt6.QtWidgets import QGraphicsItem, QInputDialog, QMenu, QWidget

from utilities import config_manager

logger = logging.getLogger(__name__)


# Node that stores data for auton route
class Node(QGraphicsItem):
    def __init__(self, x: float, y: float, parent=None, radius=15.0):
        super().__init__()
        self.widget = QWidget()

        self.parent = parent
        self.is_start_node = False
        self.is_end_node = False
        self.is_reverse_node = False
        self.turn = 0
        self.wait_time = 0
        self.dragging = False

        self.locked = False
        self.visible = True

        self.actions = list(self.parent.config_manager.get_section("actions"))
        self.action_values = [0 for _ in self.actions]

        self.offset = QPoint(0, 0)
        self.radius = radius
        self.stop = False
        self.setPos(x, y)
        self.setAcceptHoverEvents(True)
        self.image_rect = QRectF(0, 0, 2000, 2000)

        # Scale pixel value down to account for the extra padding that isn't part of the field on either side, scale to between 0-1, subtract 0.5 to center and turn into inches
        self.image_size = 2000
        self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 145.308474301
        self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 145.308474301

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.drag_start_position = None

    def get_abs_x(self):
        self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 145.308474301
        return self.abs_x

    def get_abs_y(self):
        self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 145.308474301
        return self.abs_y

    def boundingRect(self):
        return QRectF(-self.radius, -self.radius, 2 * self.radius, 2 * self.radius)

    def paint(self, painter, option, widget):
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        brushColor = None
        if self.is_start_node:
            brushColor = QColor("green")
        elif self.is_end_node:
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
        if event.button() == Qt.MouseButton.LeftButton and not self.locked:
            self.drag_start_position = event.pos()
        self.parent.set_selected_node(self)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MouseButton.LeftButton and self.drag_start_position and not self.locked:
            drag_distance = event.pos() - self.drag_start_position
            self.setPos(self.pos() + drag_distance)
            self.drag_start_position = event.pos()
            self.parent.update_path()
        super().mouseMoveEvent(event)

    def set_position(self, x, y):
        self.abs_x = x
        self.abs_y = y
        
        new_pos = QPointF(((x / (12.3266567842 * 12)) + 0.5) * self.image_size, ((y / (12.3266567842 * 12)) + 0.5) * self.image_size)
        self.setPos(new_pos)
        self.parent.update_path()
        # self.update()

    def set_visible(self, visible: bool):
        self.visible = visible
        if visible:
            self.show()
        else:
            self.hide()
        logger.info(f"Node visibility set to: {self.visible}")

    def is_visible(self):
        return self.visible
    
    def set_locked(self, locked: bool):
        self.locked = locked
        logger.info(f"Node locked set to: {self.locked}")
    
    def is_locked(self):
        return self.locked

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and not self.locked:
            self.drag_start_position = None
            self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 145.308474301
            self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 145.308474301
            self.parent.update_path()
            # self.parent.save()
        super().mouseReleaseEvent(event)

    def itemChange(self, change, value: QPointF):
        if (
            change == QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and self.scene()
        ):
            if (self.locked):
                return self.pos()
            new_pos = value
            if self.image_rect:
                # Restrict the movement within the image boundaries
                new_pos.setX(
                    max(
                        min(new_pos.x(), self.image_rect.right() - self.radius),
                        self.image_rect.left() + self.radius,
                    )
                )
                new_pos.setY(
                    max(
                        min(new_pos.y(), self.image_rect.bottom() - self.radius),
                        self.image_rect.top() + self.radius,
                    )
                )

            self.update()  # Trigger a repaint to update the coordinates
            return new_pos
        return super().itemChange(change, value)

    def contextMenuEvent(self, event):
        context_menu = QMenu()

        attributes_menu = QMenu("Attributes")
        node_menu = QMenu("Node Actions")

        start_action = QAction("Start Node", checkable=True)
        start_action.setChecked(self.is_start_node)
        start_action.triggered.connect(self.toggle_start_node)
        attributes_menu.addAction(start_action)

        end_action = QAction("End Node", checkable=True)
        end_action.setChecked(self.is_end_node)
        end_action.triggered.connect(self.toggle_end_node)
        attributes_menu.addAction(end_action)

        stop_action = QAction("Stop at Node", checkable=True)
        stop_action.setChecked(self.stop)
        stop_action.triggered.connect(self.toggle_stop)
        attributes_menu.addAction(stop_action)

        reverse_action = QAction("Reverse", checkable=True)
        reverse_action.setChecked(self.is_reverse_node)
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

        for i, action in enumerate(self.actions):
            new_action = QAction(f"{action}: {self.action_values[i]}")
            new_action.triggered.connect(lambda checked, p=i: self.action_handler(p))
            attributes_menu.addAction(new_action)
                
        context_menu.addMenu(attributes_menu)
        context_menu.addMenu(node_menu)

        context_menu.exec(event.screenPos())

    def toggle_start_node(self):
        self.is_start_node = not self.is_start_node
        if self.is_start_node:
            self.parent.set_start_node(self)
            if self.is_end_node:
                self.parent.clear_end_node()
                self.is_end_node = False
        else:
            self.parent.clear_start_node()

        self.parent.update_path()
        logger.info(f"Start Node: {self.is_start_node}")

    def toggle_end_node(self):
        self.is_end_node = not self.is_end_node
        if self.is_end_node:
            self.parent.set_end_node(self)
            if self.is_start_node:
                self.parent.clear_start_node()
                self.is_start_node = False
        else:
            self.parent.clear_end_node()
        self.update()
        self.parent.update_path()
        logger.info(f"End Node: {self.is_end_node}")

    def get_name(self):
        if (self.is_start_node):
            return "Start Node"
        elif (self.is_end_node):
            return "End Node"
        else:
            return "Node"

    def has_action(self):
        for action_value in self.action_values:
            if action_value != 0:
                return True
            
        return (
            self.is_reverse_node
            or self.stop
            or self.turn != 0
            or self.wait_time != 0
        )
    
    def get_action_values(self):
        return self.action_values
    
    def set_action_values(self, values):
        if len(values) == len(self.action_values):
            self.action_values = values
            self.parent.update_path()
            logger.info(f"Action values set to: {self.action_values}")
        else:
            logger.error("Invalid action values length")

    def is_stop_node(self):
        return self.stop

    def toggle_reverse(self):
        self.is_reverse_node = not self.is_reverse_node
        self.parent.update_path()
        logger.info(f"Reverse Node: {self.is_reverse_node}")

    def set_turn(self):
        # Get the position of the node in screen coordinates
        scene_pos = self.scenePos()
        view_pos = self.scene().views()[0].mapFromScene(scene_pos)
        screen_pos = self.scene().views()[0].viewport().mapToGlobal(view_pos)

        # Create the dialog
        dialog = QInputDialog(self.scene().views()[0])
        dialog.setWindowTitle("Set Turn")
        dialog.setLabelText("Enter turn (-180-180):")
        dialog.setIntRange(-180, 180)
        dialog.setIntValue(self.turn)

        # Set the position of the dialog
        dialog.move(
            int(screen_pos.x() + self.radius), int(screen_pos.y() + self.radius)
        )

        # Show the dialog and get the result
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            self.turn = dialog.intValue()
            self.parent.update_path()
            logger.info(f"Turn set to: {self.turn}")

    def set_wait(self):
        # Get the position of the node in screen coordinates
        scene_pos = self.scenePos()
        view_pos = self.scene().views()[0].mapFromScene(scene_pos)
        screen_pos = self.scene().views()[0].viewport().mapToGlobal(view_pos)

        # Create the dialog
        dialog = QInputDialog(self.scene().views()[0])
        dialog.setWindowTitle("Set Wait Time")
        dialog.setLabelText("Enter time (seconds):")
        dialog.setInputMode(QInputDialog.InputMode.DoubleInput)
        dialog.setDoubleValue(self.wait_time)
        dialog.setDoubleMinimum(0)

        # Set the position of the dialog
        dialog.move(
            int(screen_pos.x() + self.radius), int(screen_pos.y() + self.radius)
        )

        # Show the dialog and get the result
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            self.wait_time = dialog.doubleValue()
            logger.info(f"Wait time set to: {self.wait_time}")

    def delete_node(self):
        self.parent.remove_node(self)
        self.scene().removeItem(self)
        logger.info(f"Node at ({self.abs_x}, {self.abs_y}) deleted")

    def toggle_stop(self):
        self.stop = not self.stop
        logger.info(f"Stop at Node: {self.stop}")

    def insert_node_before(self):
        logger.info(f"Inserting node before at {self.pos()}")
        new_point = QPointF(self.pos().x() + 5, self.pos().y() + 5)
        self.parent.add_node(new_point, self.parent.index_of(self))

    def insert_node_after(self):
        logger.info(f"Inserting node after at {self.pos()}")
        new_point = QPointF(self.pos().x() + 5, self.pos().y() + 5)
        self.parent.add_node(new_point, self.parent.index_of(self) + 1)

    def action_handler(self, action_index):
        action_name = self.actions[action_index]

        # Get the position of the node in screen coordinates
        scene_pos = self.scenePos()
        view_pos = self.scene().views()[0].mapFromScene(scene_pos)
        screen_pos = self.scene().views()[0].viewport().mapToGlobal(view_pos)

        # Create the dialog
        dialog = QInputDialog(self.scene().views()[0])
        dialog.setWindowTitle(f"Set {action_name}")
        dialog.setLabelText(f"Enter {action_name} value:")
        dialog.setDoubleRange(-1000, 1000)
        dialog.setDoubleValue(self.action_values[action_index])

        # Set the position of the dialog
        dialog.move(
            int(screen_pos.x() + self.radius), int(screen_pos.y() + self.radius)
        )

        # Show the dialog and get the result
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            self.action_values[action_index] = dialog.doubleValue()
            self.parent.update_path()
            logger.info(f"{action_name} value set to: {self.action_values[action_index]}")


    def __str__(self):
        return (
            "["
            + str(self.is_start_node)
            + " "
            + str(self.is_end_node)
            + " "
            + str(self.is_reverse_node)
            + " "
            + str(self.turn)
            + " "
            + str(self.wait_time)
            + " "
            + str(self.has_action())
            + " "
            + "]"
        )
