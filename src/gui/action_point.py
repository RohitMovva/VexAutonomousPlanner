import logging

from PyQt6.QtCore import QPoint, QPointF, QRectF, Qt
from PyQt6.QtGui import QAction, QActionGroup, QColor, QPainter
from PyQt6.QtWidgets import QGraphicsItem, QInputDialog, QMenu, QWidget

logger = logging.getLogger(__name__)


class ActionPoint(QGraphicsItem):
    def __init__(self, x: float, y: float, t: float, parent=None, radius=15.0):
        super().__init__()
        self.widget = QWidget()

        self.parent = parent
        self.is_reverse_node = False
        self.turn = 0
        self.wait_time = 0
        self.actions = self.parent.config_manager.get_section("actions")
        self.action_values = [0 for _ in self.actions]
        self.dragging = False
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

        self.t = t

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
        brushColor = QColor("#0a3533")

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
            # Find the closest point on the path to the mouse position
            closest_point, closest_parameter = self.parent.find_closest_point_on_path(
                self.parent.path, self.mapToScene(event.pos())
            )

            self.setPos(closest_point)
            self.drag_start_position = closest_point
            self.t = closest_parameter
            self.parent.update_path()
            logger.info(f"Moved action point to ({closest_point})")

        # super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
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

        stop_action = QAction("Stop at Node", checkable=True)
        stop_action.setChecked(self.stop)
        stop_action.triggered.connect(self.toggle_stop)
        attributes_menu.addAction(stop_action)

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

    def has_action(self):
        for action_value in self.action_values:
            if action_value != 0:
                return True
            
        return (
            self.stop
            or self.wait_time != 0
        )

    def is_stop_node(self):
        return self.stop

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

    def delete_action_point(self):
        self.parent.remove_action_point(self)
        self.scene().removeItem(self)
        logger.info(f"Action point at ({self.abs_x}, {self.abs_y}) deleted")

    
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
        dialog.setDoubleValue(self.turn)

        # Set the position of the dialog
        dialog.move(
            int(screen_pos.x() + self.radius), int(screen_pos.y() + self.radius)
        )

        # Show the dialog and get the result
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            self.action_values[action_index] = dialog.doubleValue()
            self.parent.update_path()
            logger.info(f"{action_name} value set to: {self.action_values[action_index]}")

    def get_action_values(self):
        return self.action_values
    
    def set_action_values(self, values):
        if len(values) == len(self.action_values):
            self.action_values = values
            self.parent.update_path()
            logger.info(f"Action values set to: {self.action_values}")
        else:
            logger.error("Invalid action values length")

    def __str__(self):
        return (
            "["
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
