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
        self.spin_intake = False
        self.clamp_goal = False
        self.doink = False
        self.is_reverse_node = False
        self.turn = 0
        self.lb = 0
        self.wait_time = 0
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

        spin_menu = QMenu("Spin Intake")
        attributes_menu.addMenu(spin_menu)

        spin_options = {
            "Don't spin intake": 0,
            "Spin intake": 1,
            "Spin intake in reverse": -1,
        }

        spin_action_group = QActionGroup(spin_menu)
        spin_action_group.setExclusive(True)

        for option, value in spin_options.items():
            action = QAction(option, spin_menu, checkable=True)
            action.setChecked(self.spin_intake == value)
            action.setData(value)
            spin_action_group.addAction(action)
            spin_menu.addAction(action)

        spin_action_group.triggered.connect(self.set_spin_intake)

        clamp_action = QAction("Clamp Goal", checkable=True)
        clamp_action.setChecked(self.clamp_goal)
        clamp_action.triggered.connect(self.toggle_clamp_goal)
        attributes_menu.addAction(clamp_action)

        doink_action = QAction("Toggle Doinker", checkable=True)
        doink_action.setChecked(self.doink)
        doink_action.triggered.connect(self.toggle_doinker)
        attributes_menu.addAction(doink_action)

        stop_action = QAction("Stop at Node", checkable=True)
        stop_action.setChecked(self.stop)
        stop_action.triggered.connect(self.toggle_stop)
        attributes_menu.addAction(stop_action)

        lb_action = QAction("LB Value: " + str(self.lb))
        lb_action.triggered.connect(self.set_lb)
        attributes_menu.addAction(lb_action)

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

    def has_action(self):
        return (
            self.spin_intake
            or self.clamp_goal
            or self.is_reverse_node
            or self.stop
            or self.turn != 0
            or self.lb != 0
            or self.wait_time != 0
        )

    def is_stop_node(self):
        return self.stop

    def set_spin_intake(self, action):
        self.spin_intake = action.data()
        logger.info(f"Spin Intake: {self.spin_intake}")

    def toggle_clamp_goal(self):
        self.clamp_goal = not self.clamp_goal
        logger.info(f"Clamp Goal: {self.clamp_goal}")

    def toggle_doinker(self):
        self.doink = not self.doink
        logger.info(f"Doinker: {self.doink}")

    def set_lb(self):
        # Get the position of the node in screen coordinates
        scene_pos = self.scenePos()
        view_pos = self.scene().views()[0].mapFromScene(scene_pos)
        screen_pos = self.scene().views()[0].viewport().mapToGlobal(view_pos)

        # Create the dialog
        dialog = QInputDialog(self.scene().views()[0])
        dialog.setWindowTitle("Set LB")
        dialog.setLabelText("Enter value (0-4):")
        dialog.setIntRange(0, 4)
        dialog.setIntValue(self.lb)

        # Set the position of the dialog
        dialog.move(
            int(screen_pos.x() + self.radius), int(screen_pos.y() + self.radius)
        )

        # Show the dialog and get the result
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            self.lb = dialog.intValue()
            self.parent.update_path()
            logger.info(f"LB value set to: {self.lb}")

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
