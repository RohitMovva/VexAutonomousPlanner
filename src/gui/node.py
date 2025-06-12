import logging

from PyQt6.QtCore import QPoint, QPointF, QRectF, Qt
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
        self.spin_intake = False
        self.clamp_goal = False
        self.doink = False
        self.is_reverse_node = False
        self.turn = 0
        self.lb = 0
        self.wait_time = 0
        self.dragging = False

        self.actions = self.parent.config_manager.get_section("actions")

        self.offset = QPoint(0, 0)
        self.radius = radius
        self.stop = False
        self.setPos(x, y)
        self.setAcceptHoverEvents(True)
        self.image_rect = QRectF(0, 0, 2000, 2000)

        # Scale pixel value down to account for the extra padding that isn't part of the field on either side, scale to between 0-1, subtract 0.5 to center and turn into inches
        self.image_size = 2000
        self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 12.3266567842 * 12
        self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 12.3266567842 * 12

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.drag_start_position = None

    def get_abs_x(self):
        self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 12.3266567842 * 12
        return self.abs_x

    def get_abs_y(self):
        self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 12.3266567842 * 12
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
            self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 12.3266567842 * 12
            self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 12.3266567842 * 12
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

        start_action = QAction("Start Node", checkable=True)
        start_action.setChecked(self.is_start_node)
        start_action.triggered.connect(self.toggle_start_node)
        attributes_menu.addAction(start_action)

        end_action = QAction("End Node", checkable=True)
        end_action.setChecked(self.is_end_node)
        end_action.triggered.connect(self.toggle_end_node)
        attributes_menu.addAction(end_action)

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

        reverse_action = QAction("Reverse", checkable=True)
        reverse_action.setChecked(self.is_reverse_node)
        reverse_action.triggered.connect(self.toggle_reverse)
        attributes_menu.addAction(reverse_action)

        turn_action = QAction("Turn Value: " + str(self.turn))
        turn_action.triggered.connect(self.set_turn)
        attributes_menu.addAction(turn_action)

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

        for action in self.actions:
            new_action = QAction(action)
            new_action.triggered.connect(self.action_handler)
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

    def action_handler(self):
        # Get the position of the node in screen coordinates
        scene_pos = self.scenePos()
        view_pos = self.scene().views()[0].mapFromScene(scene_pos)
        screen_pos = self.scene().views()[0].viewport().mapToGlobal(view_pos)

        # Create the dialog
        dialog = QInputDialog(self.scene().views()[0])
        dialog.setWindowTitle("Set Action")
        dialog.setLabelText("Enter Action Value:")
        dialog.setDoubleRange(-1000, 1000)
        dialog.setDoubleValue(self.turn)

        # Set the position of the dialog
        dialog.move(
            int(screen_pos.x() + self.radius), int(screen_pos.y() + self.radius)
        )

        # Show the dialog and get the result
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            self.turn = dialog.doubleValue()
            self.parent.update_path()
            logger.info(f"Turn set to: {self.turn}")


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
