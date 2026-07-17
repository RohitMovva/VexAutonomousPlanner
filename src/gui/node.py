import logging

from PyQt6.QtCore import QPoint, QPointF, QRectF, Qt
from PyQt6.QtGui import QAction, QColor, QPainter, QPen
from PyQt6.QtWidgets import QGraphicsItem, QInputDialog, QMenu, QWidget

logger = logging.getLogger(__name__)

# Stop flags
STOP_AUTO = 0 # Will stop if reverse node or turn node
STOP_FORCED = 1 # Will stop
STOP_DISABLED = 2 # Won't stop


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

        self.actions = self.parent.config_manager.get_value("robot", "actions")
        self.action_values = [0 for _ in self.actions]

        self.offset = QPoint(0, 0)
        self.radius = radius
        self.stop_mode = STOP_AUTO
        self.setPos(x, y)
        self.setAcceptHoverEvents(True)
        self.image_rect = QRectF(0, 0, 2000, 2000)

        # Scale pixel value down to account for the extra padding that isn't part of the field on either side, scale to between 0-1, subtract 0.5 to center and turn into inches
        self.image_size = 2000
        self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 146.446120719
        self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 146.446120719

        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges)
        self.drag_start_position = None
        self._group_drag = False

        self.tangent = None
        self.incoming_magnitude = None
        self.outgoing_magnitude = None
        self.curvature = None

        self.max_velocity = 0
        self.max_acceleration = 0

    def get_abs_x(self):
        self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 146.446120719
        return self.abs_x

    def get_abs_y(self):
        self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 146.446120719
        return self.abs_y

    def set_tangent(self, tangent):
        self.tangent = tangent

    def get_tangent(self, aslist=False):
        if aslist:
            return self.tangent.tolist() if self.tangent is not None else None
        return self.tangent

    def set_incoming_magnitude(self, magnitude):
        self.incoming_magnitude = magnitude

    def get_incoming_magnitude(self):
        return self.incoming_magnitude

    def set_outgoing_magnitude(self, magnitude):
        self.outgoing_magnitude = magnitude

    def get_outgoing_magnitude(self):
        return self.outgoing_magnitude
    
    def set_tangent_derivative(self, derivative):
        self.curvature = derivative

    def get_tangent_derivative(self, aslist=False):
        if aslist:
            return self.curvature.tolist() if self.curvature is not None else None
        return self.curvature

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

        if self.parent.is_node_highlighted(self):
            # Inset by half the pen width so the ring stays inside
            # boundingRect (no geometry change needed).
            painter.setPen(QPen(QColor("#7FDBFF"), 3))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(self.boundingRect().adjusted(1.5, 1.5, -1.5, -1.5))

    def mousePressEvent(self, event):
        self._group_drag = (
            event.button() == Qt.MouseButton.LeftButton
            and self.parent.is_node_selected(self)
            and len(self.parent.selected_nodes) > 1
        )
        if (
            event.button() == Qt.MouseButton.LeftButton
            and not self.parent.is_node_selected(self)
        ): # Node isn't part of group so clear the selection
            self.parent.clear_multi_selection()

        self.parent.set_selected_node(self)
        if event.button() == Qt.MouseButton.LeftButton and not self.locked:
            self.drag_start_position = event.pos()
            self.parent.begin_undo_state()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        dragging = (
            event.buttons() & Qt.MouseButton.LeftButton
            # "is not None", not truthiness: a press exactly on the node
            # center stores QPointF(0, 0), which is falsy.
            and self.drag_start_position is not None
            and not self.locked
        )
        old_pos = self.pos()
        if dragging:
            drag_distance = event.pos() - self.drag_start_position
            self.setPos(self.pos() + drag_distance)
            self.drag_start_position = event.pos()
        # The ItemIsMovable default handler also repositions the node
        # (anchored under the cursor), so the grabbed node's total delta is
        # only known after super() runs - followers replicate that total,
        # keeping the group rigid; each still clamps itself in itemChange.
        super().mouseMoveEvent(event)
        if dragging:
            if self._group_drag:
                actual_delta = self.pos() - old_pos
                for n in self.parent.selected_nodes:
                    if n is not self and not n.locked:
                        n.setPos(n.pos() + actual_delta)
            self.parent.update_path()

    def set_position(self, x, y):
        self.abs_x = x
        self.abs_y = y

        new_pos = QPointF(
            ((x / (12.2038433932 * 12)) + 0.5) * self.image_size,
            ((y / (12.2038433932 * 12)) + 0.5) * self.image_size,
        )
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
            self.abs_x = ((self.x() / (self.image_size)) - 0.5) * 146.446120719
            self.abs_y = ((self.y() / (self.image_size)) - 0.5) * 146.446120719
            if self._group_drag:
                for n in self.parent.selected_nodes:
                    n.abs_x = ((n.x() / (n.image_size)) - 0.5) * 146.446120719
                    n.abs_y = ((n.y() / (n.image_size)) - 0.5) * 146.446120719
            self.parent.update_path()
            self.parent.commit_undo_state(
                "Move nodes" if self._group_drag else "Move node"
            )
            self._group_drag = False
            # self.parent.save()
        super().mouseReleaseEvent(event)

    def itemChange(self, change, value: QPointF):
        if (
            change == QGraphicsItem.GraphicsItemChange.ItemPositionChange
            and self.scene()
        ):
            if self.locked:
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
        selection = self.parent.selected_nodes
        if len(selection) > 1 and self in selection:
            self._batch_context_menu(event, list(selection))
            return

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

        stop_label = "Stop at Node"
        if self.auto_stop():
            stop_label += (
                " (auto)" if self.stop_mode == STOP_AUTO else " (override: no stop)"
            )
        stop_action = QAction(stop_label, checkable=True)
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

        velocity_action = QAction("Max Velocity")
        velocity_action.triggered.connect(self.set_velocity)
        attributes_menu.addAction(velocity_action)

        acceleration_action = QAction("Max Acceleration")
        acceleration_action.triggered.connect(self.set_acceleration)
        attributes_menu.addAction(acceleration_action)

        new_actions = []
        for i, action in enumerate(self.actions):
            new_actions.append(QAction(f"{action}: {self.action_values[i]}"))
            new_actions[i].triggered.connect(lambda checked, p=i: self.action_handler(p))
            attributes_menu.addAction(new_actions[i])

        context_menu.addMenu(attributes_menu)
        context_menu.addMenu(node_menu)

        # exec blocks until the chosen action (and its dialog) finishes, so
        # one snapshot pair covers every edit the menu can make.
        self.parent.begin_undo_state()
        context_menu.exec(event.screenPos())
        self.parent.commit_undo_state("Edit node")

    def _batch_context_menu(self, event, nodes):
        # Applies shared attributes to every selected node. Start/End and
        # insert before/after stay single-node only.
        context_menu = QMenu()

        header = QAction(f"{len(nodes)} nodes selected")
        header.setEnabled(False)
        context_menu.addAction(header)
        context_menu.addSeparator()

        stop_action = QAction("Stop at Node", checkable=True)
        stop_action.setChecked(all(n.stop for n in nodes))
        stop_action.triggered.connect(lambda: self._batch_toggle_stop(nodes))
        context_menu.addAction(stop_action)

        reverse_action = QAction("Reverse", checkable=True)
        reverse_action.setChecked(all(n.is_reverse_node for n in nodes))
        reverse_action.triggered.connect(lambda: self._batch_toggle_reverse(nodes))
        context_menu.addAction(reverse_action)

        turn_action = QAction("Turn Value...")
        turn_action.triggered.connect(lambda: self._batch_set_turn(nodes))
        context_menu.addAction(turn_action)

        wait_action = QAction("Wait time...")
        wait_action.triggered.connect(lambda: self._batch_set_wait(nodes))
        context_menu.addAction(wait_action)

        velocity_action = QAction("Max Velocity...")
        velocity_action.triggered.connect(lambda: self._batch_set_velocity(nodes))
        context_menu.addAction(velocity_action)

        acceleration_action = QAction("Max Acceleration...")
        acceleration_action.triggered.connect(
            lambda: self._batch_set_acceleration(nodes)
        )
        context_menu.addAction(acceleration_action)

        config_actions = []
        for i, action in enumerate(self.actions):
            config_actions.append(QAction(f"{action}..."))
            config_actions[i].triggered.connect(
                lambda checked, p=i: self._batch_action_handler(nodes, p)
            )
            context_menu.addAction(config_actions[i])

        context_menu.addSeparator()
        delete_action = QAction("Delete Selected Nodes")
        delete_action.triggered.connect(self.parent.delete_selected_nodes)
        context_menu.addAction(delete_action)

        self.parent.begin_undo_state()
        context_menu.exec(event.screenPos())
        self.parent.commit_undo_state(f"Edit {len(nodes)} nodes")

    def _batch_apply(self, nodes, apply):
        for n in nodes:
            apply(n)
            n.update()
        self.parent.update_path()

    def _batch_toggle_stop(self, nodes):
        desired = not all(n.stop for n in nodes)

        def apply(n):
            # Same tri-state logic as toggle_stop, toward the shared state.
            if desired == n.auto_stop():
                n.stop_mode = STOP_AUTO
            else:
                n.stop_mode = STOP_FORCED if desired else STOP_DISABLED

        self._batch_apply(nodes, apply)
        logger.info(f"Stop at Node set to {desired} on {len(nodes)} nodes")

    def _batch_toggle_reverse(self, nodes):
        desired = not all(n.is_reverse_node for n in nodes)
        self._batch_apply(nodes, lambda n: setattr(n, "is_reverse_node", desired))
        logger.info(f"Reverse set to {desired} on {len(nodes)} nodes")

    def _batch_set_turn(self, nodes):
        value = self._ask_int(
            "Set Turn", "Enter turn (-180-180):", -180, 180, self.turn
        )
        if value is not None:
            self._batch_apply(nodes, lambda n: setattr(n, "turn", value))
            logger.info(f"Turn set to {value} on {len(nodes)} nodes")

    def _batch_set_wait(self, nodes):
        value = self._ask_double(
            "Set Wait Time", "Enter time (seconds):", self.wait_time, low=0
        )
        if value is not None:
            self._batch_apply(nodes, lambda n: setattr(n, "wait_time", value))
            logger.info(f"Wait time set to {value} on {len(nodes)} nodes")

    def _batch_set_velocity(self, nodes):
        value = self._ask_double(
            "Set Max Velocity", "Enter Velocity (0-128):", self.max_velocity, 0, 128
        )
        if value is not None:
            self._batch_apply(nodes, lambda n: setattr(n, "max_velocity", value))
            logger.info(f"Max velocity set to {value} on {len(nodes)} nodes")

    def _batch_set_acceleration(self, nodes):
        value = self._ask_double(
            "Set Max Acceleration",
            "Enter Acceleration (0-128):",
            self.max_acceleration,
            0,
            128,
        )
        if value is not None:
            self._batch_apply(nodes, lambda n: setattr(n, "max_acceleration", value))
            logger.info(f"Max acceleration set to {value} on {len(nodes)} nodes")

    def _batch_action_handler(self, nodes, action_index):
        action_name = self.actions[action_index]
        value = self._ask_double(
            f"Set {action_name}",
            f"Enter {action_name} value:",
            self.action_values[action_index],
            -1000,
            1000,
        )
        if value is not None:

            def apply(n):
                n.action_values[action_index] = value

            self._batch_apply(nodes, apply)
            logger.info(f"{action_name} set to {value} on {len(nodes)} nodes")

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
        if self.is_start_node:
            return "Start Node"
        elif self.is_end_node:
            return "End Node"
        else:
            return "Node"

    def has_action(self):
        for action_value in self.action_values:
            if action_value != 0:
                return True

        return (
            self.is_reverse_node or self.stop or self.turn != 0 or self.wait_time != 0
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

    def auto_stop(self):
        """Whether the path geometry at this node requires a full stop."""
        return self.is_reverse_node or self.turn != 0

    @property
    def stop(self):
        if self.stop_mode == STOP_FORCED:
            return True
        if self.stop_mode == STOP_DISABLED:
            return False
        return self.auto_stop()

    def is_stop_node(self):
        return self.stop

    def toggle_reverse(self):
        self.is_reverse_node = not self.is_reverse_node
        self.parent.update_path()
        logger.info(f"Reverse Node: {self.is_reverse_node}")

    def _dialog_screen_pos(self):
        # Get the position of the node in screen coordinates
        scene_pos = self.scenePos()
        view = self.scene().views()[0]
        view_pos = view.mapFromScene(scene_pos)
        return view.viewport().mapToGlobal(view_pos)

    def _make_dialog(self, title, label):
        screen_pos = self._dialog_screen_pos()
        dialog = QInputDialog(self.scene().views()[0])
        dialog.setWindowTitle(title)
        dialog.setLabelText(label)
        dialog.move(
            int(screen_pos.x() + self.radius), int(screen_pos.y() + self.radius)
        )
        return dialog

    def _ask_int(self, title, label, low, high, initial):
        """Show an int input dialog next to the node; None if cancelled."""
        dialog = self._make_dialog(title, label)
        dialog.setIntRange(low, high)
        dialog.setIntValue(initial)
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            return dialog.intValue()
        return None

    def _ask_double(self, title, label, initial, low=None, high=None):
        """Show a double input dialog next to the node; None if cancelled."""
        dialog = self._make_dialog(title, label)
        dialog.setInputMode(QInputDialog.InputMode.DoubleInput)
        if low is not None and high is not None:
            dialog.setDoubleRange(low, high)
        elif low is not None:
            dialog.setDoubleMinimum(low)
        dialog.setDoubleValue(initial)
        if dialog.exec() == QInputDialog.DialogCode.Accepted:
            return dialog.doubleValue()
        return None

    def set_turn(self):
        value = self._ask_int(
            "Set Turn", "Enter turn (-180-180):", -180, 180, self.turn
        )
        if value is not None:
            self.turn = value
            self.parent.update_path()
            logger.info(f"Turn set to: {self.turn}")

    def set_wait(self):
        value = self._ask_double(
            "Set Wait Time", "Enter time (seconds):", self.wait_time, low=0
        )
        if value is not None:
            self.wait_time = value
            logger.info(f"Wait time set to: {self.wait_time}")

    def set_velocity(self):
        value = self._ask_double(
            "Set Max Velocity", "Enter Velocity (0-128):", self.max_velocity, 0, 128
        )
        if value is not None:
            self.max_velocity = value
            logger.info(f"Turn max velocity to: {self.max_velocity}")

    def set_acceleration(self):
        value = self._ask_double(
            "Set Max Acceleration",
            "Enter Acceleration (0-128):",
            self.max_acceleration,
            0,
            128,
        )
        if value is not None:
            self.max_acceleration = value
            logger.info(f"Turn max acceleration to: {self.max_acceleration}")

    def delete_node(self):
        self.parent.remove_node(self)
        self.scene().removeItem(self)
        logger.info(f"Node at ({self.abs_x}, {self.abs_y}) deleted")

    def toggle_stop(self):
        desired = not self.stop
        # Fall back to AUTO whenever the user lands back on the geometric
        # default, so the flag keeps tracking reverse/turn if those change.
        if desired == self.auto_stop():
            self.stop_mode = STOP_AUTO
        else:
            self.stop_mode = STOP_FORCED if desired else STOP_DISABLED
        self.parent.update_path()
        logger.info(f"Stop at Node: {self.stop} (mode {self.stop_mode})")

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
        value = self._ask_double(
            f"Set {action_name}",
            f"Enter {action_name} value:",
            self.action_values[action_index],
            -1000,
            1000,
        )
        if value is not None:
            self.action_values[action_index] = value
            self.parent.update_path()
            logger.info(
                f"{action_name} value set to: {self.action_values[action_index]}"
            )

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
