import logging
import os
from pathlib import Path

import numpy as np
from PyQt6.QtCore import QEvent, Qt, QTimer
from PyQt6.QtGui import QAction, QIcon, QKeySequence, QUndoStack
from PyQt6.QtWidgets import (
    QAbstractSpinBox,
    QApplication,
    QDialog,
    QFileDialog,
    QInputDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
)

import utilities.file_management
from gui import dock_widget, node, path
from utilities import action_spec, config_manager, route_serializer, sd_card
from utilities.field_scale import FIELD_IMAGE_IN

logger = logging.getLogger(__name__)

PATH_TIME_EMPTY = "Path time: —"
# Same amber as the path's slow-stretch overlay.
PATH_TIME_WARNING_COLOR = "#e6a23c"


def format_path_time(times, positions, velocities, report, lift_waits=0):
    """Status bar text and tooltip for a generated profile.

    Profile units are feet and seconds; distance is shown in inches to match the
    rest of the GUI. Time stood still covers node waits and in-place turns.
    ``lift_waits`` counts the wait-for-lift holds, whose length is only known on
    the robot, so they are listed rather than timed.
    """
    times = np.asarray(times, dtype=float)
    velocities = np.abs(np.asarray(velocities, dtype=float))
    still = (velocities[:-1] < 1e-3) & (velocities[1:] < 1e-3)
    stationary = float(np.sum(np.diff(times)[still]))

    lines = [
        f"Path time: {report.duration:.2f} s",
        f"Distance: {positions[-1] * 12:.1f} in",
        f"Peak speed: {report.peak_speed:.2f} ft/s",
        f"Stopped (waits and turns): {stationary:.2f} s",
    ]
    text = f"Path time: {report.duration:.2f} s"
    if lift_waits:
        plural = "s" if lift_waits > 1 else ""
        text += f" + {lift_waits} lift wait{plural}"
        lines.append(
            f"Not counted: {lift_waits} wait{plural} for the lift, "
            "which last as long as the lift takes on the robot"
        )
    lines += [f"Warning: {warning}" for warning in report.warnings]
    return text, "\n".join(lines)


# Should be integrated directly with the Gui class
class SaveNodesDialog(QDialog):
    def __init__(self, nodes_string, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Nodes String")

        layout = QVBoxLayout()

        self.text_edit = QTextEdit(self)
        self.text_edit.setReadOnly(True)
        self.text_edit.setText(nodes_string)
        layout.addWidget(self.text_edit)

        copy_button = QPushButton("Copy to Clipboard", self)
        copy_button.clicked.connect(self.copy_to_clipboard)
        layout.addWidget(copy_button)

        self.setLayout(layout)

    def copy_to_clipboard(self):
        clipboard = QApplication.clipboard()
        clipboard.setText(self.text_edit.toPlainText())


class AutonomousPlannerGUIManager(QMainWindow):
    factor = 1.5

    def __init__(self, parent=None):
        super(AutonomousPlannerGUIManager, self).__init__(parent)

        self.setWindowTitle("Path Planner")  # Totally not inspired by Pronounce that
        self.setWindowIcon(
            QIcon(utilities.file_management.resource_path("../assets/flip_logo.ico"))
        )
        # Set default window size
        self.resize(700, 400)
        self.layout = QVBoxLayout()
        self.start_node = None
        self.end_node = None

        self.current_working_file = None
        self.routes_header_path = None
        self.routes_folder_path = None

        self.clearing_nodes = False

        self.undo_stack = QUndoStack(self)
        self.selected_node = None
        # At most one of selected_node / selected_action_point is set.
        self.selected_action_point = None

        # Safeguard so file can't save warning doesn't get spammed when dragging nodes w/ autosave
        self._reported_file_errors = set()
        self._reported_action_warnings = set()

        config_file_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "config.yaml"
        )
        self.config_manager: config_manager.ConfigManager = (
            config_manager.ConfigManager(config_file_path)
        )

        autonomous_path = (
            self.config_manager.get_value("files", "header_folder")  # + "/routes.h"
        )
        routes_path = self.config_manager.get_value("files", "routes_folder")

        self.routes_header_path = autonomous_path
        self.routes_folder_path = routes_path

        # The SD card the V5 brain reads from. It is attached from Windows and
        # mounted automatically (tools/sdcard); every trajectory export is
        # mirrored into its routes folder while it is present.
        self.sd_card = sd_card.SdCard(
            self.config_manager.get_value(
                "sd_card", "mount_point", sd_card.DEFAULT_MOUNT_POINT
            ),
            self.config_manager.get_value(
                "sd_card", "routes_folder", sd_card.DEFAULT_ROUTES_FOLDER
            ),
            self.config_manager.get_value("sd_card", "pending_folder", "")
            or os.path.join(
                os.path.dirname(os.path.abspath(config_file_path)),
                "route_backups",
                "sd_card_pending",
            ),
        )

        self.max_velocity = self.config_manager.get_value("motion", "max_velocity")
        self.max_acceleration = self.config_manager.get_value(
            "motion", "max_acceleration"
        )
        self.max_jerk = self.config_manager.get_value("motion", "max_jerk")

        # A drivetrain usually brakes harder than it accelerates. Falls back to
        # max_acceleration, so leaving it out of the config changes nothing.
        self.max_deceleration = (
            self.config_manager.get_value("motion", "max_deceleration")
            or self.max_acceleration
        )

        self.track_width = self.config_manager.get_value("robot", "track_width") / 12

        # Image and path widget
        self.central_widget = path.PathWidget(self.config_manager, self)

        self.setCentralWidget(self.central_widget)
        self.central_widget.show()

        # Settings Dock Widget
        self.settings_dock_widget = dock_widget.MultiPageDockWidget(
            self.config_manager, self
        )

        self.central_widget.setLayout(self.layout)
        self.setCentralWidget(self.central_widget)

        self.addDockWidget(
            Qt.DockWidgetArea.RightDockWidgetArea, self.settings_dock_widget
        )

        self.resizeDocks([self.settings_dock_widget], [300], Qt.Orientation.Horizontal)
        self.resizeDocks([self.settings_dock_widget], [400], Qt.Orientation.Vertical)

        self.update()
        self.create_menu_bar()
        self._create_status_bar()

        # Undo/redo must work no matter which panel was last clicked. Spin
        # boxes (and any text editor) claim Ctrl+Z/Ctrl+Y for their internal
        # line-edit undo via ShortcutOverride, which suppresses the menu
        # shortcuts while one has focus - so catch the keys at application
        # level, before any widget can eat them.
        QApplication.instance().installEventFilter(self)

    def eventFilter(self, watched, event):
        if (
            event.type() == QEvent.Type.KeyPress
            # Don't mess with dialog boxes (can cause issues if node that the dialog was attached to got deleted)
            and QApplication.activeModalWidget() is None
            and QApplication.activePopupWidget() is None
        ):
            if event.matches(QKeySequence.StandardKey.Undo):
                self._global_undo()
                return True
            if event.matches(QKeySequence.StandardKey.Redo) or (
                event.key() == Qt.Key.Key_Z
                and event.modifiers()
                == (
                    Qt.KeyboardModifier.ControlModifier
                    | Qt.KeyboardModifier.ShiftModifier
                )
            ):
                self._global_redo()
                return True
        return super().eventFilter(watched, event)

    def _finish_pending_editor(self):
        """Close any in-progress spin-box edit before undo/redo runs.

        clearFocus fires editingFinished, so a half-typed value is committed
        as its own undo entry first - Ctrl+Z then reverts exactly that edit
        instead of leaving it dangling and silently mis-snapshotted.
        """
        focus_widget = QApplication.focusWidget()
        if isinstance(focus_widget, QAbstractSpinBox):
            focus_widget.clearFocus()

    def _global_undo(self):
        self._finish_pending_editor()
        self.undo_stack.undo()

    def _global_redo(self):
        self._finish_pending_editor()
        self.undo_stack.redo()

    def update_coords(self, p):
        x = round(((p.x() / (2000)) - 0.5) * FIELD_IMAGE_IN, 3)
        y = round(((p.y() / (2000)) - 0.5) * FIELD_IMAGE_IN, 3)

        self.settings_dock_widget.set_current_coordinates(x, y)

    def get_tangent_at_node(self, node):
        """Get the tangent at the selected node"""
        return self.central_widget.get_tangent_at_node(node)
    
    def get_tangent_derivative_at_node(self, node):
        """Get the tangent derivative at the selected node"""
        return self.central_widget.get_tangent_derivative_at_node(node)

    def get_incoming_magnitude_at_node(self, node):
        return self.central_widget.get_incoming_magnitude_at_node(node)

    def get_outgoing_magnitude_at_node(self, node):
        return self.central_widget.get_outgoing_magnitude_at_node(node)

    def current_constraints(self):
        """The profiler constraints the status-bar path time uses."""
        return self.central_widget.build_constraints(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.track_width,
            self.max_deceleration,
        )

    def show_optimize_page(self):
        self.settings_dock_widget.show()
        self.settings_dock_widget.switch_page(dock_widget.OPTIMIZE_PAGE)

    def refresh_optimizer_overlay(self):
        # The path can rebuild before the dock exists.
        dock = getattr(self, "settings_dock_widget", None)
        if dock is not None:
            dock.optimize_page.refresh_overlay()

    def update_path(self):
        """Set the tangent at the selected node"""
        self.central_widget.update_path()

    def create_menu_bar(self):
        menu_bar = self.menuBar()

        file_menu = menu_bar.addMenu("File")
        edit_menu = menu_bar.addMenu("Edit")
        tools_menu = menu_bar.addMenu("Tools")

        save_nodes_file_action = QAction("Save Nodes to File", self)
        save_nodes_file_action.setShortcut(QKeySequence.StandardKey.Save)
        save_nodes_file_action.triggered.connect(self.save_nodes_to_file)
        file_menu.addAction(save_nodes_file_action)

        load_nodes_file_action = QAction("Load Nodes from File", self)
        load_nodes_file_action.setShortcut(QKeySequence.StandardKey.Open)
        load_nodes_file_action.triggered.connect(self.load_nodes_from_file)
        file_menu.addAction(load_nodes_file_action)

        save_nodes_string_action = QAction("Convert to String", self)
        save_nodes_string_action.setShortcut(QKeySequence("Ctrl+Shift+C"))
        save_nodes_string_action.triggered.connect(self.save_nodes_to_string)
        file_menu.addAction(save_nodes_string_action)

        load_nodes_string_action = QAction("Load Nodes from String", self)
        load_nodes_string_action.setShortcut(QKeySequence("Ctrl+Shift+V"))
        load_nodes_string_action.triggered.connect(self.load_nodes_from_string)
        file_menu.addAction(load_nodes_string_action)

        set_current_file_action = QAction("Set Current Working File", self)
        set_current_file_action.setShortcut(QKeySequence("Ctrl+T"))
        set_current_file_action.triggered.connect(self.set_working_file)
        file_menu.addAction(set_current_file_action)

        file_menu.addSeparator()
        sd_card_menu = file_menu.addMenu("SD Card")

        sync_sd_card_action = QAction("Sync Trajectories to SD Card", self)
        sync_sd_card_action.triggered.connect(self.sync_trajectories_to_sd_card)
        sd_card_menu.addAction(sync_sd_card_action)

        eject_sd_card_action = QAction("Eject SD Card", self)
        eject_sd_card_action.setShortcut(QKeySequence("Ctrl+Shift+E"))
        eject_sd_card_action.triggered.connect(self.eject_sd_card)
        sd_card_menu.addAction(eject_sd_card_action)

        connect_sd_card_action = QAction("Connect SD Card", self)
        connect_sd_card_action.triggered.connect(self.connect_sd_card)
        sd_card_menu.addAction(connect_sd_card_action)

        undo_action = self.undo_stack.createUndoAction(self, "Undo")
        undo_action.setShortcut(QKeySequence.StandardKey.Undo)
        edit_menu.addAction(undo_action)

        redo_action = self.undo_stack.createRedoAction(self, "Redo")
        redo_action.setShortcuts(
            [
                QKeySequence(QKeySequence.StandardKey.Redo),
                QKeySequence("Ctrl+Shift+Z"),
            ]
        )
        edit_menu.addAction(redo_action)

        edit_menu.addSeparator()

        delete_node_action = QAction("Delete Selected Node(s)", self)
        delete_node_action.setShortcuts(
            [
                QKeySequence(QKeySequence.StandardKey.Delete),
                QKeySequence(Qt.Key.Key_Backspace),
            ]
        )
        delete_node_action.triggered.connect(self.delete_selected_node)
        edit_menu.addAction(delete_node_action)

        clear_nodes_action = QAction("Clear All Nodes", self)
        clear_nodes_action.triggered.connect(self.clear_nodes)
        tools_menu.addAction(clear_nodes_action)

        feasibility_action = QAction("Show Speed-Limit Coloring", self, checkable=True)
        feasibility_action.setChecked(True)
        feasibility_action.triggered.connect(
            lambda checked: self.central_widget.set_feasibility_coloring(checked)
        )
        tools_menu.addAction(feasibility_action)

        drive_line_action = QAction("Show Spin-Through Drive Line", self, checkable=True)
        drive_line_action.setChecked(True)
        drive_line_action.setToolTip(
            "The dashed white line: where the robot is sent when a spin-through turn "
            "swings it off the spline"
        )
        drive_line_action.triggered.connect(
            lambda checked: self.central_widget.set_drive_overlay_visible(checked)
        )
        tools_menu.addAction(drive_line_action)
        self.drive_line_action = drive_line_action

        # Set from a node's right-click menu (Focus From Here / Focus To Here)
        clear_focus_action = QAction("Clear Focus Range", self)
        clear_focus_action.setToolTip("Show the whole path again (Esc with nothing selected)")
        clear_focus_action.triggered.connect(lambda: self.central_widget.clear_focus())
        tools_menu.addAction(clear_focus_action)

        optimize_action = QAction("Optimize Path...", self)
        optimize_action.setToolTip("Open the Optimize page: node limits, keep-out zones and runs")
        optimize_action.triggered.connect(self.show_optimize_page)
        tools_menu.addAction(optimize_action)

        show_position_graph = QAction("Show Position Graph", self)
        show_position_graph.triggered.connect(self.position_graph)
        tools_menu.addAction(show_position_graph)

        show_velocity_graph = QAction("Show Velocity Graph", self)
        show_velocity_graph.triggered.connect(self.velocity_graph)
        tools_menu.addAction(show_velocity_graph)

        show_acceleration_graph = QAction("Show Acceleration Graph", self)
        show_acceleration_graph.triggered.connect(self.acceleration_graph)
        tools_menu.addAction(show_acceleration_graph)

        show_heading_graph = QAction("Show Heading Graph", self)
        show_heading_graph.triggered.connect(self.heading_graph)
        tools_menu.addAction(show_heading_graph)

        show_angular_velocity_graph = QAction("Show Angular Velocity Graph", self)
        show_angular_velocity_graph.triggered.connect(self.angular_velocity_graph)
        tools_menu.addAction(show_angular_velocity_graph)

        show_coords_graph = QAction("Show Coords Graph", self)
        show_coords_graph.triggered.connect(self.coords_graph)
        tools_menu.addAction(show_coords_graph)

    def update_coordinate_display(self, x, y):
        self.settings_dock_widget.set_current_coordinates(x, y)

    def set_max_velocity(self, new_velocity):
        self.max_velocity = new_velocity
        self.schedule_path_time_update()

    def set_max_acceleration(self, new_acceleration):
        self.max_acceleration = new_acceleration
        self.schedule_path_time_update()

    def set_max_jerk(self, new_jerk):
        self.max_jerk = new_jerk
        self.schedule_path_time_update()

    def save_nodes_to_string(self):
        # Compact single-line form: the paste target (QInputDialog's line
        # edit) would mangle the multi-line file layout.
        nodes_string = self.convert_nodes(compact=True)
        logger.info(f"Nodes saved as string: {nodes_string}")
        dialog = SaveNodesDialog(nodes_string, self)
        dialog.exec()

    def load_nodes_from_string(self):
        nodes_string, ok = QInputDialog.getText(
            self, "Load Nodes", "Enter nodes string:"
        )
        if ok and nodes_string:
            self.central_widget.load_nodes(nodes_string)
            self._reset_edit_history()

    def _reset_edit_history(self):
        """A newly loaded route is a fresh document - drop the old history."""
        self.undo_stack.clear()
        self.set_selected_node(None)

    def mirror_nodes(self):
        self.central_widget.begin_undo_state()
        self.central_widget.mirror_nodes()
        self.central_widget.commit_undo_state("Mirror path")

    def _report_file_error(self, title, message):
        logger.error(message)
        if message not in self._reported_file_errors:
            self._reported_file_errors.add(message)
            QMessageBox.warning(self, title, message)

    def _lint_actions(self, nodes, nodes_map, action_points, actions_map):
        """Warn about action values the firmware will refuse or ignore.

        Points are checked in the order the robot runs them, so a macro that
        needs a primed lift is caught here instead of on the field.
        """
        ordered = action_spec.order_points(
            [self._exported_actions(cur_node) for cur_node in nodes],
            nodes_map,
            [self._exported_actions(cur_action) for cur_action in action_points],
            actions_map,
        )
        warnings = action_spec.lint(ordered)
        for kind, items in (("node", nodes), ("action point", action_points)):
            for i, item in enumerate(items):
                for message in action_spec.check_stop(
                    self._exported_actions(item), item.stop
                ):
                    warnings.append(f"{kind} {i + 1}: {message}")
        if not warnings:
            return
        for message in warnings:
            logger.warning(f"Action check: {message}")
        summary = "\n".join(f"- {message}" for message in warnings)
        if summary not in self._reported_action_warnings:
            self._reported_action_warnings.add(summary)
            QMessageBox.warning(
                self,
                "Action Check",
                "The route was saved, but the lift will not do what these points ask:"
                f"\n\n{summary}",
            )

    @staticmethod
    def _exported_actions(item):
        """Non-zero action values that have a firmware slot (see action_spec)."""
        return {
            name: value
            for name, value in item.get_action_map().items()
            if name in item.actions
        }

    def fill_txt_file(self, nodes_data):
        res = ""
        for data in nodes_data:
            for v in data:
                res += f"{v} "
            res += "\n"

        header_on_card = self.sd_card.is_on_card_path(self.routes_header_path)
        if header_on_card and not self.sd_card.is_mounted():
            # The card is the destination and it is out: park the trajectory
            # locally instead of failing; it moves to the card on insert.
            self._save_pending_trajectory(res)
            return

        trajectory_path = (
            self.routes_header_path + "/" + self.current_working_file + ".txt"
        )
        try:
            os.makedirs(os.path.dirname(trajectory_path), exist_ok=True)
            with open(trajectory_path, "w") as file:
                file.write(res)
            if header_on_card:
                # The brain cannot list directories; it reads this index instead.
                sd_card.write_folder_index(self.routes_header_path)
            self._reported_file_errors.clear()
        except OSError as e:
            self._report_file_error(
                "Trajectory Save Failed",
                f"Could not write the trajectory to {trajectory_path}\n\n{e}\n\n"
                "Check that the header folder (usually the SD card) is available.",
            )
            return
        if header_on_card:
            self.statusBar().showMessage(f"Saved to SD card: {trajectory_path}", 4000)
        else:
            self._mirror_trajectory_to_sd_card(res)

    # --- Path time -----------------------------------------------------------

    def schedule_path_time_update(self):
        """Grey out the path time and recompute it once edits pause."""
        if not hasattr(self, "_path_time_timer"):
            return  # the path can rebuild before the status bar exists
        self.path_time_label.setEnabled(False)
        self._path_time_timer.start()

    def _refresh_path_time(self):
        view = self.central_widget
        self.path_time_label.setEnabled(True)
        self.path_time_label.setStyleSheet("")
        if not (view.start_node and view.end_node and view.spline_manager.splines):
            self.path_time_label.setText(PATH_TIME_EMPTY)
            self.path_time_label.setToolTip(
                "Designate a start and end node to time the path"
            )
            return
        try:
            # Same arguments as save_nodes_to_file, so the export reuses this profile.
            times, positions, velocities, *_ = view.generate_motion_profile_lists(
                self.max_velocity,
                self.max_acceleration,
                self.max_jerk,
                self.track_width,
                self.max_deceleration,
            )
        except Exception:
            # An exception escaping a Qt timer kills the app without a traceback.
            logger.exception("Could not profile the path for the path time display")
            self.path_time_label.setText(PATH_TIME_EMPTY)
            self.path_time_label.setToolTip("The motion profile could not be generated")
            return
        lift_waits = sum(
            1
            for item in (*view.nodes, *view.action_points)
            if item.stop and action_spec.waits_for_lift(self._exported_actions(item))
        )
        text, tooltip = format_path_time(
            times, positions, velocities, view.feasibility_report, lift_waits
        )
        self.path_time_label.setText(text)
        self.path_time_label.setToolTip(tooltip)
        if view.feasibility_report.warnings:
            self.path_time_label.setStyleSheet(f"color: {PATH_TIME_WARNING_COLOR};")

    # --- SD card -------------------------------------------------------------

    def _create_status_bar(self):
        """Status bar with the path time and a live SD card indicator (polled every 2 s)."""
        self.path_time_label = QLabel(PATH_TIME_EMPTY)
        self.path_time_label.setToolTip("Designate a start and end node to time the path")
        self.statusBar().addPermanentWidget(self.path_time_label)
        # Profiling a route takes 30-300 ms, too slow for every drag frame, so the
        # label is recomputed once edits pause.
        self._path_time_timer = QTimer(self)
        self._path_time_timer.setSingleShot(True)
        self._path_time_timer.setInterval(300)
        self._path_time_timer.timeout.connect(self._refresh_path_time)

        self.sd_card_label = QLabel()
        self.statusBar().addPermanentWidget(self.sd_card_label)
        self._sd_card_was_mounted = None
        self._sd_card_timer = QTimer(self)
        self._sd_card_timer.setInterval(2000)
        self._sd_card_timer.timeout.connect(self._poll_sd_card)
        self._sd_card_timer.start()
        self._poll_sd_card()

    def _poll_sd_card(self):
        mounted = self.sd_card.is_mounted()
        self.sd_card_label.setText(self.sd_card.describe())
        if mounted == self._sd_card_was_mounted:
            return
        first_poll = self._sd_card_was_mounted is None
        self._sd_card_was_mounted = mounted
        if mounted:
            logger.info(f"SD card mounted at {self.sd_card.mount_point}")
            if self.config_manager.get_value("sd_card", "auto_sync", True):
                self.sync_trajectories_to_sd_card()
        elif not first_poll:
            logger.info("SD card removed")
            self.statusBar().showMessage("SD card removed", 5000)

    def _save_pending_trajectory(self, content):
        """Header folder is the card and the card is out: keep the file locally."""
        name = self.current_working_file
        try:
            path = self.sd_card.write_pending_trajectory(name, content)
            self._reported_file_errors.clear()
        except OSError as e:
            self._report_file_error(
                "Trajectory Save Failed",
                f"The SD card is not inserted and the trajectory could not be kept "
                f"locally either\n\n{e}",
            )
            return
        self.statusBar().showMessage(
            f"SD card not inserted - trajectory kept at {path}; "
            "it will be moved to the card when it is inserted",
            6000,
        )

    def _mirror_trajectory_to_sd_card(self, content):
        """Header folder is elsewhere: copy the exported trajectory to the card too."""
        if self.current_working_file is None:
            return
        name = self.current_working_file
        try:
            written = self.sd_card.write_trajectory(name, content)
        except OSError as e:
            self._report_file_error(
                "SD Card Save Failed",
                f"Could not write {name}.txt to the SD card "
                f"({self.sd_card.routes_folder})\n\n{e}",
            )
            return
        if written:
            self.statusBar().showMessage(
                f"Saved to SD card: {self.sd_card.trajectory_path(name)}", 4000
            )
        else:
            self.statusBar().showMessage(
                "SD card not inserted - trajectory saved locally; "
                "it will be copied to the card when it is inserted",
                6000,
            )

    def sync_trajectories_to_sd_card(self):
        """Bring the card up to date with what was saved while it was out.

        Pending trajectories (saved while the card was the destination but
        absent) are moved onto it. When the header folder is a local folder
        instead, every trajectory newer there than on the card is copied.
        Nothing on the card is deleted.
        """
        if not self.sd_card.is_mounted():
            self.statusBar().showMessage("SD card not inserted", 4000)
            return
        try:
            copied = self.sd_card.move_pending_to_card()
            if (
                not self.sd_card.is_on_card_path(self.routes_header_path)
                and self.routes_folder_path
            ):
                names = sd_card.route_names(self.routes_header_path)
                copied += self.sd_card.sync_trajectories(
                    self.routes_header_path, names
                )
        except OSError as e:
            self._report_file_error(
                "SD Card Sync Failed",
                f"Could not copy trajectories to the SD card\n\n{e}",
            )
            return
        if copied:
            message = (
                f"Copied {len(copied)} trajectories to SD card: "
                f"{', '.join(sorted(copied))}"
            )
        else:
            message = "SD card trajectories are up to date"
        logger.info(message)
        self.statusBar().showMessage(message, 6000)

    def eject_sd_card(self):
        """Flush and unmount the card so it is safe to pull out."""
        ok, output = self.sd_card.eject()
        if ok:
            self.statusBar().showMessage("SD card ejected - safe to remove", 6000)
        else:
            QMessageBox.warning(self, "Eject SD Card Failed", output)
        self._poll_sd_card()

    def connect_sd_card(self):
        """Take the reader back from Windows and mount the card (after eject/release)."""
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        try:
            ok, output = self.sd_card.attach()
        finally:
            QApplication.restoreOverrideCursor()
        if ok:
            self.statusBar().showMessage("SD card connected", 4000)
        else:
            QMessageBox.warning(self, "Connect SD Card Failed", output)
        self._poll_sd_card()

    def set_sd_card_folder(self, folder_path):
        self.sd_card.routes_folder = folder_path
        self._poll_sd_card()

    # Called by the Settings page when a folder is changed there, so the
    # running app follows the config instead of waiting for a restart.
    def set_routes_folder(self, folder_path):
        self.routes_folder_path = folder_path
        logger.info(f"Routes folder set to {folder_path}")
        self.settings_dock_widget.routes_page.refresh_files()

    def set_header_folder(self, folder_path):
        self.routes_header_path = folder_path
        logger.info(f"Header folder set to {folder_path}")
        self._poll_sd_card()

    def save_nodes_to_file(self):
        nodes_string = ""
        nodes = self.central_widget.get_nodes()
        if len(nodes) > 0:
            nodes_string = self.convert_nodes()
        full_path = None

        if self.current_working_file is None:
            file_name, ok = QInputDialog.getText(
                self, "Save Route to File", "Enter file name (without extension):"
            )
            if not ok or not file_name:
                return

            file_name = file_name.strip() + ".txt"
            folder = QFileDialog.getExistingDirectory(
                self,
                "Select Directory to Save File",
                str(Path(os.getcwd()).parent.absolute()),
            )
            if not folder:
                return

            full_path = f"{folder}/{file_name}"
        else:
            full_path = (
                self.routes_folder_path + "/" + self.current_working_file + ".txt"
            )
        nodes_data = []
        nodes_map = []
        actions_map = []
        if (
            self.central_widget.start_node
            and self.central_widget.end_node
        ):
            (
                time_intervals,
                positions,
                velocities,
                accelerations,
                headings,
                angular_velocities,
                nodes_map,
                actions_map,
                coords,
            ) = self.central_widget.generate_motion_profile_lists(
                self.max_velocity,
                self.max_acceleration,
                self.max_jerk,
                self.track_width,
                self.max_deceleration,
            )
            for i in range(0, len(time_intervals), 1):  # Every 10ms save data
                nodes_data.append(
                    [
                        0,
                        time_intervals[i],
                        coords[i][0] * 12,
                        coords[i][1] * -12,
                        headings[i],
                        velocities[i] * 12,
                        angular_velocities[i],
                    ]
                )
        logger.info(f"nodes map: {nodes_map} traj length {len(nodes_data)}")
        nodes_actions = [[1] + cur_node.get_action_values() for cur_node in nodes]

        actions_data = [
            [1] + cur_action.get_action_values()
            for cur_action in self.central_widget.action_points
        ]

        logger.info(
            f"Nodes map length {len(nodes_map)}, Node actions length {len(nodes_actions)}"
        )
        for i in range(0, len(nodes_map)):
            nodes_data.insert(int(nodes_map[i] / 1) + i, nodes_actions[i])
        for i in range(0, len(actions_map)):
            nodes_data.insert(int(actions_map[i] / 1) + i, actions_data[i])

        self._lint_actions(
            nodes, nodes_map, self.central_widget.action_points, actions_map
        )

        # self.fill_template(nodes_data)
        self.fill_txt_file(nodes_data)
        try:
            os.makedirs(os.path.dirname(full_path), exist_ok=True)
            with open(full_path, "w") as file:
                file.write(nodes_string)
        except OSError as e:
            self._report_file_error(
                "Route Save Failed", f"Could not write the route to {full_path}\n\n{e}"
            )
            return
        logger.info(f"Route saved to {full_path}")

    def load_nodes_from_file(self, dialogue=True):
        file_name = None
        if dialogue:
            file_name, _ = QFileDialog.getOpenFileName(
                self, "Load Route from File", "", "Text Files (*.txt);;All Files (*)"
            )
        else:
            file_name = (
                self.routes_folder_path + "/" + self.current_working_file + ".txt"
            )
        if file_name:
            try:
                with open(file_name, "r") as file:
                    nodes_string = file.read()
            except OSError as e:
                self._report_file_error(
                    "Route Load Failed",
                    f"Could not read the route from {file_name}\n\n{e}",
                )
                return
            logger.info(f"Route loaded from {file_name}")
            self.central_widget.load_nodes(nodes_string)
            self._reset_edit_history()

    def load_route_file(self, file_path):
        """Load a route file"""

        if not file_path.endswith(".txt") or not os.path.exists(
            os.path.join(self.routes_folder_path, file_path)
        ):
            logger.error(f"Invalid file path: {file_path}")
            return
        self.current_working_file = file_path[:-4]  # cut off the .txt
        file_path = os.path.join(self.routes_folder_path, file_path)
        try:
            with open(file_path, "r") as file:
                nodes_string = file.read()
        except OSError as e:
            self._report_file_error(
                "Route Load Failed", f"Could not read the route from {file_path}\n\n{e}"
            )
            return

        logger.info(f"Route loaded from {file_path}")
        self.central_widget.load_nodes(nodes_string)
        self._reset_edit_history()

    def set_working_file(self):
        file_name, ok = QInputDialog.getText(
            self, "File to save route to", "Enter file name (without extension):"
        )
        if ok and file_name:
            self.current_working_file = file_name
            folder = self.routes_folder_path
            if folder is None:
                folder = QFileDialog.getExistingDirectory(
                    self,
                    "Select Directory to Save File",
                    str(Path(os.getcwd()).parent.absolute()),
                )

            if folder:
                self.routes_folder_path = folder
                # A name typed as "suite/new_route" lands in that subfolder.
                full_path = f"{folder}/{self.current_working_file}.txt"
                os.makedirs(os.path.dirname(full_path), exist_ok=True)
                if not os.path.exists(full_path):
                    with open(
                        full_path, "w+"
                    ):  # Creates file if it isn't already created
                        pass
                logger.info(f"Set current working file at {full_path}")
                if (
                    len(self.central_widget.get_nodes()) > 0
                    or os.stat(full_path).st_size == 0
                ):
                    self.auto_save()
                else:
                    self.load_nodes_from_file(False)

    def get_nodes(self):
        return self.central_widget.get_nodes()

    def clear_nodes(self):
        logger.info("Clearing all nodes...")
        self.central_widget.begin_undo_state()
        self.central_widget.clear_nodes()
        self.central_widget.commit_undo_state("Clear all nodes")
        self.set_selected_node(None)

    def convert_nodes(self, compact=False):
        """Serialize the current path as route text (the vap-route format).

        compact=True yields a single line for clipboard sharing; the default
        multi-line layout is what route files and undo snapshots use.
        """
        nodes: list[node.Node] = self.central_widget.get_nodes()
        logger.info("Converting nodes...")
        logger.debug(f"Nodes: {nodes}")
        route = route_serializer.Route(
            nodes=[
                route_serializer.NodeRecord(
                    x=cur_node.get_abs_x(),
                    y=cur_node.get_abs_y(),
                    is_start=cur_node.is_start_node,
                    is_end=cur_node.is_end_node,
                    is_reverse=cur_node.is_reverse_node,
                    stop_mode=int(cur_node.stop_mode),
                    turn=cur_node.turn,
                    spin_through=cur_node.spin_through,
                    wait_time=cur_node.wait_time,
                    max_velocity=cur_node.max_velocity,
                    max_acceleration=cur_node.max_acceleration,
                    tangent=cur_node.get_tangent(True),
                    tangent_derivative=cur_node.get_tangent_derivative(True),
                    incoming_magnitude=cur_node.get_incoming_magnitude(),
                    outgoing_magnitude=cur_node.get_outgoing_magnitude(),
                    locked=cur_node.is_locked(),
                    visible=cur_node.is_visible(),
                    actions=cur_node.get_action_map(),
                    optimize=dict(cur_node.optimize),
                )
                for cur_node in nodes
            ],
            action_points=[
                route_serializer.ActionPointRecord(
                    x=cur_action.get_abs_x(),
                    y=cur_action.get_abs_y(),
                    t=cur_action.t,
                    stop=bool(cur_action.stop),
                    wait_time=cur_action.wait_time,
                    max_velocity=cur_action.max_velocity,
                    max_acceleration=cur_action.max_acceleration,
                    actions=cur_action.get_action_map(),
                )
                for cur_action in self.central_widget.action_points
            ],
            keep_out=[
                route_serializer.KeepOutRecord(k.shape, list(k.values))
                for k in self.central_widget.keep_out
            ],
        )
        action_names = self.config_manager.get_value("robot", "actions") or []
        return route_serializer.dumps_route(route, action_names, compact=compact)

    def auto_save(self):
        if (
            self.current_working_file is not None
            and self.central_widget.start_node
            and self.central_widget.end_node
            and not self.clearing_nodes
        ):
            logger.info("Auto Saving...")
            if len(self.central_widget.get_nodes()) > 0:
                self.save_nodes_to_file()
            else:
                self.load_nodes_from_file(False)

    def fill_template(self, nodes_data):
        # Create the string to insert
        stringified = []
        for i in range(0, len(nodes_data)):
            if len(nodes_data[i]) > 2:
                stringified.append("{")
                for j in range(0, len(nodes_data[i])):
                    stringified[-1] += f"{nodes_data[i][j]}"
                    if j < len(nodes_data[i]) - 1:
                        stringified[-1] += ", "
                stringified[-1] += "}"
            else:
                stringified.append(f"{{{nodes_data[i][0]}, {nodes_data[i][1]}}}")
        insertion = f"std::vector<std::vector<double>> {self.current_working_file} = {{{', '.join(stringified)}}};\n"

        try:
            # Read the content of routes.h
            with open(self.routes_header_path, "r") as routes_file:
                content = routes_file.readlines()

            # Find the line with the specified route name
            inserted = False
            for i, line in enumerate(content):
                if line.strip().startswith(
                    f"std::vector<std::vector<double>> {self.current_working_file} ="
                ):
                    content[i] = insertion
                    inserted = True
                    break
            if not inserted:
                # Insert before the #endif line
                for i, line in enumerate(content):
                    if line.strip() == "#endif":
                        content.insert(i, insertion)
                        break

        except FileNotFoundError:
            # If routes.h does not exist, create a new one with the necessary structure
            content = [
                "#ifndef ROUTES_H\n",
                "#define ROUTES_H\n",
                "#include <vector>\n",
                "\n",
                insertion,
                "\n",
                "#endif\n",
            ]

        if (
            not os.path.exists(self.routes_header_path)
            or os.stat(self.routes_header_path).st_size == 0
        ):
            # If the file is empty, write the content
            content = [
                "#ifndef ROUTES_H\n",
                "#define ROUTES_H\n",
                "#include <vector>\n",
                "\n",
                insertion,
                "\n",
                "#endif\n",
            ]

        logger.info(f"Saving motion profile to {self.routes_header_path}")
        with open(self.routes_header_path, "w") as routes_file:
            routes_file.writelines(content)

    def set_selected_node(self, node: node.Node):
        """Set the currently selected node in the central widget"""
        # Any node selection change (including a clear) drops the action point.
        self._set_selected_action_point(None)
        previous = self.selected_node
        self.selected_node = node
        self.settings_dock_widget.set_selected_node(node)
        # Repaint both so the selection ring follows the panel selection.
        if previous is not node:
            for n in (previous, node):
                if n is None:
                    continue
                try:
                    n.update()
                except RuntimeError:
                    pass  # the previous node may already be deleted

    def set_selected_action_point(self, point):
        """Select an action point for the Path panel; clears any node selection."""
        if point is not None:
            self.set_selected_node(None)
        self._set_selected_action_point(point)

    def _set_selected_action_point(self, point):
        previous = self.selected_action_point
        self.selected_action_point = point
        self.settings_dock_widget.set_selected_action_point(point)
        if previous is not point:
            for p in (previous, point):
                if p is None:
                    continue
                try:
                    p.update()
                except RuntimeError:
                    pass  # the previous point may already be deleted

    def refresh_selected_action_point(self):
        """Re-read the selected point's values into the panel (after a drag or
        a path rebuild moved it)."""
        self.settings_dock_widget.refresh_selected_action_point()

    def delete_selected_node(self):
        point = self.selected_action_point
        if point is not None and point in self.central_widget.action_points:
            self.central_widget.begin_undo_state()
            point.delete_action_point()
            self.central_widget.commit_undo_state("Delete action point")
            return

        # Iterate a copy: remove_node discards from the live selection set.
        multi = [
            n
            for n in self.central_widget.selected_nodes
            if n in self.central_widget.get_nodes()
        ]
        if len(multi) > 1:
            self.central_widget.begin_undo_state()
            for n in multi:
                n.delete_node()
            self.central_widget.commit_undo_state(f"Delete {len(multi)} nodes")
            self.central_widget.clear_multi_selection()
            self.set_selected_node(None)
            return

        node_to_delete = self.selected_node
        if (
            node_to_delete is None
            or node_to_delete not in self.central_widget.get_nodes()
        ):
            return
        self.central_widget.begin_undo_state()
        node_to_delete.delete_node()
        self.central_widget.commit_undo_state("Delete node")
        self.set_selected_node(None)

    def switch_field(self, fieldType: str):
        print(fieldType)
        if fieldType == "Push Back Match":
            self.central_widget.update_image_path(
                utilities.file_management.resource_path(
                    "../assets/V5RC-Override-Match-2000x2000.png"
                )
            )
        else:
            self.central_widget.update_image_path(
                utilities.file_management.resource_path(
                    "../assets/V5RC-PushBack-Skills-2000x2000.png"
                )
            )

    def set_velocity(self, new_velocity):
        self.max_velocity = new_velocity
        utilities.file_management.set_config_value("max_velocity", self.max_velocity)
        self.schedule_path_time_update()

    def update_path_page(self):
        """Update the current page view in the dock widget"""
        self.settings_dock_widget.update_path_page()
        logger.info("Path page updated")

    def set_acceleration(self, new_acceleration):
        self.max_acceleration = new_acceleration
        utilities.file_management.set_config_value(
            "max_acceleration", self.max_acceleration
        )
        self.schedule_path_time_update()

    def set_jerk(self, new_jerk):
        self.max_jerk = new_jerk
        utilities.file_management.set_config_value("max_jerk", self.max_jerk)
        self.schedule_path_time_update()

    def set_track_width(self, new_track_width):
        self.track_width = new_track_width
        self.schedule_path_time_update()

    def toggle_robot_visualization(self, state):
        self.central_widget.toggle_visualization(state)

    def position_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.track_width,
            self.max_deceleration,
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.positions,
            12,
            8,
            "Position Profile",
            "Time (s)",
            "Position (ft)",
        )

    def velocity_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.track_width,
            self.max_deceleration,
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.velocities,
            12,
            8,
            "Velocity Profile",
            "Time (s)",
            "Velocity (ft/s)",
        )

    def acceleration_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.track_width,
            self.max_deceleration,
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.accelerations,
            12,
            8,
            "Acceleration Profile",
            "Time (s)",
            "Acceleration (ft/s²)",
        )

    def heading_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.track_width,
            self.max_deceleration,
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.headings,
            12,
            8,
            "Heading Profile",
            "Time (s)",
            "Heading (degrees)",
        )

    def angular_velocity_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.track_width,
            self.max_deceleration,
        )

        utilities.file_management.create_mpl_plot(
            self.central_widget.time_intervals,
            self.central_widget.angular_velocities,
            12,
            8,
            "Angular Velocity Profile",
            "Time (s)",
            "Angular Velocity (radians/s)",
        )

    def coords_graph(self):
        self.central_widget.generate_motion_profile_lists(
            self.max_velocity,
            self.max_acceleration,
            self.max_jerk,
            self.track_width,
            self.max_deceleration,
        )

        x, y = zip(*self.central_widget.coords)

        utilities.file_management.create_mpl_plot(
            x,
            y,
            8,
            8,
            "Coordinate Profile",
            "X",
            "Y",
            ((-6), (6)),
            ((-6), (6)),
            1,
        )
