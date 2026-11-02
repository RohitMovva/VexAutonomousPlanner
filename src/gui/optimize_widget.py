"""The Optimize page: per-node optimizer limits, keep-out zones, runs and the result.

- **Node limits** edit the selected node's (or the multi-selection's) ``optimize``
  field: for heading, handles, curvature and position, Default / Free / Locked /
  Limit (an amount), and the node's corridor. Every edit is an undo step.
- **Keep-out zones** edit the route's ``keep_out`` list: circles and rectangles in
  field inches the robot's footprint may get no deeper into. Every edit is an undo
  step.
- **Run** optimizes the route as it is now, in a child process
  (``utilities.optimizer_worker``), with the run-wide defaults set here.
- **Result** is the last run only: previewed on the canvas (dashed, changed nodes
  ringed). Apply replaces the path with it as a single undo step; Save copy writes it
  to a new route file. Once the route changes (other than by Apply, or an undo back
  to where the run started) the result is dropped; run again.

While the page is showing, the canvas draws each node's limits and the keep-out zones
(see ``gui.optimizer_overlay``).
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from typing import Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFontDatabase
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from gui.path import PathStateCommand
from utilities import optimizer_worker
from utilities import path_optimizer as po
from utilities import route_serializer as rs

logger = logging.getLogger(__name__)

DEFAULT, FREE, LOCKED, LIMIT = "Default", "Free", "Locked", "Limit"
SETTING_MODES = (DEFAULT, FREE, LOCKED, LIMIT)
# A yes / no setting: no amount to limit it to.
SWITCH_MODES = (DEFAULT, FREE, LOCKED)
CORRIDOR_MODES = (DEFAULT, LIMIT)
UNDO_TEXT = "Optimizer limits"


@dataclass
class RunResult:
    source_text: str  # the route as it was when the run started
    source_route: rs.Route
    options: po.OptimizerOptions
    result: po.OptimizationResult
    # The route as Apply left it; the result reads as applied while the route still is
    # exactly that (an undo makes it applicable again).
    applied_text: Optional[str] = None


def _spin(lo, hi, decimals, step, suffix="", prefix=""):
    spin = QDoubleSpinBox()
    spin.setRange(lo, hi)
    spin.setDecimals(decimals)
    spin.setSingleStep(step)
    spin.setSuffix(suffix)
    spin.setPrefix(prefix)
    spin.setKeyboardTracking(False)
    return spin


def node_record(n) -> rs.NodeRecord:
    """The fields of a canvas node that node_limits reads."""
    return rs.NodeRecord(
        is_start=n.is_start_node,
        is_end=n.is_end_node,
        is_reverse=n.is_reverse_node,
        stop_mode=int(n.stop_mode),
        turn=n.turn,
        spin_through=n.spin_through,
        wait_time=n.wait_time,
        actions=n.get_action_map(),
        optimize=dict(n.optimize),
    )


def node_role(rec: rs.NodeRecord) -> str:
    return "start" if rec.is_start else "end" if rec.is_end else ""


class OptimizeWidget(QWidget):
    def __init__(self, gui_manager, parent=None):
        super().__init__(parent)
        self.gui = gui_manager
        self.active = False
        self._loading = False
        self._job: Optional[optimizer_worker.OptimizerJob] = None
        self._job_text = ""
        self._job_options: Optional[po.OptimizerOptions] = None
        self._job_started = 0.0
        self._stop_requested = False
        self.result: Optional[RunResult] = None

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(10)
        layout.addWidget(self._build_node_group())
        layout.addWidget(self._build_keep_out_group())
        layout.addWidget(self._build_run_group())
        layout.addWidget(self._build_result_group(), 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setWidget(content)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(150)
        self._poll_timer.timeout.connect(self._poll)

        self.gui.undo_stack.indexChanged.connect(self._route_changed)
        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.shutdown)
        self.refresh_node_editor()
        self.refresh_keep_out_list()
        self._refresh_result()

    # --- layout -------------------------------------------------------------------

    def _build_node_group(self) -> QGroupBox:
        group = QGroupBox("Node limits")
        form = QFormLayout(group)
        self.node_title = QLabel()
        self.node_title.setWordWrap(True)
        form.addRow(self.node_title)

        # key -> (combo, spin); the spin holds the amount for Limit.
        self.rows = {}
        specs = (
            (po.HEADING, "Heading", SETTING_MODES, _spin(0.5, 90, 1, 1, " °", "±"),
             "How far the node's heading may turn"),
            (po.HANDLES, "Handles", SETTING_MODES, _spin(1.05, 5, 2, 0.1, "", "×"),
             "How far each handle may scale, either way (×2 = half to double)"),
            (po.CURVATURE, "Curvature", SETTING_MODES, _spin(0.05, 20, 2, 0.1, " 1/ft", "±"),
             "How far the curvature at the node may change"),
            (po.POSITION, "Position", SETTING_MODES, _spin(0.5, 24, 1, 0.5, " in"),
             "How far the node may move from where it is (a radius)"),
            (po.CORRIDOR, "Corridor", CORRIDOR_MODES, _spin(0.0, 12, 2, 0.25, " in"),
             "How far the path near this node may move from where it is now"),
            (po.SPIN_THROUGH, "Spin-through", SWITCH_MODES, _spin(0, 1, 0, 1, ""),
             "Whether the run may make this turn-around a spin-through turn"),
        )
        for key, label, modes, spin, tip in specs:
            combo = QComboBox()
            combo.addItems(modes)
            if key == po.CORRIDOR:
                combo.setToolTip("Default: the run's corridor\nLimit: this node's own corridor")
            elif key == po.SPIN_THROUGH:
                combo.setToolTip(
                    "Default / Free: the run may spin through a reverse or turn node with no"
                    " action, wait or locked heading\nLocked: it stays a stop"
                )
                spin.hide()
            else:
                combo.setToolTip(
                    "Default: the run's setting (heading is locked at start, end and action"
                    " nodes, curvature at the start and wherever the robot stops, position"
                    " everywhere)\nFree: the run's range\nLocked: never changed\nLimit: at"
                    " most the amount given"
                )
            spin.setToolTip(tip)
            combo.currentTextChanged.connect(lambda _text, k=key: self._on_mode_changed(k))
            spin.valueChanged.connect(lambda _value, k=key: self._on_amount_changed(k))
            spin.editingFinished.connect(self._commit_amount)
            row = QHBoxLayout()
            row.addWidget(combo)
            row.addWidget(spin, 1)
            form.addRow(f"{label}:", row)
            self.rows[key] = (combo, spin)

        self.node_effective = QLabel()
        self.node_effective.setWordWrap(True)
        self.node_effective.setStyleSheet("color: gray;")
        form.addRow(self.node_effective)

        buttons = QHBoxLayout()
        self.reset_node_button = QPushButton("Reset to defaults")
        self.reset_node_button.clicked.connect(self._reset_node_limits)
        self.lock_node_button = QPushButton("Lock all")
        self.lock_node_button.setToolTip("Lock heading, handles, curvature, position and spin-through")
        self.lock_node_button.clicked.connect(self._lock_node)
        buttons.addWidget(self.reset_node_button)
        buttons.addWidget(self.lock_node_button)
        form.addRow(buttons)
        return group

    def _build_run_group(self) -> QGroupBox:
        group = QGroupBox("Run")
        form = QFormLayout(group)
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        d = po.OptimizerOptions()

        self.setting_checks = {}
        checks = QHBoxLayout()
        for key, label in (
            (po.HEADING, "Heading"), (po.HANDLES, "Handles"), (po.CURVATURE, "Curvature"), (po.POSITION, "Position"),
            (po.SPIN_THROUGH, "Spin-through"),
        ):
            box = QCheckBox(label)
            box.setChecked(key in d.settings)
            box.toggled.connect(self._run_options_changed)
            checks.addWidget(box)
            self.setting_checks[key] = box
        form.addRow(QLabel("Settings the run may change:"))
        form.addRow(checks)

        self.corridor_spin = _spin(0.25, 12, 2, 0.25, " in")
        self.corridor_spin.setValue(d.corridor_in)
        self.corridor_spin.setToolTip("Default corridor: how far the path may move from where it is now")
        self.heading_spin = _spin(1, 90, 1, 1, " °", "±")
        self.heading_spin.setValue(d.heading_range_deg)
        self.heading_spin.setToolTip("How far a free heading may turn")
        self.handle_spin = _spin(1.05, 5, 2, 0.1, "", "×")
        self.handle_spin.setValue(d.handle_scale[1])
        self.handle_spin.setToolTip("How far free handles may scale, either way")
        self.ap_heading_spin = _spin(0, 30, 1, 0.5, " °", "±")
        self.ap_heading_spin.setValue(d.action_point_heading_deg)
        self.ap_heading_spin.setToolTip(
            "Largest heading change allowed at an action point with a distance reset,"
            " lift_state or wait_lift"
        )
        self.position_spin = _spin(0.5, 24, 1, 0.5, " in")
        self.position_spin.setValue(d.position_radius_in)
        self.position_spin.setToolTip("How far a node whose position is Free may move")
        self.seeds_spin = _spin(0, 20, 0, 1)
        self.seeds_spin.setValue(d.seeds)
        self.seeds_spin.setToolTip(
            "When a node may move: how many other starting layouts to search from"
            " (each adds about as long again)"
        )
        self.rate_spin = _spin(0, 100, 0, 5, " %")
        self.rate_spin.setValue(round((d.curvature_rate_margin - 1) * 100))
        self.rate_spin.setToolTip(
            "How much each stretch's curvature rate (99th-percentile |dk/ds|) may grow."
            " RAMSETE follows fast curvature changes poorly."
        )
        for label, spin in (
            ("Corridor:", self.corridor_spin),
            ("Heading range:", self.heading_spin),
            ("Handle range:", self.handle_spin),
            ("Action points:", self.ap_heading_spin),
            ("Curvature rate:", self.rate_spin),
            ("Position radius:", self.position_spin),
            ("Extra starts:", self.seeds_spin),
        ):
            spin.valueChanged.connect(self._run_options_changed)
            form.addRow(label, spin)

        self.show_limits_check = QCheckBox("Show limits on the field")
        self.show_limits_check.setChecked(True)
        self.show_limits_check.setToolTip(
            "Band: corridor. Wedge: how far the heading may turn."
            " Ring: green all free, amber some, red locked."
            " Dashed circle: how far a node may move."
        )
        self.show_limits_check.toggled.connect(self.refresh_overlay)
        form.addRow(self.show_limits_check)

        buttons = QHBoxLayout()
        self.run_button = QPushButton("Optimize")
        self.run_button.setToolTip("Optimize the route as it is now")
        self.run_button.clicked.connect(self.start_run)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setToolTip("Finish now and keep the best route found so far")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_run)
        buttons.addWidget(self.run_button)
        buttons.addWidget(self.stop_button)
        form.addRow(buttons)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.hide()
        form.addRow(self.progress_bar)
        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        form.addRow(self.status_label)
        return group

    def _build_keep_out_group(self) -> QGroupBox:
        group = QGroupBox("Keep-out zones")
        group.setToolTip(
            "Areas the robot's footprint may get no deeper into than the route already"
            " does. Field inches. Saved with the route."
        )
        layout = QVBoxLayout(group)
        self.keep_out_list = QListWidget()
        self.keep_out_list.setMaximumHeight(90)
        self.keep_out_list.currentRowChanged.connect(self._keep_out_selected)
        layout.addWidget(self.keep_out_list)

        buttons = QHBoxLayout()
        self.add_circle_button = QPushButton("Add circle")
        self.add_circle_button.setToolTip("A 6 in circle at the selected node (or the field centre)")
        self.add_circle_button.clicked.connect(lambda: self._add_keep_out("circle"))
        self.add_rect_button = QPushButton("Add rectangle")
        self.add_rect_button.setToolTip("A 12 in square at the selected node (or the field centre)")
        self.add_rect_button.clicked.connect(lambda: self._add_keep_out("rect"))
        self.remove_keep_out_button = QPushButton("Remove")
        self.remove_keep_out_button.clicked.connect(self._remove_keep_out)
        for b in (self.add_circle_button, self.add_rect_button, self.remove_keep_out_button):
            buttons.addWidget(b)
        layout.addLayout(buttons)

        # One spin per value of the selected zone: x, y, r for a circle, the two
        # corners for a rectangle.
        form = QFormLayout()
        self.keep_out_spins = []
        self.keep_out_labels = []
        for _ in range(4):
            spin = _spin(-80, 80, 1, 0.5, " in")
            spin.valueChanged.connect(self._on_keep_out_value)
            spin.editingFinished.connect(self._commit_keep_out)
            label = QLabel()
            form.addRow(label, spin)
            self.keep_out_spins.append(spin)
            self.keep_out_labels.append(label)
        layout.addLayout(form)
        return group

    def _build_result_group(self) -> QGroupBox:
        group = QGroupBox("Result")
        layout = QVBoxLayout(group)
        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self.details.setMinimumHeight(180)
        self.details.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        layout.addWidget(self.details, 1)

        buttons = QHBoxLayout()
        self.apply_button = QPushButton("Apply")
        self.apply_button.setToolTip("Replace the path with the result (undoable)")
        self.apply_button.clicked.connect(self.apply_result)
        self.save_copy_button = QPushButton("Save copy…")
        self.save_copy_button.setToolTip("Write the result to a new route file")
        self.save_copy_button.clicked.connect(self.save_result_copy)
        self.discard_button = QPushButton("Discard")
        self.discard_button.setToolTip("Drop the result")
        self.discard_button.clicked.connect(self.discard_result)
        for b in (self.apply_button, self.save_copy_button, self.discard_button):
            buttons.addWidget(b)
        layout.addLayout(buttons)
        return group

    # --- page state ---------------------------------------------------------------

    def set_active(self, active: bool) -> None:
        """The dock switched to (or away from) this page."""
        self.active = active
        if active:
            self.refresh_node_editor()
            self.refresh_keep_out_list()
            self.refresh_overlay()
            self._show_result_overlay()
        else:
            self._overlay().clear()

    def _overlay(self):
        return self.gui.central_widget.optimizer_overlay

    def _action_names(self):
        return self.gui.config_manager.get_value("robot", "actions") or []

    def run_options(self) -> po.OptimizerOptions:
        cfg = self.gui.config_manager
        width, length = cfg.get_value("robot", "width") or 16.0, cfg.get_value("robot", "length") or 15.0
        robot_size = max(width, length)
        scale = self.handle_spin.value()
        return po.OptimizerOptions(
            corridor_in=self.corridor_spin.value(),
            heading_range_deg=self.heading_spin.value(),
            handle_scale=(1.0 / scale, scale),
            curvature_rate_margin=1.0 + self.rate_spin.value() / 100.0,
            action_point_heading_deg=self.ap_heading_spin.value(),
            settings=tuple(k for k, box in self.setting_checks.items() if box.isChecked()),
            robot_size_in=robot_size,
            footprint_in=(length, width),
            position_radius_in=self.position_spin.value(),
            seeds=int(self.seeds_spin.value()),
        )

    def _run_options_changed(self, *_):
        self.refresh_node_editor()
        self.refresh_overlay()

    def refresh_overlay(self, *_):
        if not self.active:
            return
        overlay = self._overlay()
        overlay.show_keep_out(self._zones(), self._keep_out_row())
        if not self.show_limits_check.isChecked():
            overlay.clear_limits()
            return
        try:
            route = rs.parse_route(self.gui.convert_nodes(), self._action_names())
        except Exception:
            overlay.clear_limits()
            return
        overlay.show_limits(route, self.run_options())

    # --- node limits --------------------------------------------------------------

    def targets(self):
        """The nodes a limit edit applies to: the multi-selection when the panel's node
        is part of it, else the panel's node."""
        view = self.gui.central_widget
        selected = self.gui.selected_node
        if selected is None or selected not in view.nodes:
            return []
        multi = [n for n in view.nodes if n in view.selected_nodes]
        return multi if selected in multi and len(multi) > 1 else [selected]

    def set_selected_node(self, _node) -> None:
        self.refresh_node_editor()

    def refresh_node_editor(self) -> None:
        targets = self.targets()
        enabled = bool(targets)
        for combo, spin in self.rows.values():
            combo.setEnabled(enabled)
        self.reset_node_button.setEnabled(enabled)
        self.lock_node_button.setEnabled(enabled)
        if not targets:
            self.node_title.setText("Select a node on the field to set how far the optimizer may change it.")
            self.node_effective.setText("")
            self._loading = True
            try:
                for key, (combo, spin) in self.rows.items():
                    combo.setCurrentIndex(0)
                    self._show_amount(key, None)
            finally:
                self._loading = False
            return

        view = self.gui.central_widget
        first = targets[0]
        index = view.nodes.index(first)
        rec = node_record(first)
        if len(targets) > 1:
            self.node_title.setText(
                f"{len(targets)} nodes selected. Showing node {index + 1}; edits apply to all."
            )
        else:
            role = node_role(rec)
            self.node_title.setText(f"Node {index + 1} of {len(view.nodes)}" + (f" ({role})" if role else ""))

        options = self.run_options()
        self._loading = True
        try:
            for key, (combo, spin) in self.rows.items():
                value = first.optimize.get(key)
                if value is None:
                    mode = DEFAULT
                elif value is True:
                    mode = FREE
                elif value is False:
                    mode = LOCKED
                elif key != po.CORRIDOR and float(value) < spin.minimum():
                    mode = LOCKED  # 0 degrees, x1 handles, 0 1/ft: locked in effect
                else:
                    mode = LIMIT
                combo.setCurrentText(mode if combo.findText(mode) >= 0 else LIMIT)
                self._show_amount(key, float(value) if mode == LIMIT else None)
        finally:
            self._loading = False
        self.node_effective.setText(self._effective_text(rec, options))

    def _suggested_amount(self, key) -> float:
        """What Limit starts at: the run's own range, so switching changes nothing yet
        (curvature has no run range; 1/ft is a modest start)."""
        return {
            po.HEADING: self.heading_spin.value(),
            po.HANDLES: self.handle_spin.value(),
            po.CURVATURE: 1.0,
            po.POSITION: self.position_spin.value(),
            po.CORRIDOR: self.corridor_spin.value(),
        }[key]

    def _show_amount(self, key, value: Optional[float]) -> None:
        """Show an amount in the row's spin box, or blank it (None) outside Limit mode:
        the effective line below says what Default / Free / Locked amount to."""
        _, spin = self.rows[key]
        if value is None:
            spin.setSpecialValueText("—")
            spin.setValue(spin.minimum())
            spin.setEnabled(False)
        else:
            spin.setSpecialValueText("")
            spin.setValue(value)
            spin.setEnabled(True)

    @staticmethod
    def _effective_text(rec: rs.NodeRecord, options: po.OptimizerOptions) -> str:
        lim = po.node_limits(rec, options)

        def off(key):
            return "" if key in options.settings else " (off for this run)"

        if lim.heading_deg > 0:
            heading = f"±{lim.heading_deg:g}°"
        elif po.HEADING not in rec.optimize and po.heading_locked_by_default(rec):
            heading = "locked (start, end and action nodes by default)"
        else:
            heading = "locked"
        handles = (
            f"×{lim.handle_scale[0]:.2f}–{lim.handle_scale[1]:g}" if lim.handle_scale else "locked"
        )
        if lim.curvature <= 0 and po.CURVATURE not in rec.optimize and po.curvature_locked_by_default(rec):
            curvature = "locked (start and stops by default)"
        elif lim.curvature <= 0:
            curvature = "locked"
        elif lim.curvature == float("inf"):
            curvature = "free"
        else:
            curvature = f"±{lim.curvature:g} 1/ft"
        position = f"within {lim.position_in:g} in" if lim.position_in > 0 else "locked"
        corridor = (
            "no corridor where it moves" if lim.position_in > 0 and po.POSITION in options.settings
            else f"corridor {lim.corridor_in:g} in"
        )
        if lim.spin_through:
            spin = f"may spin through{off(po.SPIN_THROUGH)}"
        elif rec.spin_through:
            spin = "spin-through (kept)"
        else:
            spin = "stays a stop" if po.spin_candidate(rec) else "no spin-through (not a plain turn-around)"
        return (
            f"Heading {heading}{off(po.HEADING)} · handles {handles}{off(po.HANDLES)}"
            f" · curvature {curvature}{off(po.CURVATURE)} · position {position}{off(po.POSITION)}"
            f" · {corridor} · {spin}"
        )

    def _set_limit(self, key, value) -> None:
        for n in self.targets():
            if value is None:
                n.optimize.pop(key, None)
            else:
                n.optimize[key] = value

    def _on_mode_changed(self, key) -> None:
        if self._loading or not self.targets():
            return
        combo, spin = self.rows[key]
        mode = combo.currentText()
        self._loading = True
        try:
            self._show_amount(key, self._suggested_amount(key) if mode == LIMIT else None)
        finally:
            self._loading = False
        value = {DEFAULT: None, FREE: True, LOCKED: False, LIMIT: float(spin.value())}[mode]
        view = self.gui.central_widget
        view.begin_undo_state()
        self._set_limit(key, value)
        view.commit_undo_state(UNDO_TEXT)
        self._limits_edited()

    def _on_amount_changed(self, key) -> None:
        if self._loading or not self.targets():
            return
        combo, spin = self.rows[key]
        if combo.currentText() != LIMIT:
            return
        # One undo entry per spin session, committed on editingFinished.
        self.gui.central_widget.begin_undo_state_once()
        self._set_limit(key, float(spin.value()))
        self._limits_edited()

    def _commit_amount(self) -> None:
        self.gui.central_widget.commit_undo_state(UNDO_TEXT)

    def _reset_node_limits(self) -> None:
        view = self.gui.central_widget
        view.begin_undo_state()
        for n in self.targets():
            n.optimize = {}
        view.commit_undo_state(UNDO_TEXT)
        self.refresh_node_editor()
        self._limits_edited()

    def _lock_node(self) -> None:
        view = self.gui.central_widget
        view.begin_undo_state()
        for n in self.targets():
            for key in po.SETTINGS:
                n.optimize[key] = False
        view.commit_undo_state(UNDO_TEXT)
        self.refresh_node_editor()
        self._limits_edited()

    def _limits_edited(self) -> None:
        targets = self.targets()
        if targets:
            self.node_effective.setText(self._effective_text(node_record(targets[0]), self.run_options()))
        self.refresh_overlay()
        self._route_changed()

    # --- runs ---------------------------------------------------------------------

    def start_run(self) -> None:
        if self._job is not None:
            return
        view = self.gui.central_widget
        if not (view.start_node and view.end_node and view.spline_manager.splines):
            QMessageBox.information(self, "Optimize", "Designate a start and an end node first.")
            return
        options = self.run_options()
        if not options.settings:
            QMessageBox.information(self, "Optimize", "Tick at least one setting the run may change.")
            return
        text = self.gui.convert_nodes()
        try:
            self._job = optimizer_worker.OptimizerJob(
                text, self._action_names(), self.gui.current_constraints(), options
            )
            self._job.start()
        except Exception as exc:
            logger.exception("Could not start the optimizer")
            self._job = None
            QMessageBox.warning(self, "Optimize", f"Could not start the optimizer.\n\n{exc}")
            return
        self._job_text = text
        self._job_options = options
        self._job_started = time.monotonic()
        self._stop_requested = False
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.progress_bar.show()
        self.status_label.setText("Starting…")
        self._poll_timer.start()

    def stop_run(self) -> None:
        if self._job is None:
            return
        self._stop_requested = True
        self._job.request_stop()
        self.stop_button.setEnabled(False)
        self.status_label.setText("Stopping; keeping the best route found so far…")

    def shutdown(self) -> None:
        self._poll_timer.stop()
        if self._job is not None:
            self._job.kill()
            self._job = None

    def _poll(self) -> None:
        # Runs from a QTimer: nothing may escape (it would kill the app silently).
        try:
            job = self._job
            if job is None:
                self._poll_timer.stop()
                return
            elapsed = time.monotonic() - self._job_started
            for kind, payload in job.poll():
                if kind == optimizer_worker.PROGRESS and not self._stop_requested:
                    self.status_label.setText(f"{payload}  ({elapsed:.0f} s)")
                elif kind == optimizer_worker.DONE:
                    self._end_run()
                    self._set_result(payload)
                    return
                elif kind == optimizer_worker.ERROR:
                    self._end_run()
                    logger.error(f"Optimizer failed:\n{payload}")
                    self.status_label.setText("The optimizer failed; see the log.")
                    last = str(payload).strip().splitlines()[-1:] or [""]
                    QMessageBox.warning(self, "Optimize", f"The optimizer failed.\n\n{last[0]}")
                    return
            if self._job is not None and self.status_label.text() == "Starting…":
                self.status_label.setText(f"Starting… ({elapsed:.0f} s)")
        except Exception:
            logger.exception("Optimizer poll failed")
            self._end_run()

    def _end_run(self) -> None:
        self._poll_timer.stop()
        if self._job is not None:
            self._job.kill()
        self._job = None
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.progress_bar.hide()

    def _set_result(self, result: po.OptimizationResult) -> None:
        self.result = RunResult(
            source_text=self._job_text,
            source_route=rs.parse_route(self._job_text, self._action_names()),
            options=self._job_options,
            result=result,
        )
        pct = 100 * result.saved / result.before_time if result.before_time else 0.0
        self.status_label.setText(
            f"{result.before_time:.2f} s → {result.after_time:.2f} s"
            f" ({-pct:+.1f}%) in {result.seconds:.0f} s"
            + (", stopped early" if result.stopped_early else "")
            + ("" if result.changes else "; no change was an improvement")
        )
        self._refresh_result()

    # --- result -------------------------------------------------------------------

    def _route_changed(self, *_):
        self.refresh_keep_out_list()
        r = self.result
        if r is not None and self.gui.convert_nodes() not in (r.source_text, r.applied_text):
            self.result = None
            self.status_label.setText("The route changed, so the last result was dropped. Run again.")
        self._refresh_result()

    def _is_applied(self) -> bool:
        r = self.result
        return r is not None and r.applied_text is not None and self.gui.convert_nodes() == r.applied_text

    def _refresh_result(self) -> None:
        r = self.result
        has_changes = r is not None and bool(r.result.changes)
        applied = self._is_applied()
        self.apply_button.setEnabled(has_changes and not applied)
        self.save_copy_button.setEnabled(has_changes)
        self.discard_button.setEnabled(r is not None)
        self.details.setPlainText(self._details_text(r) if r is not None else "")
        self._show_result_overlay()

    def _show_result_overlay(self) -> None:
        if not self.active:
            return
        r = self.result
        overlay = self._overlay()
        if r is None or not r.result.changes or self._is_applied():
            overlay.clear_candidate()
            return
        overlay.show_candidate(
            r.result.route, [ch.node for ch in r.result.changes], self.gui.current_constraints()
        )

    def _details_text(self, run: RunResult) -> str:
        r, o = run.result, run.options
        pct = 100 * r.saved / r.before_time if r.before_time else 0.0
        lines = [
            f"Planned {r.before_time:.2f} s -> {r.after_time:.2f} s ({-r.saved:+.2f} s, {-pct:+.1f}%)",
            f"may change {', '.join(o.settings)}; corridor {o.corridor_in:g} in, heading"
            f" +-{o.heading_range_deg:g} deg, handles x{o.handle_scale[1]:g}",
            f"{r.variables} variables, {r.evaluations} evaluations, {r.seconds:.0f} s"
            + (", stopped early" if r.stopped_early else ""),
        ]
        if run.source_route.keep_out:
            lines.append(f"{len(run.source_route.keep_out)} keep-out zone(s)")
        if self._is_applied():
            lines.append("Applied. Undo (Ctrl+Z) puts the old path back.")
        lines.append("")

        nodes = run.source_route.nodes
        if r.changes:
            lines.append("Changed nodes:")
            for ch in r.changes:
                role = node_role(nodes[ch.node])
                lines.append(f"  {ch.node + 1:3d} {role:5s} {ch.describe()}")
        else:
            lines.append("No change was an improvement.")
        moved = [s for s in r.stretches if abs(s.after - s.before) > 5e-4]
        if moved:
            lines.append("Stretches (between stops):")
            lines += [f"  {s.first + 1}->{s.last + 1}: {s.before:.2f} s -> {s.after:.2f} s" for s in moved]
        turned = [(j, d) for j, d in r.action_point_headings if abs(d) >= 0.5]
        if turned:
            lines.append("Action points:")
            lines += [f"  {j + 1}: heading there moved {d:+.1f} deg" for j, d in turned]
        lines.append(f"Feasibility warnings: {len(r.warnings_before)} -> {len(r.warnings_after)}")
        lines += [f"  {tag:7s} {w}" for tag, w in r.warning_changes()]
        if r.untouchable:
            lines.append(
                "Kept as is: nodes " + ", ".join(str(i + 1) for i in r.untouchable)
                + " (a zero handle mid-route or similar the optimizer cannot reproduce)"
            )
        lines.append("")
        lines.append("Planned times only; run both on the robot before trusting the saving.")
        return "\n".join(lines)

    def apply_result(self) -> None:
        r = self.result
        if r is None or not r.result.changes or self._is_applied():
            return
        view = self.gui.central_widget
        before = self.gui.convert_nodes()
        if before != r.source_text:
            QMessageBox.information(self, "Apply", "The route has changed since this run. Run the optimizer again.")
            return
        view._restore_state(rs.dumps_route(r.result.route, self._action_names()))
        after = self.gui.convert_nodes()
        # Set before the push: its indexChanged runs _route_changed, which keeps the
        # result only while the route is where the run started or where Apply left it.
        r.applied_text = after
        # PathStateCommand skips its first redo: the state is already restored.
        self.gui.undo_stack.push(PathStateCommand(view, before, after, "Apply optimizer result"))
        view.auto_save()
        self._refresh_result()
        self.refresh_overlay()
        self.status_label.setText("Applied. Undo puts the old path back.")

    def save_result_copy(self) -> None:
        r = self.result
        if r is None or not r.result.changes:
            return
        folder = self.gui.routes_folder_path or os.getcwd()
        name = self.gui.current_working_file or "route"
        default = os.path.join(folder, f"{name}_opt.txt")
        path, _ = QFileDialog.getSaveFileName(self, "Save optimized copy", default, "Route files (*.txt)")
        if not path:
            return
        if not path.endswith(".txt"):
            path += ".txt"
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "w") as f:
                f.write(rs.dumps_route(r.result.route, self._action_names()))
        except OSError as exc:
            QMessageBox.warning(self, "Save copy", f"Could not write {path}\n\n{exc}")
            return
        logger.info(f"Optimizer result saved to {path}")
        self.status_label.setText(f"Saved to {path}")
        self.gui.settings_dock_widget.refresh_route_files()

    def discard_result(self) -> None:
        self.result = None
        self.status_label.setText("")
        self._refresh_result()

    # --- keep-out zones -----------------------------------------------------------

    ZONE_FIELDS = {"circle": ("X", "Y", "Radius"), "rect": ("X min", "Y min", "X max", "Y max")}

    def _zones(self) -> list:
        return self.gui.central_widget.keep_out

    def _keep_out_row(self) -> Optional[int]:
        row = self.keep_out_list.currentRow()
        return row if 0 <= row < len(self._zones()) else None

    @staticmethod
    def _zone_label(index: int, zone: rs.KeepOutRecord) -> str:
        if zone.shape == "circle":
            x, y, r = zone.values
            return f"{index + 1}. circle ({x:.1f}, {y:.1f}) r {r:g} in"
        x0, y0, x1, y1 = zone.values
        return f"{index + 1}. rectangle ({x0:.1f}, {y0:.1f}) to ({x1:.1f}, {y1:.1f})"

    def refresh_keep_out_list(self) -> None:
        zones = self._zones()
        row = self.keep_out_list.currentRow()
        self.keep_out_list.blockSignals(True)
        try:
            self.keep_out_list.clear()
            for i, zone in enumerate(zones):
                self.keep_out_list.addItem(self._zone_label(i, zone))
            if zones:
                self.keep_out_list.setCurrentRow(min(max(row, 0), len(zones) - 1))
        finally:
            self.keep_out_list.blockSignals(False)
        self._show_zone_values()

    def _show_zone_values(self) -> None:
        row = self._keep_out_row()
        zone = self._zones()[row] if row is not None else None
        self._loading = True
        try:
            for j, (spin, label) in enumerate(zip(self.keep_out_spins, self.keep_out_labels)):
                if zone is None or j >= len(zone.values):
                    spin.hide()
                    label.hide()
                    continue
                radius = zone.shape == "circle" and j == 2
                spin.setRange(0.5, 72) if radius else spin.setRange(-80, 80)
                label.setText(self.ZONE_FIELDS[zone.shape][j] + ":")
                spin.setValue(zone.values[j])
                spin.show()
                label.show()
        finally:
            self._loading = False
        self.remove_keep_out_button.setEnabled(row is not None)

    def _keep_out_selected(self, _row) -> None:
        self._show_zone_values()
        self.refresh_overlay()

    def _add_keep_out(self, shape: str) -> None:
        view = self.gui.central_widget
        node = self.gui.selected_node
        if node is not None and node in view.nodes:
            cx, cy = node.get_abs_x(), node.get_abs_y()
        else:
            cx, cy = 0.0, 0.0
        values = [cx, cy, 6.0] if shape == "circle" else [cx - 6, cy - 6, cx + 6, cy + 6]
        view.begin_undo_state()
        view.keep_out.append(rs.KeepOutRecord(shape, [round(v, 1) for v in values]))
        view.commit_undo_state("Add keep-out zone")
        view.auto_save()
        self.refresh_keep_out_list()
        self.keep_out_list.setCurrentRow(len(view.keep_out) - 1)
        self.refresh_overlay()

    def _remove_keep_out(self) -> None:
        row = self._keep_out_row()
        if row is None:
            return
        view = self.gui.central_widget
        view.begin_undo_state()
        del view.keep_out[row]
        view.commit_undo_state("Remove keep-out zone")
        view.auto_save()
        self.refresh_keep_out_list()
        self.refresh_overlay()

    def _on_keep_out_value(self, *_) -> None:
        if self._loading:
            return
        row = self._keep_out_row()
        if row is None:
            return
        zone = self._zones()[row]
        values = [spin.value() for spin in self.keep_out_spins[: len(zone.values)]]
        if zone.shape == "rect" and (values[0] >= values[2] or values[1] >= values[3]):
            return  # a rectangle needs its min corner below its max; not stored until it is
        # One undo entry per spin session, committed on editingFinished.
        self.gui.central_widget.begin_undo_state_once()
        zone.values = values
        self.keep_out_list.item(row).setText(self._zone_label(row, zone))
        self.refresh_overlay()

    def _commit_keep_out(self) -> None:
        view = self.gui.central_widget
        if view._pending_undo_state is None:
            return
        view.commit_undo_state("Edit keep-out zone")
        view.auto_save()
