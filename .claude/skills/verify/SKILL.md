---
name: verify
description: How to launch and drive the planner GUI headlessly to verify changes
---

# Verifying VexAutonomousPlanner changes

## Launch (headless)

- Set `QT_QPA_PLATFORM=offscreen` and **run with cwd = `src/`** (not the repo
  root). `resource_path()` resolves `../assets` and `../routes` against cwd;
  with the wrong cwd the field pixmap is empty and `fit_image_to_view`
  divides by zero on `show()`. PyQt6 aborts the process on any Python
  exception escaping a Qt event handler — it looks like a native crash with
  exit code 127 and **no traceback**. If that happens, `sys.settrace` a
  call/line tracer filtered to app files to find the raising line.
- Construct the real window: `from gui import gui_manager;
  w = gui_manager.AutonomousPlannerGUIManager(); w.show()`. Skip
  `qdarktheme.setup_theme()` — not needed for behavior.
- "Error loading route files" on startup is a caught, logged non-error.

## Drive

- `view = w.central_widget` (PathWidget), `vp = view.viewport()`.
- Synthesize input by `QApplication.sendEvent(vp, QMouseEvent(...))` with
  positions from `view.mapFromScene(...)`; `QTest.qWait(...)` between events
  so the 10 ms throttled path rebuild runs.
- **A path/spline only builds when a start AND end node are designated**
  (`_execute_update_path`). After adding nodes, call
  `view.nodes[0].toggle_start_node()` / `view.nodes[-1].toggle_end_node()`,
  or anything needing `view.path` / `_canvas_samples` (e.g. ctrl+shift
  add-action-point) silently does nothing.
- Blocking menus/dialogs: pre-arm `QTimer.singleShot(...)` before sending a
  `QContextMenuEvent`; in the callback use `QApplication.activePopupWidget()`
  (menu) / `activeModalWidget()` (QInputDialog — `setDoubleValue` + `accept()`).
- QAction keyboard shortcuts (Delete, etc.) do NOT dispatch offscreen
  (window never activates). Find the action in `w.menuBar().actions()` and
  `.trigger()` it instead. App-level Ctrl+Z/Ctrl+Y DO work via the
  eventFilter: `QTest.keyClick(w, Qt.Key.Key_Z, ControlModifier)`.
- Screenshots: `view.grab().save(path)` works offscreen. Menu text renders
  as empty boxes offscreen (font glyphs) — assert on `menu.actions()` texts
  instead.

## Known-good driver example

A full end-to-end driver (multi-select feature, 27 checks) was built this
way; pattern: add nodes via synthesized shift+clicks, designate start/end,
send modifier-click/drag events, assert on `view.nodes` /
`view.selected_nodes` / `w.undo_stack` state, grab screenshots.
