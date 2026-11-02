"""The Path panel's action point editor, driven through the real window.

Covers selection, editing t from the panel, keeping action_points sorted (the
profile generator walks the list in order and skips a point that is out of
order), and undo. Run with QT_QPA_PLATFORM=offscreen.
"""

import os
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PyQt6.QtGui import QMouseEvent  # noqa: E402
from PyQt6.QtTest import QTest  # noqa: E402
from PyQt6.QtWidgets import QApplication, QComboBox  # noqa: E402

from utilities.field_scale import FIELD_IMAGE_FT  # noqa: E402  (conftest puts src/ on sys.path)

SRC = Path(__file__).resolve().parents[1] / "src"


@pytest.fixture
def window(monkeypatch):
    # resource_path() resolves ../assets against cwd, as when main.py runs.
    monkeypatch.chdir(SRC)
    app = QApplication.instance() or QApplication([])
    from gui import gui_manager

    w = gui_manager.AutonomousPlannerGUIManager()
    # Never write the user's last-opened route.
    w.auto_save = lambda: None
    w.save_nodes_to_file = lambda *a, **k: None
    w.current_working_file = None
    w.show()
    QTest.qWait(50)

    view = w.central_widget
    view.clear_nodes()
    for x, y in [(400, 1000), (1000, 700), (1600, 1000)]:
        view.add_node(QPointF(x, y))
    view.nodes[0].toggle_start_node()
    view.nodes[-1].toggle_end_node()
    QTest.qWait(50)
    yield w
    w.close()
    app.processEvents()


def _mouse(w, kind, scene_pos):
    vp = w.central_widget.viewport()
    pos = QPointF(w.central_widget.mapFromScene(scene_pos))
    # The scene routes by global position, so it has to be set.
    QApplication.sendEvent(
        vp,
        QMouseEvent(
            kind,
            pos,
            QPointF(vp.mapToGlobal(pos.toPoint())),
            Qt.MouseButton.LeftButton,
            # A release reports no buttons held; otherwise the scene keeps the
            # item as mouse grabber and it swallows the next click.
            Qt.MouseButton.NoButton
            if kind == QEvent.Type.MouseButtonRelease
            else Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        ),
    )
    QTest.qWait(30)


def _click(w, item):
    _mouse(w, QEvent.Type.MouseButtonPress, item.scenePos())
    _mouse(w, QEvent.Type.MouseButtonRelease, item.scenePos())


def _canvas_point(view, t):
    point = view.spline_manager.get_point_at_parameter(t)
    return (point / FIELD_IMAGE_FT + 0.5) * 2000


def _add_points(w, *ts):
    view = w.central_widget
    points = [view.add_action_point(QPointF(), t) for t in ts]
    view.update_path()
    QTest.qWait(50)
    return points


def test_click_selects_and_shows_editor(window):
    a, _ = _add_points(window, 0.5, 1.5)
    panel = window.settings_dock_widget.path_page
    _click(window, a)

    assert window.selected_action_point is a
    assert window.selected_node is None
    assert not panel.action_group.isHidden()
    assert panel.position_group.isHidden() and panel.tangent_group.isHidden()
    assert panel.action_t.value() == pytest.approx(0.5)
    assert panel.action_t.maximum() == 2.0
    assert len(panel.action_value_editors) == len(a.actions)

    _click(window, window.central_widget.nodes[1])
    assert window.selected_action_point is None
    assert panel.action_group.isHidden()


def test_selecting_does_not_change_values(window):
    # Building the combos fires currentTextChanged; that must not write the
    # first choice into the point or push an undo entry.
    (a,) = _add_points(window, 0.5)
    lift = a.actions.index("lift_state")
    a.action_values[lift] = 1.0
    before = window.undo_stack.index()

    window.set_selected_action_point(a)

    assert a.action_values[lift] == 1.0
    assert window.undo_stack.index() == before
    combo = window.settings_dock_widget.path_page.action_value_editors[lift]
    assert isinstance(combo, QComboBox) and combo.currentText() == "Score"


def test_panel_t_moves_sorts_and_undoes(window):
    view = window.central_widget
    a, b = _add_points(window, 0.5, 1.5)
    panel = window.settings_dock_widget.path_page
    window.set_selected_action_point(a)
    before = window.undo_stack.index()

    panel.action_t.setValue(1.8)
    panel.action_t.editingFinished.emit()
    QTest.qWait(50)

    assert a.t == pytest.approx(1.8)
    assert view.action_points == [b, a]
    expected = _canvas_point(view, 1.8)
    assert np.hypot(a.x() - expected[0], a.y() - expected[1]) < 1e-6
    assert window.undo_stack.index() == before + 1

    # The undo state lists the points in their old order; the selection must
    # follow the edited point, not its current list index.
    window.undo_stack.undo()
    QTest.qWait(50)
    assert [p.t for p in view.action_points] == pytest.approx([0.5, 1.5])
    assert window.selected_action_point is view.action_points[0]
    assert panel.action_t.value() == pytest.approx(0.5)

    window.undo_stack.redo()
    QTest.qWait(50)
    assert window.selected_action_point is view.action_points[1]
    assert window.selected_action_point.t == pytest.approx(1.8)


def test_t_is_clamped_to_the_path(window):
    (a,) = _add_points(window, 0.5)
    a.set_t(5.0)
    assert a.t == 2.0
    a.set_t(-1.0)
    assert a.t == 0.0


def test_drag_past_another_point_keeps_order(window):
    view = window.central_widget
    a, b = _add_points(window, 0.3, 1.0)

    _mouse(window, QEvent.Type.MouseButtonPress, a.scenePos())
    target = _canvas_point(view, 1.6)
    _mouse(window, QEvent.Type.MouseMove, QPointF(*target))
    _mouse(window, QEvent.Type.MouseButtonRelease, QPointF(*target))

    assert a.t > b.t
    assert view.action_points == [b, a]


def test_delete_selected_action_point(window):
    view = window.central_widget
    a, _ = _add_points(window, 0.5, 1.5)
    window.set_selected_action_point(a)
    before = window.undo_stack.index()

    window.delete_selected_node()

    assert a not in view.action_points
    assert window.selected_action_point is None
    assert window.undo_stack.index() == before + 1


def test_appending_node_without_end_node_keeps_action_points(window):
    # With no end node, add_node appends; that renumbers no segment, so the
    # action points must stay put (they used to all shift one segment).
    view = window.central_widget
    a, b = _add_points(window, 0.5, 1.5)
    view.nodes[-1].toggle_end_node()
    assert view.end_node is None

    view.add_node(QPointF(1600, 1400))

    assert (a.t, b.t) == (0.5, 1.5)


def test_inserting_node_shifts_later_action_points(window):
    view = window.central_widget
    a, b = _add_points(window, 0.5, 1.5)

    view.add_node(QPointF(1300, 850), 2)  # between nodes 1 and 2: splits segment 1

    assert (a.t, b.t) == (0.5, 1.5)
    view.add_node(QPointF(700, 850), 1)  # between nodes 0 and 1: renumbers 1 onward
    assert (a.t, b.t) == (0.5, 2.5)
