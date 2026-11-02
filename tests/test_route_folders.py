"""Routes one folder deep ("suite/corner_R6"): the card helper, the folder index
the brain reads, the routes tree, and the headless exporter must all agree."""
import os
import subprocess
import sys
from pathlib import Path

import pytest

from utilities import sd_card

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def card(tmp_path):
    mount = tmp_path / "card"
    mount.mkdir()
    c = sd_card.SdCard(str(mount), str(mount / "routes"), str(tmp_path / "pending"))
    c.is_mounted = lambda: True
    return c


def test_route_names_lists_root_then_one_level_of_folders(tmp_path):
    (tmp_path / "turn_test.txt").write_text("x")
    (tmp_path / "suite").mkdir()
    (tmp_path / "suite" / "corner_R6.txt").write_text("x")
    (tmp_path / "suite" / "arc_R6.txt").write_text("x")
    (tmp_path / "archive").mkdir()
    (tmp_path / "archive" / "sawp.txt").write_text("x")
    (tmp_path / "archive" / "notes.md").write_text("x")
    (tmp_path / sd_card.FOLDER_INDEX).write_text("suite\n")
    (tmp_path / ".hidden.txt").write_text("x")
    assert sd_card.route_names(str(tmp_path)) == [
        "turn_test", "archive/sawp", "suite/arc_R6", "suite/corner_R6"]
    assert sd_card.subfolders(str(tmp_path)) == ["archive", "suite"]
    assert sd_card.route_names(str(tmp_path / "missing")) == []


def test_write_trajectory_into_subfolder_and_index(card):
    assert card.write_trajectory("suite/corner_R6", "0 0 0\n")
    assert card.write_trajectory("turn_test", "0 0 0\n")
    assert (Path(card.routes_folder) / "suite" / "corner_R6.txt").read_text() == "0 0 0\n"
    index = Path(card.routes_folder) / sd_card.FOLDER_INDEX
    assert index.read_text() == "suite\n"
    assert card.write_trajectory("archive/sawp", "0 0 0\n")
    assert index.read_text() == "archive\nsuite\n"


def test_pending_round_trip_keeps_subfolders(card, tmp_path):
    card.is_mounted = lambda: False
    path = card.write_pending_trajectory("suite/hop_30in", "0 0 0\n")
    assert path == str(tmp_path / "pending" / "suite" / "hop_30in.txt")
    assert card.pending_trajectories() == ["suite/hop_30in"]
    card.is_mounted = lambda: True
    assert card.move_pending_to_card() == ["suite/hop_30in"]
    assert (Path(card.routes_folder) / "suite" / "hop_30in.txt").exists()
    assert card.pending_trajectories() == []
    assert not (tmp_path / "pending" / "suite").exists()
    assert (Path(card.routes_folder) / sd_card.FOLDER_INDEX).read_text() == "suite\n"


def test_sync_copies_into_subfolders(card, tmp_path):
    local = tmp_path / "local"
    (local / "suite").mkdir(parents=True)
    (local / "suite" / "arc_R6.txt").write_text("a")
    (local / "turn_test.txt").write_text("b")
    names = sd_card.route_names(str(local))
    assert card.sync_trajectories(str(local), names) == ["turn_test", "suite/arc_R6"]
    assert (Path(card.routes_folder) / "suite" / "arc_R6.txt").read_text() == "a"
    assert (Path(card.routes_folder) / sd_card.FOLDER_INDEX).read_text() == "suite\n"


def test_profile_routes_exports_suite_into_suite_folder(tmp_path):
    out = tmp_path / "routes"
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "profile_routes.py"), "--suite", "hop_30in",
         "--export", str(out)],
        capture_output=True, text=True, cwd=ROOT)
    assert result.returncode == 0, result.stderr
    assert (out / "suite" / "hop_30in.txt").exists()
    assert (out / sd_card.FOLDER_INDEX).read_text() == "suite\n"


def test_routes_tree_groups_by_folder(tmp_path):
    pytest.importorskip("PyQt6")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from gui.routes_widget import ROUTE_ROLE, RouteFilesWidget

    (tmp_path / "suite").mkdir()
    (tmp_path / "suite" / "corner_R6.txt").write_text("x")
    (tmp_path / "archive").mkdir()
    (tmp_path / "archive" / "sawp.txt").write_text("x")
    (tmp_path / "turn_test.txt").write_text("x")

    class Parent:
        routes_folder_path = str(tmp_path)
        current_working_file = "suite/corner_R6"
        loaded = []

        def load_route_file(self, name):
            self.loaded.append(name)

    parent = Parent()
    widget = RouteFilesWidget(parent)
    tree = widget.file_tree
    tops = [tree.topLevelItem(i) for i in range(tree.topLevelItemCount())]
    assert [t.text(0) for t in tops] == ["turn_test", "archive", "suite"]
    suite = tops[2]
    assert suite.isExpanded() and not tops[1].isExpanded()
    assert suite.child(0).data(0, ROUTE_ROLE) == "suite/corner_R6"
    widget.on_item_double_clicked(suite.child(0))
    assert parent.loaded == ["suite/corner_R6.txt"]
    widget.on_item_double_clicked(tops[1])  # a folder toggles, never loads
    assert parent.loaded == ["suite/corner_R6.txt"] and tops[1].isExpanded()
