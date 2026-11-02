"""Tests for the SD card mirror (utilities/sd_card.py).

Pure Python - no Qt, no real card. "Mounted" is simulated by patching
SdCard.is_mounted, and the card's routes folder is a tmp_path directory.
Run from the repo root:
    pytest tests/test_sd_card.py -v
"""

import os
import subprocess

import pytest

from utilities import sd_card


@pytest.fixture
def card(tmp_path):
    """A 'mounted' card whose routes folder is tmp_path/card/routes."""
    mount = tmp_path / "card"
    mount.mkdir()
    c = sd_card.SdCard(str(mount), str(mount / "routes"))
    c.is_mounted = lambda: True  # instance patch, so it can coexist with absent_card
    return c


@pytest.fixture
def absent_card(tmp_path):
    mount = tmp_path / "absent"
    c = sd_card.SdCard(str(mount), str(mount / "routes"))
    c.is_mounted = lambda: False
    return c


# --- state ---------------------------------------------------------------------


def test_is_mounted_uses_real_mount_table(tmp_path):
    # A plain directory is not a mount point; the root always is.
    assert not sd_card.SdCard(str(tmp_path), str(tmp_path / "routes")).is_mounted()
    assert sd_card.SdCard("/", "/routes").is_mounted()
    assert not sd_card.SdCard("", "").is_mounted()
    assert not sd_card.SdCard(str(tmp_path / "missing"), "").is_mounted()


def test_is_mounted_follows_a_symlinked_mount_point(tmp_path):
    # /mnt/sdcard is a symlink to the real mount under /mnt/wsl, and
    # os.path.ismount() is False for a symlink unless it is resolved first.
    link = tmp_path / "sdcard"
    link.symlink_to("/")
    assert sd_card.SdCard(str(link), str(link / "routes")).is_mounted()

    dangling = tmp_path / "gone"
    dangling.symlink_to(tmp_path / "missing")
    assert not sd_card.SdCard(str(dangling), "").is_mounted()


def test_describe(card, absent_card):
    assert card.describe() == f"SD card: {card.routes_folder}"
    assert absent_card.describe() == "SD card: not inserted"


def test_contains_only_paths_on_the_card(card, tmp_path):
    assert card.contains(card.routes_folder)
    assert card.contains(card.mount_point)
    assert not card.contains(str(tmp_path / "elsewhere"))
    assert not card.contains("")
    # Same-prefix trap: /mnt/sdcard2 is not inside /mnt/sdcard.
    sibling = tmp_path / "card2"
    sibling.mkdir()
    assert not card.contains(str(sibling))


def test_contains_is_false_when_not_mounted(absent_card):
    assert not absent_card.contains(absent_card.routes_folder)


def test_is_on_card_path_ignores_mount_state(absent_card, tmp_path):
    # The header folder can be configured to the card while the card is out.
    assert absent_card.is_on_card_path(absent_card.routes_folder)
    assert absent_card.is_on_card_path(absent_card.mount_point)
    assert not absent_card.is_on_card_path(str(tmp_path / "elsewhere"))
    assert not absent_card.is_on_card_path("")
    assert not sd_card.SdCard("", "").is_on_card_path("/anything")


# --- pending trajectories --------------------------------------------------------


def test_write_pending_trajectory_parks_file_locally(absent_card, tmp_path):
    absent_card.pending_folder = str(tmp_path / "pending")
    path = absent_card.write_pending_trajectory("auton", "1 2\n")
    assert path == str(tmp_path / "pending" / "auton.txt")
    with open(path) as f:
        assert f.read() == "1 2\n"
    assert absent_card.pending_trajectories() == ["auton"]
    assert not os.path.exists(absent_card.routes_folder)


def test_write_pending_trajectory_requires_folder(absent_card):
    with pytest.raises(OSError):
        absent_card.write_pending_trajectory("auton", "x")
    assert absent_card.pending_trajectories() == []


def test_move_pending_to_card_replaces_and_clears(card, tmp_path):
    card.pending_folder = str(tmp_path / "pending")
    _write(card.trajectory_path("auton"), "stale card copy")
    card.write_pending_trajectory("auton", "fresh")
    card.write_pending_trajectory("new_one", "new")
    _write(str(tmp_path / "pending" / "notes.md"), "ignored")

    assert card.move_pending_to_card() == ["auton", "new_one"]
    with open(card.trajectory_path("auton")) as f:
        assert f.read() == "fresh"
    with open(card.trajectory_path("new_one")) as f:
        assert f.read() == "new"
    assert card.pending_trajectories() == []
    assert os.path.exists(tmp_path / "pending" / "notes.md")
    assert card.move_pending_to_card() == []


def test_move_pending_is_noop_when_not_mounted(absent_card, tmp_path):
    absent_card.pending_folder = str(tmp_path / "pending")
    absent_card.write_pending_trajectory("auton", "x")
    assert absent_card.move_pending_to_card() == []
    assert absent_card.pending_trajectories() == ["auton"]


def _write(path, text, mtime=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    if mtime is not None:
        os.utime(path, (mtime, mtime))


# --- write_trajectory ------------------------------------------------------------


def test_write_trajectory_skips_when_not_mounted(absent_card):
    assert absent_card.write_trajectory("auton", "1 2 3\n") is False
    assert not os.path.exists(absent_card.routes_folder)


def test_write_trajectory_creates_folder_and_file(card):
    assert card.write_trajectory("auton", "1 2 3\n") is True
    path = card.trajectory_path("auton")
    assert path == os.path.join(card.routes_folder, "auton.txt")
    with open(path) as f:
        assert f.read() == "1 2 3\n"
    # No temp file is left behind by the atomic write.
    # folders.txt is the folder index the brain reads; it is always written too.
    assert sorted(os.listdir(card.routes_folder)) == ["auton.txt", sd_card.FOLDER_INDEX]


def test_write_trajectory_overwrites_and_keeps_odd_names(card):
    card.write_trajectory("threepin'", "old\n")
    card.write_trajectory("threepin'", "new\n")
    with open(card.trajectory_path("threepin'")) as f:
        assert f.read() == "new\n"


def test_write_trajectory_propagates_os_errors(card):
    # The routes folder exists as a *file*, so the write cannot succeed.
    with open(card.routes_folder, "w") as f:
        f.write("not a directory")
    with pytest.raises(OSError):
        card.write_trajectory("auton", "x")


# --- sync_trajectories -----------------------------------------------------------


def test_sync_copies_missing_and_newer_only(card, tmp_path):
    local = tmp_path / "local"
    now = 1_800_000_000
    _write(str(local / "missing.txt"), "missing", now)
    _write(str(local / "newer.txt"), "local newer", now + 60)
    _write(card.trajectory_path("newer"), "card older", now)
    _write(str(local / "same.txt"), "same", now)
    _write(card.trajectory_path("same"), "same", now)
    _write(str(local / "older.txt"), "local older", now - 60)
    _write(card.trajectory_path("older"), "card newer", now)
    # Within FAT's 2 s resolution counts as "same", not newer.
    _write(str(local / "fat_slack.txt"), "local", now + 1.5)
    _write(card.trajectory_path("fat_slack"), "card", now)

    copied = card.sync_trajectories(
        str(local),
        ["missing", "newer", "same", "older", "fat_slack", "no_local_file"],
    )

    assert copied == ["missing", "newer"]
    with open(card.trajectory_path("missing")) as f:
        assert f.read() == "missing"
    with open(card.trajectory_path("newer")) as f:
        assert f.read() == "local newer"
    with open(card.trajectory_path("older")) as f:
        assert f.read() == "card newer"
    with open(card.trajectory_path("fat_slack")) as f:
        assert f.read() == "card"
    assert not os.path.exists(card.trajectory_path("no_local_file"))
    assert not any(n.endswith(".tmp") for n in os.listdir(card.routes_folder))


def test_sync_is_noop_when_not_mounted_or_no_folder(absent_card, card, tmp_path):
    local = tmp_path / "local"
    _write(str(local / "a.txt"), "a")
    assert absent_card.sync_trajectories(str(local), ["a"]) == []
    assert not os.path.exists(absent_card.routes_folder)
    assert card.sync_trajectories("", ["a"]) == []
    assert card.sync_trajectories(None, ["a"]) == []


# --- run_helper ------------------------------------------------------------------


class _Result:
    def __init__(self, returncode, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def test_run_helper_success(monkeypatch):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append(argv)
        return _Result(0, stdout="SD card ejected\n")

    monkeypatch.setattr(sd_card.subprocess, "run", fake_run)
    assert sd_card.run_helper("eject") == (True, "SD card ejected")
    assert calls == [["sudo", "-n", sd_card.HELPER, "eject"]]


def test_run_helper_reports_missing_sudo_rule(monkeypatch):
    monkeypatch.setattr(
        sd_card.subprocess,
        "run",
        lambda *a, **k: _Result(1, stderr="sudo: a password is required\n"),
    )
    ok, output = sd_card.run_helper("eject")
    assert ok is False
    assert "a password is required" in output
    assert "tools/sdcard/install.sh" in output


def test_run_helper_reports_helper_failure(monkeypatch):
    monkeypatch.setattr(
        sd_card.subprocess,
        "run",
        lambda *a, **k: _Result(1, stderr="error: no card is mounted\n"),
    )
    assert sd_card.run_helper("eject") == (False, "error: no card is mounted")


def test_run_helper_never_raises(monkeypatch):
    def missing(*a, **k):
        raise FileNotFoundError("sudo")

    monkeypatch.setattr(sd_card.subprocess, "run", missing)
    ok, output = sd_card.run_helper("eject")
    assert ok is False and "install.sh" in output

    def slow(*a, **k):
        raise subprocess.TimeoutExpired(cmd="sudo", timeout=1)

    monkeypatch.setattr(sd_card.subprocess, "run", slow)
    ok, output = sd_card.run_helper("attach")
    assert ok is False and "timed out" in output


def test_card_control_methods_call_helper(monkeypatch):
    seen = []
    monkeypatch.setattr(sd_card, "run_helper", lambda cmd: seen.append(cmd) or (True, ""))
    c = sd_card.SdCard()
    c.eject()
    c.mount()
    c.attach()
    c.release()
    assert seen == ["eject", "mount", "attach", "release"]
