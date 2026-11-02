"""SD card integration for the path planner.

The card reader is passed through from Windows with usbipd and auto-mounted by
the system setup in tools/sdcard/ (udev + systemd). This module is the
planner's view of that mount: it mirrors exported trajectories into the card's
routes folder (what the V5 brain reads as /usd/routes) and drives eject/attach
through `sdcard-helper` over passwordless sudo.

Pure Python, no Qt - the GUI glue lives in gui.gui_manager.
"""

import logging
import os
import shutil
import subprocess

logger = logging.getLogger(__name__)

HELPER = "/usr/local/sbin/sdcard-helper"
DEFAULT_MOUNT_POINT = "/mnt/sdcard"
DEFAULT_ROUTES_FOLDER = "/mnt/sdcard/routes"

# Routes live in one level of subfolders under the routes folder ("suite/corner_R6",
# "archive/sawp") or at its root ("turn_test"). A route NAME is that relative path
# without ".txt", always with "/" separators. The brain cannot list directories, so
# the planner keeps an index of the subfolder names next to the trajectories and
# the brain's selector reads it.
FOLDER_INDEX = "folders.txt"


def route_names(folder):
    """Sorted route names under folder: root files first, then one level of
    subfolders as "sub/name". Hidden entries and non-.txt files are skipped."""
    try:
        entries = sorted(e for e in os.listdir(folder) if not e.startswith("."))
    except OSError:
        return []
    files = [e[:-4] for e in entries
             if e.endswith(".txt") and e != FOLDER_INDEX and os.path.isfile(os.path.join(folder, e))]
    nested = []
    for entry in entries:
        path = os.path.join(folder, entry)
        if os.path.isdir(path):
            nested += [f"{entry}/{sub[:-4]}" for sub in sorted(os.listdir(path))
                       if sub.endswith(".txt") and not sub.startswith(".")]
    return files + nested


def subfolders(folder):
    """Sorted names of the immediate subfolders of folder."""
    try:
        return sorted(
            e for e in os.listdir(folder)
            if os.path.isdir(os.path.join(folder, e)) and not e.startswith(".")
        )
    except OSError:
        return []


def write_folder_index(routes_folder):
    """Write folders.txt listing the subfolders, for the brain's route selector.
    Returns the names written."""
    names = subfolders(routes_folder)
    os.makedirs(routes_folder, exist_ok=True)
    _atomic_write(os.path.join(routes_folder, FOLDER_INDEX), "".join(f"{n}\n" for n in names))
    return names

# FAT32 stores modification times with 2-second resolution, so "newer" needs
# slack or a file copied a moment ago looks older than its source.
MTIME_SLACK_SECONDS = 2.0

# How long to wait for the helper. `attach` may wait for the reader and the
# mount to show up, so it gets longer than an eject.
HELPER_TIMEOUT_SECONDS = 90


def _atomic_write(path, content):
    """Write content to path via a temp file + rename.

    A half-written trajectory would make the robot misbehave, so the old file
    stays intact until the new one is completely on the card.
    """
    tmp = path + ".tmp"
    with open(tmp, "w") as file:
        file.write(content)
        file.flush()
        os.fsync(file.fileno())
    os.replace(tmp, path)
    # The rename lives in the directory entry, and on the card's FAT mount that is
    # not durable until the directory itself is synced. A card pulled before that
    # happened has shown up on the brain as a 0-byte folders.txt.
    try:
        dir_fd = os.open(os.path.dirname(path) or ".", os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except OSError:
        pass  # not every filesystem allows fsync on a directory


def _atomic_copy(src, dst):
    tmp = dst + ".tmp"
    shutil.copyfile(src, tmp)
    os.replace(tmp, dst)


def _is_newer(src, dst):
    """True when dst is missing or src was modified after it (FAT slack applied)."""
    if not os.path.exists(dst):
        return True
    return os.path.getmtime(src) > os.path.getmtime(dst) + MTIME_SLACK_SECONDS


def run_helper(command):
    """Run `sudo -n sdcard-helper <command>`.

    Returns (ok, output). Never raises: a missing helper or sudo rule comes
    back as (False, explanation) so the GUI can show it.
    """
    try:
        result = subprocess.run(
            ["sudo", "-n", HELPER, command],
            capture_output=True,
            text=True,
            timeout=HELPER_TIMEOUT_SECONDS,
        )
    except FileNotFoundError:
        return False, "sudo is not available; run tools/sdcard/install.sh"
    except subprocess.TimeoutExpired:
        return False, f"sdcard-helper {command} timed out"
    output = (result.stdout + result.stderr).strip()
    if result.returncode != 0:
        if "password" in output or "sudo:" in output:
            output += "\n\nThe SD card helper is not installed. Run (once):\n  sudo tools/sdcard/install.sh"
        return False, output
    return True, output


class SdCard:
    """The auto-mounted SD card, as the planner sees it.

    pending_folder is where trajectories go when the header folder *is* the
    card but the card is out; they are moved onto the card when it appears.
    """

    def __init__(
        self,
        mount_point=DEFAULT_MOUNT_POINT,
        routes_folder=DEFAULT_ROUTES_FOLDER,
        pending_folder="",
    ):
        self.mount_point = mount_point
        self.routes_folder = routes_folder
        self.pending_folder = pending_folder

    # --- state ---------------------------------------------------------------

    def is_mounted(self):
        """True when a card is mounted at the mount point.

        The mount point is resolved first: /mnt/sdcard is a symlink to the real
        mount under /mnt/wsl (see tools/sdcard/README.md), and os.path.ismount
        reports False for a symlink without following it.
        """
        try:
            return bool(self.mount_point) and os.path.ismount(
                os.path.realpath(self.mount_point)
            )
        except OSError:
            return False

    def describe(self):
        """One-line status for the status bar."""
        if self.is_mounted():
            return f"SD card: {self.routes_folder}"
        return "SD card: not inserted"

    def trajectory_path(self, name):
        return os.path.join(self.routes_folder, f"{name}.txt")

    def is_on_card_path(self, folder):
        """True when folder lies under the mount point, whether or not a card is in."""
        if not folder or not self.mount_point:
            return False
        mount = os.path.realpath(self.mount_point)
        return os.path.commonpath([mount, os.path.realpath(folder)]) == mount

    def contains(self, folder):
        """True when folder is on the card and the card is mounted."""
        return self.is_mounted() and self.is_on_card_path(folder)

    # --- writing -------------------------------------------------------------

    def write_trajectory(self, name, content):
        """Write one trajectory to the card. name may carry a subfolder ("suite/x").

        Returns False without touching anything when the card is not mounted.
        OSErrors (card yanked mid-write, full card) propagate to the caller.
        """
        if not self.is_mounted():
            return False
        target = self.trajectory_path(name)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        _atomic_write(target, content)
        write_folder_index(self.routes_folder)
        logger.info(f"Trajectory written to SD card: {target}")
        return True

    def sync_trajectories(self, local_folder, names):
        """Copy trajectories from local_folder to the card.

        Only files that are newer locally, or missing on the card, are copied;
        nothing on the card is ever deleted. Returns the names copied.
        """
        if not self.is_mounted() or not local_folder:
            return []
        copied = []
        for name in names:
            src = os.path.join(local_folder, f"{name}.txt")
            if not os.path.isfile(src):
                continue
            dst = self.trajectory_path(name)
            if not _is_newer(src, dst):
                continue
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            _atomic_copy(src, dst)
            copied.append(name)
        if copied:
            write_folder_index(self.routes_folder)
            logger.info(f"Copied trajectories to SD card: {copied}")
        return copied

    # --- pending trajectories (header folder is the card, card is out) --------------

    def write_pending_trajectory(self, name, content):
        """Keep a trajectory locally until the card is back. Returns its path."""
        if not self.pending_folder:
            raise OSError("no pending folder configured (sd_card.pending_folder)")
        path = os.path.join(self.pending_folder, f"{name}.txt")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        _atomic_write(path, content)
        logger.info(f"SD card out; trajectory kept at {path}")
        return path

    def pending_trajectories(self):
        """Names of trajectories waiting in the pending folder."""
        if not self.pending_folder or not os.path.isdir(self.pending_folder):
            return []
        return route_names(self.pending_folder)

    def move_pending_to_card(self):
        """Move pending trajectories onto the card. Returns the names moved.

        A pending file was saved while the card was out, so it is newer than
        whatever the card holds and always replaces it.
        """
        if not self.is_mounted():
            return []
        moved = []
        for name in self.pending_trajectories():
            src = os.path.join(self.pending_folder, f"{name}.txt")
            dst = self.trajectory_path(name)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            _atomic_copy(src, dst)
            os.remove(src)
            moved.append(name)
        if moved:
            write_folder_index(self.routes_folder)
            # Leave no empty subfolders behind in the pending area.
            for sub in subfolders(self.pending_folder):
                path = os.path.join(self.pending_folder, sub)
                if not os.listdir(path):
                    os.rmdir(path)
            logger.info(f"Moved pending trajectories to SD card: {moved}")
        return moved

    # --- control (via sdcard-helper) -------------------------------------------

    def eject(self):
        """Flush and unmount the card so it can be pulled. Returns (ok, output)."""
        return run_helper("eject")

    def mount(self):
        """Mount an inserted card that is not mounted (e.g. after an eject)."""
        return run_helper("mount")

    def attach(self):
        """Take the reader back from Windows and wait for the card to mount."""
        return run_helper("attach")

    def release(self):
        """Hand the reader back to Windows (stops auto-attach until `attach`)."""
        return run_helper("release")
