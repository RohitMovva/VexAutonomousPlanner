#!/usr/bin/env python3
"""One-shot: sort loose route files into folders.

    src/.venv/bin/python tools/organize_routes.py --local            # the planner's routes folder (from config.yaml)
    src/.venv/bin/python tools/organize_routes.py --card             # the SD card's routes folder (trajectories)
    src/.venv/bin/python tools/organize_routes.py --card --dry-run

Suite routes (the names in suite/manifest.yaml, plus turn_test) go to "suite/",
everything else at the folder's root goes to "archive/". Files already inside a
subfolder are left alone. On the card, folders.txt is rewritten afterwards so the
brain's selector sees the folders.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from utilities import sd_card  # noqa: E402

SUITE_EXTRAS = {"turn_test"}


def organize(folder: Path, suite_names: set[str], dry_run: bool) -> list[tuple[str, str]]:
    moves = []
    for path in sorted(folder.glob("*.txt")):
        if path.name == sd_card.FOLDER_INDEX:
            continue
        sub = "suite" if path.stem in suite_names else "archive"
        moves.append((path.name, f"{sub}/{path.name}"))
        if not dry_run:
            (folder / sub).mkdir(exist_ok=True)
            shutil.move(str(path), str(folder / sub / path.name))
    return moves


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--local", action="store_true", help="organise the planner's routes folder")
    parser.add_argument("--card", action="store_true", help="organise the SD card's routes folder")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    args = parser.parse_args()
    if not (args.local or args.card):
        parser.error("pick --local, --card, or both")

    cfg = yaml.safe_load(Path(args.config).read_text())
    manifest = yaml.safe_load((ROOT / "suite" / "manifest.yaml").read_text())
    suite_names = set(manifest["routes"]) | SUITE_EXTRAS

    targets = []
    if args.local:
        targets.append(("local", Path(cfg["files"]["routes_folder"])))
    if args.card:
        card = sd_card.SdCard(cfg["sd_card"]["mount_point"], cfg["sd_card"]["routes_folder"])
        if not card.is_mounted():
            print(f"SD card not mounted at {card.mount_point}; insert it and run again", file=sys.stderr)
            return 1
        targets.append(("card", Path(card.routes_folder)))

    for label, folder in targets:
        if not folder.is_dir():
            print(f"{label}: {folder} does not exist", file=sys.stderr)
            continue
        moves = organize(folder, suite_names, args.dry_run)
        verb = "would move" if args.dry_run else "moved"
        print(f"{label} ({folder}): {verb} {len(moves)} files")
        for src, dst in moves:
            print(f"    {src:30s} -> {dst}")
        if label == "card" and not args.dry_run:
            print(f"    folders.txt: {sd_card.write_folder_index(str(folder))}")
            print("    eject the card with tools/sdcard before pulling it, or the last writes may not land")
        elif label == "card":
            print(f"    folders.txt would list: {sorted(set(d.split('/')[0] for _, d in moves) | set(sd_card.subfolders(str(folder))))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
