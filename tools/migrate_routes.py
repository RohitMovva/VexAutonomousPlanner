"""Batch-migrate route files to the current vap-route format.

Reads each route with the same parser the planner uses (so every legacy
layout it accepts, this accepts) and rewrites it in the current versioned
format. Files already in the current format are left untouched unless
--force is given.

Usage (from the repo root):
    python tools/migrate_routes.py                 # every .txt in the configured routes folder
    python tools/migrate_routes.py routes/a.txt    # specific files
    python tools/migrate_routes.py --dry-run       # report only, write nothing
    python tools/migrate_routes.py --backup        # keep <name>.txt.bak of anything rewritten
"""

import argparse
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from utilities import route_serializer  # noqa: E402


def load_config():
    with open(REPO_ROOT / "config.yaml", "r") as f:
        return yaml.safe_load(f)


def collect_files(paths, config):
    if paths:
        return [Path(p) for p in paths]
    routes_folder = Path(config["files"]["routes_folder"])
    if not routes_folder.is_dir():
        sys.exit(f"routes folder from config.yaml does not exist: {routes_folder}")
    return sorted(routes_folder.glob("*.txt"))


def main():
    parser = argparse.ArgumentParser(
        description="Migrate route files to the current vap-route format."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help="route files to migrate (default: every .txt in the configured routes folder)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="report what would change, write nothing"
    )
    parser.add_argument(
        "--backup", action="store_true", help="write <name>.txt.bak before rewriting"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="rewrite files already in the current format (normalizes layout)",
    )
    args = parser.parse_args()

    config = load_config()
    action_names = config.get("robot", {}).get("actions") or []
    files = collect_files(args.paths, config)
    if not files:
        sys.exit("no route files found")

    migrated, current, failed = 0, 0, 0
    for file_path in files:
        try:
            text = file_path.read_text()
            if not text.strip():
                print(f"SKIP     {file_path.name}: empty file")
                current += 1
                continue
            route = route_serializer.parse_route(text, action_names)
        except (OSError, route_serializer.RouteParseError) as e:
            print(f"ERROR    {file_path.name}: {e}")
            failed += 1
            continue

        if (
            route.source_version == route_serializer.FORMAT_VERSION
            and not args.force
        ):
            current += 1
            continue

        for warning in route.warnings:
            # Migrating legacy files is this tool's whole purpose; only
            # surface notes beyond the generic "this is a legacy file".
            if warning.startswith("loaded a legacy"):
                continue
            print(f"NOTE     {file_path.name}: {warning}")

        out = route_serializer.dumps_route(route, action_names)
        label = "would migrate" if args.dry_run else "migrated"
        print(
            f"{'MIGRATE':8} {file_path.name}: {label} "
            f"(v{route.source_version} -> v{route_serializer.FORMAT_VERSION}, "
            f"{len(route.nodes)} nodes, {len(route.action_points)} action points)"
        )
        if args.dry_run:
            migrated += 1
            continue

        if args.backup:
            file_path.with_suffix(file_path.suffix + ".bak").write_text(text)
        file_path.write_text(out)
        migrated += 1

    print(
        f"\n{migrated} migrated, {current} already current, {failed} failed "
        f"({len(files)} total)"
    )
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
