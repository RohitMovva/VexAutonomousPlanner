#!/usr/bin/env python3
"""Profile saved routes headlessly and report what each asks of the chassis.

    src/.venv/bin/python tools/profile_routes.py                 # every route in the config's routes folder
    src/.venv/bin/python tools/profile_routes.py sawp fourpin    # named routes
    src/.venv/bin/python tools/profile_routes.py --compare       # also profile with the yaw caps off
    src/.venv/bin/python tools/profile_routes.py --suite         # the test suite in suite/routes instead
    src/.venv/bin/python tools/profile_routes.py --suite --export /mnt/sdcard/routes   # trajectories -> routes/suite/
    src/.venv/bin/python tools/profile_routes.py --export /mnt/sdcard/routes           # each route into its own subfolder

Routes sit one folder deep ("suite/corner_R6", "archive/sawp"); --export mirrors that
subfolder under DIR and refreshes DIR/folders.txt, the index the brain's route
selector reads. The suite always exports into the "suite" subfolder.

Reads config.yaml for the constraints, exactly as the GUI does, so the numbers
here are the numbers the robot gets. Use it before and after a constraint change
to see which routes it touched and by how much.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from utilities import action_spec  # noqa: E402
from utilities import headless_profile as hp  # noqa: E402
from utilities import sd_card  # noqa: E402


def configured_actions(actions, action_names):
    """Only the action values that have a firmware slot, i.e. get exported."""
    return {name: value for name, value in actions.items() if name in action_names}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="*", help="route names or paths (default: all)")
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--compare", action="store_true", help="also profile with max_angular_* unset")
    parser.add_argument("--suite", action="store_true", help="profile suite/routes instead of the config's routes folder")
    parser.add_argument("--routes-dir", help="profile the routes in this folder instead")
    parser.add_argument("--export", metavar="DIR",
                        help="write each route's trajectory file (firmware format) into DIR, e.g. the card's routes folder")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO if args.verbose else logging.ERROR)

    cfg = yaml.safe_load(Path(args.config).read_text())
    motion = cfg["motion"]
    track_width = cfg["robot"]["track_width"]
    action_names = cfg["robot"].get("actions", [])
    routes_dir = Path(cfg["files"]["routes_folder"])
    if args.routes_dir:
        routes_dir = Path(args.routes_dir)
    elif args.suite:
        routes_dir = ROOT / "suite" / "routes"

    if args.routes:
        paths = [Path(r) if Path(r).exists() else routes_dir / f"{r}.txt" for r in args.routes]
    else:
        paths = [routes_dir / f"{name}.txt" for name in sd_card.route_names(str(routes_dir))]

    def subfolder(path: Path) -> str:
        """Where this route exports to: its subfolder under routes_dir, or 'suite'."""
        if args.suite:
            return "suite"
        try:
            rel = path.resolve().relative_to(routes_dir.resolve())
        except ValueError:
            return ""
        return rel.parent.as_posix() if rel.parent != Path(".") else ""
    if not paths:
        print(f"no routes found in {routes_dir}", file=sys.stderr)
        return 1

    variants = [("capped", motion)]
    if args.compare:
        uncapped = {**motion, "max_angular_velocity": None, "max_angular_acceleration": None,
                    "max_angular_velocity_left_outer": None,
                    "max_angular_velocity_right_outer": None}
        variants.insert(0, ("uncapped", uncapped))

    print(f"track_width {track_width} in, max_velocity {motion['max_velocity']} ft/s, "
          f"max_angular_velocity {motion.get('max_angular_velocity')} rad/s "
          f"(left outer {motion.get('max_angular_velocity_left_outer')}, "
          f"right outer {motion.get('max_angular_velocity_right_outer')}), "
          f"max_angular_acceleration {motion.get('max_angular_acceleration')} rad/s^2")
    header = f"{'route':22s} {'variant':9s} {'dur s':>7s} {'peak v':>7s} {'peak w':>7s} {'min R ft':>9s} {'outer':>6s} {'@cap':>5s}"
    print(header)
    failures = 0
    for path in paths:
        try:
            route = hp.load_route_file(path, action_names)
            nodes, action_points = hp.nodes_from_route(route)
            manager = hp.build_spline_manager(nodes, action_points)
        except Exception as exc:  # a route the profiler cannot load is a finding, not a crash
            print(f"{path.stem:22s} SKIP: {exc}")
            failures += 1
            continue
        for label, m in variants:
            constraints = hp.constraints_from_config(m, track_width)
            profiled = hp.profile_route(manager, constraints)
            r = profiled.report
            if args.export and label == "capped":
                out = Path(args.export) / subfolder(path) / f"{path.stem}.txt"
                out.parent.mkdir(parents=True, exist_ok=True)
                out.write_text(hp.export_trajectory(profiled, nodes, action_points, action_names))
            radius = "straight" if r.min_turn_radius == float("inf") else f"{r.min_turn_radius:9.2f}"
            print(f"{path.stem:22s} {label:9s} {r.duration:7.2f} {r.peak_speed:7.2f} "
                  f"{r.peak_yaw_rate:7.2f} {radius:>9s} {r.peak_outer_wheel_speed:6.2f} {r.samples_at_yaw_cap:5d}")
            for w in r.warnings:
                print(f"{'':22s} {'':9s} WARN {w}")
            if label == "capped":
                points = action_spec.order_points(
                    [configured_actions(n.actions, action_names) for n in nodes],
                    profiled.nodes_map,
                    [configured_actions(ap.actions, action_names) for ap in action_points],
                    profiled.actions_map,
                )
                for w in action_spec.lint(points):
                    print(f"{'':22s} {'':9s} ACTION {w}")
    if args.export:
        folders = sd_card.write_folder_index(args.export)
        print(f"exported into {args.export}; folder index: {folders}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
