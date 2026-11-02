#!/usr/bin/env python3
"""Optimize routes for time by solving node headings, handles, curvature and, where a
route frees them, positions.

    src/.venv/bin/python tools/optimize_route.py close_fourpin              # report only
    src/.venv/bin/python tools/optimize_route.py close_fourpin --write      # also write close_fourpin_opt.txt
    src/.venv/bin/python tools/optimize_route.py archive/sawp --settings handles,curvature
    src/.venv/bin/python tools/optimize_route.py close_fourpin --lock 4:heading,curvature --free 2:heading
    src/.venv/bin/python tools/optimize_route.py straight_cycle --free 2:position --free 4:position --lock 3:heading
    src/.venv/bin/python tools/optimize_route.py close_fourpin --show-locks
    src/.venv/bin/python tools/optimize_route.py --all -j 4                 # every route in the routes folder

Waits, stops and actions never change, nor do turns except where the run makes a
turn-around node a spin-through turn; only the settings a node leaves free do
(see src/utilities/path_optimizer.py). By default a node's heading is locked at the
start, the end and wherever the node carries an action, its curvature at the start and
wherever the robot stops, and its position everywhere; everything else is free. A route
overrides that per node with an "optimize" field, e.g.
"optimize": {"heading": false, "curvature": true}, or with amounts,
{"heading": 10, "handles": 1.5, "position": 6, "corridor": 1.5} (degrees, scale factor,
inches, inches; the planner's Optimize page edits these); --lock/--free override it for
one run (a freed position gets --position-radius), and --save-locks writes them into the
route file (nothing else in it changes). The route's "keep_out" zones bound where the
robot's footprint may go.

Node numbers are 1-based in route-file order, the same numbering the Action Check uses.

The original route is never overwritten: --write saves <name>_opt.txt beside it (or
--suffix). Times are what the profiler plans; whether the robot actually gets them is
for a hardware run to show.
"""

from __future__ import annotations

import argparse
import copy
import logging
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from utilities import headless_profile as hp  # noqa: E402
from utilities import path_optimizer as po  # noqa: E402
from utilities import route_serializer as rs  # noqa: E402
from utilities import sd_card  # noqa: E402


def parse_node_settings(specs, what):
    """["4:heading,curvature", ...] -> {3: ["heading", "curvature"]} (0-based nodes)."""
    out = {}
    for spec in specs or []:
        node, _, names = spec.partition(":")
        settings = [s.strip() for s in names.split(",") if s.strip()] or list(po.SETTINGS)
        unknown = [s for s in settings if s not in po.SETTINGS]
        if not node.isdigit() or int(node) < 1 or unknown:
            raise SystemExit(f"--{what} {spec!r}: expected NODE[:{'|'.join(po.SETTINGS)},...]")
        out.setdefault(int(node) - 1, []).extend(settings)
    return out


def apply_overrides(route, locks, frees):
    for flag, table in ((False, locks), (True, frees)):
        for index, settings in table.items():
            if index >= len(route.nodes):
                raise SystemExit(f"node {index + 1} does not exist ({len(route.nodes)} nodes)")
            for setting in settings:
                route.nodes[index].optimize[setting] = flag


def lock_summary(rec: rs.NodeRecord, options: po.OptimizerOptions) -> str:
    return po.describe_limits(rec, options)


def node_label(i: int, rec: rs.NodeRecord) -> str:
    role = "start" if rec.is_start else "end" if rec.is_end else ""
    return f"{i + 1:3d} {role:5s}"


def run_one(job):
    """Optimize one route; returns the printable report (runs in a worker process)."""
    path, args, cfg = job
    logging.basicConfig(level=logging.ERROR)
    logging.disable(logging.CRITICAL)
    action_names = cfg["robot"].get("actions", [])
    constraints = hp.constraints_from_config(cfg["motion"], cfg["robot"]["track_width"])
    lines = []
    try:
        route = hp.load_route_file(path, action_names)
    except Exception as exc:  # a route that cannot be read is a finding, not a crash
        return False, f"{path.stem}: SKIP ({exc})"
    apply_overrides(route, parse_node_settings(args.lock, "lock"), parse_node_settings(args.free, "free"))

    if args.save_locks and (args.lock or args.free):
        original = hp.load_route_file(path, action_names)
        for rec, new in zip(original.nodes, route.nodes):
            rec.optimize = dict(new.optimize)
        path.write_text(rs.dumps_route(original, action_names))
        lines.append(f"{path.stem}: saved optimizer locks into {path}")

    options = po.OptimizerOptions(
        corridor_in=args.corridor,
        heading_range_deg=args.heading_range,
        curvature_rate_margin=args.curvature_rate_margin,
        action_point_heading_deg=args.action_point_heading,
        settings=tuple(args.settings.split(",")),
        robot_size_in=max(cfg["robot"].get("width", 16.0), cfg["robot"].get("length", 16.0)),
        footprint_in=(cfg["robot"].get("length", 15.0), cfg["robot"].get("width", 16.0)),
        position_radius_in=args.position_radius,
        seeds=args.seeds,
    )
    if args.show_locks:
        lines.append(f"{path.stem}: free settings per node")
        for i, rec in enumerate(route.nodes):
            lines.append(f"  {node_label(i, rec)} {lock_summary(rec, options)}")
        return True, "\n".join(lines)

    try:
        result = po.optimize_route(route, action_names, constraints, options)
    except Exception as exc:
        return False, f"{path.stem}: FAILED ({exc})"

    pct = 100 * result.saved / result.before_time if result.before_time else 0.0
    lines.append(
        f"{path.stem}: {result.before_time:.2f} s -> {result.after_time:.2f} s "
        f"({-result.saved:+.2f} s, {-pct:+.1f}%)  "
        f"{result.variables} variables, {result.evaluations} evaluations, {result.seconds:.0f} s"
    )
    changes = {c.node: c for c in result.changes}
    lines.append(f"  {'node':>9s}  {'free':24s} change")
    for i, rec in enumerate(route.nodes):
        free = "kept as is" if i in result.untouchable else lock_summary(rec, options)
        change = changes[i].describe() if i in changes else ""
        lines.append(f"  {node_label(i, rec)}  {free:24s} {change}")
    for s in result.stretches:
        if abs(s.after - s.before) > 5e-4:
            lines.append(f"  stretch {s.first + 1}->{s.last + 1}: {s.before:.2f} s -> {s.after:.2f} s")
    for j, delta in result.action_point_headings:
        if abs(delta) >= 0.5:
            lines.append(f"  action point {j + 1}: heading there moved {delta:+.1f} deg")
    lines.append(f"  feasibility warnings: {len(result.warnings_before)} -> {len(result.warnings_after)}")
    lines += [f"    {tag:7s} {warning}" for tag, warning in result.warning_changes()]
    if result.untouchable:
        lines.append(
            "  kept as is: nodes " + ", ".join(str(i + 1) for i in result.untouchable)
            + " (a zero handle mid-route or similar the explicit model cannot reproduce)"
        )

    if args.write and result.changes:
        out = path.with_name(f"{path.stem}{args.suffix}.txt")
        if out.exists() and not args.force:
            lines.append(f"  NOT written: {out} exists (use --force)")
        else:
            out.write_text(rs.dumps_route(result.route, action_names))
            lines.append(f"  written {out}")
    elif args.write:
        lines.append("  nothing to write: no change was an improvement")
    return True, "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("routes", nargs="*", help='route names ("close_fourpin", "archive/sawp") or paths')
    parser.add_argument("--all", action="store_true", help="every route in the routes folder")
    parser.add_argument("--suite", action="store_true", help="resolve names against suite/routes")
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--settings", default=",".join(po.SETTINGS),
                        help="settings the run may change at all (default: all four)")
    parser.add_argument("--lock", action="append", metavar="NODE[:SETTINGS]", help="lock settings at a node for this run")
    parser.add_argument("--free", action="append", metavar="NODE[:SETTINGS]", help="free settings at a node for this run")
    parser.add_argument("--save-locks", action="store_true", help="write --lock/--free into the route file")
    parser.add_argument("--show-locks", action="store_true", help="list each node's free settings and stop")
    parser.add_argument("--corridor", type=float, default=3.0, help="max distance from the original path, in (default 3)")
    parser.add_argument("--heading-range", type=float, default=30.0, help="max heading change, deg (default 30)")
    parser.add_argument("--curvature-rate-margin", type=float, default=1.10,
                        help="allowed growth of a stretch's 99th-percentile |dk/ds| (default 1.10)")
    parser.add_argument("--action-point-heading", type=float, default=2.0,
                        help="max heading change at an action point with a reset or lift action, deg (default 2)")
    parser.add_argument("--position-radius", type=float, default=6.0,
                        help='how far a node freed with "position": true may move, in (default 6)')
    parser.add_argument("--seeds", type=int, default=6,
                        help="extra starting layouts when a node may move (default 6)")
    parser.add_argument("--write", action="store_true", help="write <name><suffix>.txt beside each route")
    parser.add_argument("--suffix", default="_opt")
    parser.add_argument("--force", action="store_true", help="overwrite an existing output file")
    parser.add_argument("-j", "--jobs", type=int, default=1, help="routes to optimize in parallel")
    args = parser.parse_args()

    unknown = [s for s in args.settings.split(",") if s not in po.SETTINGS]
    if unknown:
        parser.error(f"--settings: unknown {', '.join(unknown)}; choose from {', '.join(po.SETTINGS)}")
    cfg = yaml.safe_load(Path(args.config).read_text())
    routes_dir = ROOT / "suite" / "routes" if args.suite else Path(cfg["files"]["routes_folder"])
    if not routes_dir.is_absolute():
        routes_dir = ROOT / "src" / routes_dir

    if args.all:
        # Skip earlier optimizer output (close_fourpin_opt, close_fourpin_opt_curv, ...)
        names = [n for n in sd_card.route_names(str(routes_dir)) if args.suffix not in n.rsplit("/", 1)[-1]]
        paths = [routes_dir / f"{n}.txt" for n in names]
    elif args.routes:
        paths = [Path(r) if Path(r).exists() else routes_dir / f"{r}.txt" for r in args.routes]
    else:
        parser.error("name one or more routes, or pass --all")
    missing = [p for p in paths if not p.exists()]
    if missing:
        parser.error("no such route: " + ", ".join(str(p) for p in missing))

    jobs = [(p, args, cfg) for p in paths]
    ok = True
    if args.jobs > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            results = pool.map(run_one, jobs)
            for success, text in results:
                print(text, flush=True)
                ok &= success
    else:
        for job in jobs:
            success, text = run_one(job)
            print(text, flush=True)
            ok &= success
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
