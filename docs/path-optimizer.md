# Path optimizer

`tools/optimize_route.py` shortens a route's planned time by re-solving the shape
settings the route leaves free: per node, the **heading** (tangent direction), the
**handles** (incoming/outgoing tangent magnitudes) and the **curvature** (the normal
part of `tangent_derivative`), and, where the route allows it, the node's
**position** within a radius. It may also make a turn-around node a **spin-through**
turn and choose its turn (see [Spin-through placement](#spin-through-placement)).
Otherwise turns, waits, stops and actions never change. The result is an ordinary
route and the firmware is untouched. The engine is
`src/utilities/path_optimizer.py`.

```bash
src/.venv/bin/python tools/optimize_route.py close_fourpin              # report only
src/.venv/bin/python tools/optimize_route.py close_fourpin --write      # + close_fourpin_opt.txt beside it
src/.venv/bin/python tools/optimize_route.py close_fourpin --show-locks # what each node may change
src/.venv/bin/python tools/optimize_route.py --all -j 4                 # every route, report only
src/.venv/bin/python tools/optimize_route.py straight_cycle --free 2:position --free 4:position --lock 3:heading
```

The original route is never overwritten. Open `<name>_opt.txt` in the planner to
compare, and run both on the robot before trusting the planned saving.

## In the planner

The **Optimize** page of the control panel (or Tools > Optimize Path...) runs the
same engine on the route being edited:

- **Node limits:** for the selected node (or every node of a multi-selection), set
  heading, handles, curvature and position to *Default*, *Free* (the run's range),
  *Locked* or *Limit* (at most an amount; for position, a radius in inches), and
  give the node its own corridor. The line underneath says what the node may
  actually do. Every edit is an undo step and is saved in the route's `optimize`
  field.
- **Keep-out zones:** circles and rectangles in field inches that the robot's
  footprint may get no deeper into than the route already does. *Add circle* /
  *Add rectangle* put one at the selected node; the fields below edit the selected
  zone. Every edit is an undo step and is saved in the route's `keep_out` list.
- **Run:** the run-wide defaults (corridor, heading and handle ranges, the
  action-point heading tolerance, the curvature-rate growth, the radius a *Free*
  position gets, how many extra starting layouts to try when a node may move, and
  which settings may change at all), then *Optimize*. The search runs in a separate process, so the
  window stays usable; *Stop* ends it early and keeps the best route found so far.
  It uses the same speed limits as the status bar's path time.
- **Result:** the last run only. It is drawn dashed over the path with its changed
  nodes ringed, and the box lists the changes, stretch times, action point heading
  moves and feasibility warnings. *Apply* replaces the path with it as one undo step
  (and saves like any other edit); *Save copy...* writes it to a new route file
  instead; *Discard* drops it. Any other change to the route drops it too (an undo
  back to where the run started keeps it).

While the page is showing, the field shows each node's limits and the keep-out
zones. The band is the corridor, the wedge is how far the heading may turn, the ring
around each node is green when heading, handles and curvature are all free, amber
when some are, and red when all are locked, and a dashed circle shows how far a node
may move. Keep-out zones are filled red, the selected one outlined.

## Locks

| Node | Heading | Handles | Curvature | Position |
|---|---|---|---|---|
| start, end | locked | free | locked | locked |
| any node the robot stops at (reverse, turn, forced stop, wait_lift) | free* | free | locked | locked |
| any node with an action | locked | free | free* | locked |
| every other node | free | free | free | locked |

\* unless another row locks it. Curvature is locked wherever the robot sets off from
rest: curvature there asks the robot to be turning the moment it starts, which it
cannot do (far_fourpin's reverse arc drifted 2-3 in every run until its reversal
node's curvature went to 0). A lock keeps the curvature as drawn; it does not zero it.
A node whose position is free gets a heading range of +-90 deg unless its heading is
locked or set, since where a node sits largely decides the direction it wants.

A route overrides these per node with an `optimize` field (see
[route-format.md](route-format.md)): `"optimize": {"heading": false}` locks,
`true` frees to the run's range, and a number frees by at most that much:

| Key | Number means | Locked at |
|---|---|---|
| `heading` | degrees either way | 0 |
| `handles` | scale factor either way (1.5 = x0.67 to x1.5) | 1 |
| `curvature` | change along the normal, 1/ft | 0 |
| `position` | how far the node may move, inches (`true` = `--position-radius`, 6) | 0 |
| `corridor` | this node's corridor, inches (numbers only) | |

A node's corridor applies at the node and blends linearly into its neighbours'
along the spline. The planner's Optimize page edits all of these. From the command line, `--lock 4:heading,curvature` and
`--free 2:heading` change them for one run (1-based node numbers, as in the Action
Check), `--save-locks` writes them into the route file, and `--settings handles`
limits the whole run to some settings. `--position-radius` sets what `--free N:position`
allows and `--seeds` how many extra starting layouts a run with movable nodes tries.

## Keep-out zones

A route may list areas the optimizer must keep the robot out of (see
[route-format.md](route-format.md)):

```json
"keep_out": [
  {"circle": [-64.7, 57.5, 6]},
  {"rect": [-72, 30, -60, 40]}
]
```

Field inches, the same frame as the nodes. The check uses the robot's real
footprint (`robot.length` along the heading by `robot.width`), rotated with the
path at every sample and swept through every turn in place. Like the walls, it is
relative: a result may get no *deeper* into a zone than the route as drawn already
does, so a route that starts inside a zone can still be optimized, and a zone around
a node the robot has to reach constrains only how the robot gets there.

## Guards

Each is checked per stretch (between stops), against the route as drawn:

- **Corridor:** the new path stays within the corridor of the old one *and* still
  covers all of it. The corridor is `--corridor` (3 in) unless a node sets its own.
  A stretch containing a node that may move has no corridor (moving the node is the
  point); the keep-out zones and walls bound it instead.
- **Keep-out zones:** the footprint gets no deeper into any zone.
- **Field walls:** the robot footprint stays inside the field, or no further out
  than the route already went.
- **Tightest turn:** the smallest radius taken while moving gets no smaller.
- **Curvature rate:** the 99th-percentile |dk/ds| grows by at most 10%
  (`--curvature-rate-margin`). The profiler barely charges for fast curvature
  changes and RAMSETE follows them poorly; without this guard the optimizer buys
  time with them. Where a node moves, the old shape of its stretch is no reference,
  so the cap is 10% over the route's worst stretch instead.
- **Action points:** the heading at an action point carrying a reset, a
  `lift_state` or a `wait_lift` moves at most 2° (`--action-point-heading`).
  Intake and `lift_height` points are free to turn.

Nodes whose settings the explicit model cannot reproduce exactly (a zero outgoing
handle on a node the robot drives through, for instance) are left exactly as they
were, along with their neighbours; the report lists them as "kept as is".

## How it works

1. Every node's effective tangent, handles and curvature are read off the fitted
   spline and made explicit, so a node's settings only shape its two neighbouring
   segments.
2. The route is split at stop nodes. The robot is at rest at each, so the route time
   is the sum of independent stretches (plus turns and waits, which do not depend on
   the shape), and a setting only re-times the stretches that contain its node.
3. Each stretch is timed with the profiler's forward-backward pass alone (distance
   over speed on its distance grid), which varies smoothly with the shape.
4. A bounded compass search (derivative-free; the profiler is not smooth) steps one
   setting at a time and keeps any change that saves time and passes the guards. A
   position is two settings (x and y) held inside the node's circle.
5. When any node may move, the time is far from convex in where it sits, so the
   search runs again from `--seeds` other layouts (every movable node somewhere
   else in its circle) and keeps the fastest.

The route it returns is re-profiled in full; that is the time reported.

## Spin-through placement

With `spin_through` among the run's settings (the default; the **Spin-through**
checkbox on the Optimize page), the search also tries each turn-around node as a
spin-through turn ([motion-profiling.md](motion-profiling.md#spin-through-turns)).

**Candidates** are reverse or turn nodes that are not the start or end and have no
action, no wait, no locked heading and no `"spin_through": false` in their
`optimize` field. A locked heading excludes a node because the robot is meant to
arrive there facing that way, which it would not do mid-spin. So lock the heading of a
matchloader or goal node that carries no action (`--lock 3:heading`), or set its
Spin-through row to Locked.

**Units.** A spin-through node joins the stretches either side of it: the robot is no
longer at rest there. The joined unit is timed with the whole profiler instead of the
forward-backward pass, since only the profiler overlaps the spin with the ramps. That
costs about 0.05 s an evaluation, so a run with candidates takes longer, roughly 30 to
50 s on a cycle route.

**Per candidate:**

1. Profile the joined stretch with the node still a stop. That is the time to beat.
2. Starting points: turn of +-180, +-135 or +-90 deg; the node's heading as drawn or
   aimed along the chord between the stops either side; and, where the node may move,
   as far toward the middle of that chord as its radius allows (a flip wants to sit
   on the line between the stops).
3. A short local search (`spin_through_sweeps`) over the nodes of the joined stretch,
   from the two best starts that pass the guards.
4. Keep it only if it beats stopping by `spin_through_min_gain` (0.05 s).

A final sweep then searches everything together. A spin-through node's turn is a
variable (+-180 deg), and its heading range widens to `position_heading_deg` unless the
route sets one.

**Guards** apply to joined units as to any stretch, with two additions. The profile's
own poses (the swing) are checked against the walls and keep-out zones. A unit may not
gain feasibility warnings. The radius and curvature-rate guards skip samples nearer a
spin-through node than either end of the unit *and* inside the spin's envelope
(`|v| / max_velocity + |v k| / max_turn_velocity <= 1`): that slow pivot is the point,
and the profiler polices it.

On the planner's own cycle routes (2026-10-26, headless, `--lock 3:heading`):

| route | before | after | notes |
|---|---|---|---|
| curved_cycle | 5.05 s | 4.24 s | flips at turns of -60 and +45 deg |
| straight_cycle | 5.22 s | 4.44 s | 6 in position radius |
| straight_cycle, `--position-radius 12` | 5.22 s | 4.24 s | flip nodes move ~10-12 in toward the line |

None of this has been flown.

## What it found (planned times, 2026-09-23)

Across the routes folder, planned savings run from nothing on simple suite shapes
to 11% on `skills` (61.2 s → 54.3 s). `close_fourpin`, already tuned by hand, saves
0.35 s (2.6%). Most of the time comes from the **handles**; curvature adds a little,
and freeing headings at action nodes (not allowed by default) would add more.

The biggest single effect is at S-bends. Yaw acceleration is v² · dk/ds, so where
the curvature flips sign over a short distance the 5 rad/s² yaw-acceleration limit
forces a deep speed dip. The optimizer moves the bending towards the stops, where
the robot is slow anyway, and eases the crossover: `chicane_R12` stops dipping to
0.82 ft/s mid-path and saves 26% while its path moves no more than 0.21 in.

**Moving nodes** (2026-09-26). `straight_cycle` turns the robot round with a
reverse node on the way out and another on the way back. With both free to move
6 in and the intake's heading locked, the optimizer moves them about 3.5 in
towards -y and plans 6.25 s -> 5.62 s (-10%); holding them in place, the best it
found was 6.14 s. Doing the same by hand (and also zeroing the start and end
curvature) gave 5.59 s.

These are the profiler's numbers. Whether the robot delivers them, and tracks the
new shapes as well as the old, is for a hardware A/B run to show.
