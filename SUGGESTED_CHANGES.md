# Suggested Changes — VexAutonomousPlanner

Prioritized list from a full-stack review (planner + OverrideAutonomous firmware) on 2026-07-15.
Companion file: `OverrideAutonomous/SUGGESTED_CHANGES.md`.

---

## 1. Bug fixes (done 2026-07-15)

- [x] **First-node turn crash.** Fixed: the first-node turn block now derives the starting pose
  from the path (`-get_heading(0)` + `get_point_at_parameter(0)`) instead of indexing the empty
  result arrays; a first-node wait after a turn now holds the post-turn heading.
- [x] **Final node's `turn`/`wait_time` silently dropped.** Fixed: `generate_motion_profile`
  flushes un-fired node transitions after the sampling loop (marker + turn/wait samples); the
  redundant caller-side `nodes_map.append` in `path.py` was removed. Bonus fix found while
  testing: `build_path` (`spline_manager.py`) indexed `points[i+1]` when the *final* node had a
  turn/reverse — crashed before the profiler even ran. Verified with a headless smoke test
  (turns/waits at first node, last node, and both ends).
- [x] **Unguarded file I/O.** Fixed: trajectory/route reads and writes in `gui_manager.py` are
  wrapped and report via `QMessageBox` (deduped so auto-save can't spam dialogs when the SD
  card is missing); `load_nodes` in `path.py` rejects invalid JSON with a warning dialog.
- [x] **`config.yaml` cleanup.** Fixed: stray top-level `track_width` removed along with its
  root cause (`set_track_width` in `gui_manager.py` was re-writing it on every settings change);
  `routes_folder` now points at the repo's actual location.
- [x] **Crash when loading a second route** (found by user, not in the original review).
  `clear_nodes` popped each node before `delete_node()` called back into `remove_node()`, whose
  `self.nodes.index(...)` then raised `ValueError` — so switching routes (or "Clear All Nodes")
  always crashed. `clear_nodes` now lets `remove_node` own the list removal, and `remove_node`
  tolerates already-removed nodes. Verified headless: load → switch → switch back → clear.

## 2. Performance (done 2026-07-15)

Measured on an 8-node route with a reverse and a turn; all outputs verified bit-identical to the
pre-optimization reference:

- [x] **Vectorized spline evaluation.** New batch APIs `get_points/get_derivatives/
  get_second_derivatives` on the spline (basis rows × `(N,6,2)` segment stack via einsum) and
  `get_points_at_parameters` etc. on the manager (`np.searchsorted` over cumulative segment
  offsets instead of the per-call linear scan). `build_lookup_table` and
  `precompute_path_properties` now use them: **table rebuild 1140 ms → 24 ms (48×)**.
- [x] **Full profile generation 1713 ms → 134 ms (12.8×).** Biggest single win beyond the spline
  batching: `lerp()` was handed `sample_times` as a Python list, so `np.searchsorted` re-converted
  ~4500 elements to an array on every one of ~1400 calls. Also batched the per-sample
  `distance_to_time`/`get_curvature` lookups in `forward_backward_pass` (new
  `distances_to_times`/`get_curvatures` + vectorized `_interpolate_properties`).
- [x] **Motion profile cached** in `PathWidget.generate_motion_profile_lists`, keyed on a
  fingerprint of actual node/action/constraint state (never stale by construction — no
  invalidation hooks to miss). Second graph view / save after a view: **113 ms → 0.07 ms**.
- [x] **Canvas queries vectorized.** `update_spline` samples the drawn path in one batch call
  (35 ms → 0.7 ms per drag tick) and keeps the samples; `find_closest_point_on_path` is now
  argmin over that table + a 64-point refinement (**~27 ms → 0.5 ms per mouse move**);
  `find_path_angle_at_point`/`draw_rect` read the angle from the spline tangent instead of a
  101-sample scan.
- [x] **Per-evaluation debug checks removed** from `get_point` (scalar evaluators are now plain
  `basis @ segment` dot products).
- [x] ~~QThread worker for saves~~ — not needed: the full save path is now well under 150 ms.

## 3. UX improvements

- [x] **Undo/redo** (done 2026-07-15). Snapshot-based `QUndoStack`: since the path already
  round-trips through `convert_nodes`/`load_nodes`, each command stores serialized
  before/after states — one mechanism covers adds, deletes, drags, mirror, clear-all, and
  every context-menu property edit (menus block in `exec`, so one snapshot pair per menu).
  No-op edits push nothing; loading a route resets history. Edit menu added.
- [x] **Keyboard shortcuts** (done 2026-07-15). Ctrl+S save, Ctrl+O load, Ctrl+Shift+C
  convert-to-string, Ctrl+Shift+V load-from-string, Ctrl+T set working file, Ctrl+Z undo,
  Ctrl+Y / Ctrl+Shift+Z redo, Del / Backspace delete selected node.
- [x] **Feasibility coloring** (done 2026-07-15). Path stretches where curvature caps the
  chassis below 60% of max velocity draw orange, below 35% red (same
  `v <= max_vel / (1 + |k|·lever)` bound the profiler uses, computed from batch derivatives
  live while dragging). Toggle: Tools → "Show Speed-Limit Coloring".
  Note found along the way: selecting a node nudges it ~0.005 px via the dock's rounded
  write-back (pre-existing, physically irrelevant; undo hooks are ordered around it).
- [ ] **Log overlay (biggest capability win).** Firmware SD-logs pose every 10 ms and
  `OverrideAutonomous/tests/log_analysis/` already parses those logs. Add: load a run log, draw
  the actual robot path over the planned path on the field canvas, with a playback scrubber and
  cross-track-error coloring. Turns path iteration into "run, look, drag the offending node."
- [ ] **Follower simulation before deploy.** Reimplement the firmware's RAMSETE + feedforward
  follower (~100 lines of numpy; reference `OverrideAutonomous/src/controllers/ramsete_controller.cpp`
  and `drivetrain_controller.cpp`) to predict tracking error and animate a simulated robot before
  touching the field.

## 4. Planner↔firmware contract (coordinate with OverrideAutonomous)

- [ ] **Versioned trajectory header.** Prepend a header line to every exported trajectory, e.g.
  `# v2 track_width=13.5 wheel_diam=2.65 max_v=55 hash=<route-hash>`, emitted by `fill_txt_file`
  (`gui_manager.py:232-242`). Firmware validates at load and warns on mismatch — turns silent
  wrong-constants runs into loud failures.
- [ ] **Shared action-slot spec.** ActionPoint slot meanings (`actions[0..9]`) are defined only
  implicitly by `Robot::processAction` in the firmware. Write a single spec (markdown table +
  matching constants in both repos) so reordering actions in the planner can't silently rewire
  robot behavior.
- [ ] **Deployment verification.** Write the export directly to the SD path and verify with a
  checksum; the firmware already logs a `Build config:` line — include the route hash so every
  recorded log traces to the exact route version that ran.
- [ ] **Constraint round-tripping.** Firmware has velocity-tuning helpers that measure kS/kV/kA.
  Add an import path so `config.yaml`'s drivetrain model (`free_speed`, `stall_accel`, etc.) stays
  synced with measured values instead of hand-copied.

## 5. Code quality / hygiene

- [ ] **Delete dead code:**
  - `fill_template` (`gui_manager.py:459-524`, superseded by txt export — call site already
    commented out at `:324`), `src/routes.h`, `src/throwawayidk.py`
  - `src/test.py` (imports non-existent `splines.natural_cubic_spline`)
  - `src/motion_profiling_v2/test.py` (calls removed signatures)
  - `validate_path_continuity` stub (`spline_manager.py:453-457`)
  - Leftover debug `print`s: `gui_manager.py:531`, `path.py:649`
- [ ] **Centralize coordinate magic numbers.** `2000` (image px), `146.446120719` (field inches),
  `12.2038433932` (field feet) are repeated in `node.py:44-45`, `path.py:127-128,380-381,624-625,750,787`,
  `action_point.py:31-32`. One `FieldGeometry` constants module.
- [ ] **Deduplicate:**
  - The 7 near-identical graph methods (`gui_manager.py:571-688`) → one parameterized method.
  - `get_wheel_speeds`/`get_wheel_trajectory` duplicate the diff-drive formula
    (`motion_profile_generator.py:205-211` vs `:703-718`).
  - Scalar `_get_curvature`/`_get_heading` vs their vectorized copies in
    `precompute_path_properties`.
  - Node/ActionPoint dialog boilerplate (`set_velocity`/`set_acceleration`/`action_handler`).
- [ ] **Tests + CI.** Zero tests are currently collected. The spline/profiler math is ideal pytest
  territory: arc-length accuracy vs. Gauss-Legendre reference, G2 continuity at joints, profile
  respects velocity/accel constraints, round-trip save/load. Add a test + lint job to
  `.github/workflows/build.yml` (currently build-only, manual trigger).
- [ ] **Repo hygiene.** `dist/`, `build/`, and `src/logs/` (dozens of dated log folders) appear
  committed — expand `.gitignore` and remove from tracking. `max_jerk` is configured but unused
  by the profiler — implement or remove the setting.
- [ ] **Add a `CLAUDE.md`** capturing units conventions (route files: inches; trajectory export:
  inches + radians, y-flipped; profiler internals: feet) and the two-file output model
  (route `.txt` = editable source, header-folder `.txt` = compiled trajectory).
