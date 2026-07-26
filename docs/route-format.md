# Route file format (`vap-route`)

Route source files (`routes/<name>.txt`, and the strings produced by
*Convert to String*) are owned by a single module:
[`src/utilities/route_serializer.py`](../src/utilities/route_serializer.py).
Nothing else in the codebase reads or writes the format. The module is pure
Python (no Qt/numpy), so tools and tests can use it standalone.

This file describes the **route source** format only. The trajectory export
written to the header folder / SD card (space-delimited 10 ms samples read by
the firmware's `Trajectory::loadFromFile`) is a separate format and is
unchanged.

## Current format: version 2

A JSON object, one record per line:

```json
{
  "format": "vap-route",
  "version": 2,
  "actions": ["intake", "little_will", "rake"],
  "nodes": [
    {"x":62.938,"y":35.405,"start":true,"tangent":[0,-0.646],"actions":{"intake":1}},
    {"x":49.983,"y":22.142,"stop_mode":1,"wait_time":1.5}
  ],
  "action_points": [
    {"x":10.0,"y":-4.0,"t":1.52,"actions":{"rake":1}}
  ]
}
```

### Design rules

- **Action values are keyed by name, never by position.** Adding, removing,
  or reordering entries in `config.yaml` `robot.actions` cannot corrupt saved
  routes. (This was the main failure mode of the legacy format.)
- **Defaults are omitted** on save and filled back in on load. Zero action
  values are omitted (zero is every action's default).
- **Unknown data survives.** An action name that is no longer in the config
  is kept through a load/save cycle (held on the node as `extra_actions`)
  and flagged once at load; it is *not* exported to the trajectory, since it
  has no firmware slot. Unknown fields from a newer format version are
  ignored with a warning.
- The top-level `"actions"` list is informational: the config order at save
  time (which is also the firmware's action-slot order at export time).

### Units

Same conventions as the rest of the app: positions in field **inches**
(origin at field center, +y down in the planner's canvas sense), `turn` in
degrees, `wait_time` in seconds, tangents/magnitudes in the spline's
feet-based space. `stop_mode` is `0` auto / `1` forced / `2` disabled
(see `src/gui/node.py`).

### Node record fields

| key | type | default | meaning |
|---|---|---|---|
| `x`, `y` | number | *required* | position, inches |
| `start`, `end`, `reverse` | bool | `false` | node roles |
| `stop_mode` | int | `0` | stop behavior (see above) |
| `turn` | number | `0` | in-place turn, degrees |
| `wait_time` | number | `0` | seconds |
| `max_velocity`, `max_acceleration` | number | `0` | per-node constraint override (0 = none) |
| `tangent` | `[x,y]` | `null` | user-pinned tangent |
| `tangent_derivative` | `[x,y]` | `null` | user-pinned second derivative |
| `incoming_magnitude`, `outgoing_magnitude` | number | `null` | spline handle magnitudes |
| `locked` | bool | `false` | node not draggable |
| `visible` | bool | `true` | node drawn on canvas |
| `actions` | object | `{}` | non-zero action values by name |

### Action-point record fields

`x`, `y` (inches), `t` (spline parameter, *required*), plus `stop`,
`wait_time`, `max_velocity`, `max_acceleration`, `actions` with the same
semantics as above.

## Legacy format: version 1 (pre-July-2026)

`[nodes_data, action_data]` (or a bare `nodes_data` list in the oldest
files): each node a positional list of 14 header fields
(`x y start end reverse stop_mode turn wait max_vel max_accel tangent
tangent_derivative in_mag out_mag`) followed by unlabeled action values and,
in later files, `locked`/`visible` flags. Because the action values carried
no names or count, any change to `robot.actions` silently misaligned or
dropped them.

The parser still reads this format (best effort: positional values map onto
the current config's action names in order, which is correct because actions
were historically only appended). Legacy files are rewritten in the current
format on the next save from the GUI.

## Migrating

- Loading a legacy file in the planner shows a one-time note and migrates it
  on the next save.
- Batch: `python tools/migrate_routes.py` (options: `--dry-run`, `--backup`,
  `--force`; specific paths as arguments). `routes/` is **not** tracked by
  git, so prefer `--backup`.

## Changing the format

Bump `FORMAT_VERSION` in `route_serializer.py` only when a change is not
backward compatible (new optional fields don't need a bump — old files just
omit them, new files warn-and-ignore in old planners). Keep the parser able
to read every version it ever wrote. `tests/test_route_serializer.py`
round-trips every real route in `routes/` — run it after any format change:

```bash
pytest tests/test_route_serializer.py -v
```
