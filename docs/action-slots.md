# Action slots (planner ↔ firmware contract)

An action point in an exported trajectory is one line of the form

```
1 <slot 0> <slot 1> <slot 2> ...
```

The planner writes the slots in `config.yaml` `robot.actions` order
(`gui_manager.save_nodes_to_file`); the firmware reads them positionally in
`Robot::processAction` (`OverrideAutonomous/src/hardware/robot.cpp`). **Both
sides have to agree on this table**, and a mismatch fails silently on the field.

Route *source* files key action values by name instead, so editing the order in
`config.yaml` cannot corrupt a saved route — but it does change what the robot
does with an already-exported trajectory. Re-export after any change here.

| Slot | `config.yaml` name | Firmware | Values |
|---|---|---|---|
| 0 | `intake` | `Intake::set_state` | `0` off · `1` intake · `2` outtake |
| 1 | `lift_state` | `Lift` macros | see below |
| 2 | `right_reset` | `DistanceResetOdometry::resetRight` | `0` off · `1` reset |
| 3 | `left_reset` | `DistanceResetOdometry::resetLeft` | `0` off · `1` reset |
| 4 | `front_reset` | `DistanceResetOdometry::resetFront` | `0` off · non-zero reset |
| 5 | `lift_height` | `Lift::raise_to` | `0` no command · winch deg above home |
| 6 | `intake_lift` | `Intake::raise_lift` / `lower_lift` | `0` no command · `1` raise · `-1` lower |
| 7 | `roller` | `RollerMech::set_enabled` | `0` no command · `1` enable · `2` enable + force PTO · `-1` disable |
| 8 | `wait_lift` | `Robot::followTrajectory` hold | `0` no wait · `1` hold until the lift is done |
| 9 | `chainbar` | `Chainbar` poses | see below |
| 10 | `sticks` | sticks piston (ADI 1) | `0` no command · `1` up (extended) · `-1` down (retracted) |

Slots the line does not carry read as `0`, so a trajectory exported before a
slot existed still follows.

> Until 2026-09-28 the firmware had slots 2 and 3 swapped, so every side reset
> fired the opposite sensor. Boot 513 (`skills`, node 7) settled it: a
> `right_reset` ran the left sensor, which faced open field and read 9999, while
> the right sensor faced the wall. The names here were correct; the firmware
> now matches them. Robots on firmware older than that still have the swap.

## `lift_state` (slot 1)

| Value | Effect | Requires |
|---|---|---|
| `0` | no command (slot 5 may still carry a height) | — |
| `-1` | `Lift::return_home()` — down to the matchload pose (`MATCHLOAD_HEIGHT_DEG`), claw open, loader sensor armed | — |
| `1` | `Lift::score()` — descend onto the tower, open the claw, return to the matchload pose. With a `lift_height` on the same point: `Lift::score_at(height)` — descend to that winch height and release there, ignoring the tower sensor | lift `PRIMED` |
| `2` | `Lift::matchload()` — go to the matchload pose with the claw open, take the piece off the loader when the loader sensor sees it, come up `PRIMED` to `MATCHLOAD_PRIME_HEIGHT_DEG` | — |
| `3` | `Lift::grab()` — close the claw in the matchload pose, no raise | lift `MATCHLOAD` |
| `4` | as `2`. Was matchload at the vertical-stack jaw width; the claw has no widths now, so routes that carry it run a plain matchload | — |
| `5` | as `2`, but the take **raises while the claw closes** instead of holding the winch for `CLAW_CLOSE_MS` first (`Lift::matchload(true)`). One take only | — |
| `6` | as `2`, but the claw **waits closed** in the matchload pose and the lift raises as soon as the loader sees the piece (`Lift::set_matchload_claw_closed(true)`). Sticks until a `2`/`4`/`5` | — |

Since 2026-10-09 the lift has three modes: `MATCHLOAD` (the only rest pose, and
where it boots), `PRIMED` and `SCORING`. There is no loading pose and no angler;
every return, score or `-1`, comes back to `MATCHLOAD`. The claw is a motor:
it spins open for `CLAW_OPEN_MS` and then holds, or spins closed for
`CLAW_CLOSE_MS` and then keeps a light squeeze on the piece.

A take (`2`, `4`, `5`, or the re-arm after a score) needs the loader sensor to
read **clear** after the claw finishes opening *and* after the latest `-1`/`2`/
`4`/`5`, then a piece in range for a few ticks. A jaw left in the beam, or
anything else riding on the robot in front of the sensor, reads like a piece
but never reads clear; the claw then stays open, and a later `lift_height`
closes it on whatever is in the jaws. Arriving back at `MATCHLOAD` with the claw already open (after
a score) re-runs the open once, so plan about `CLAW_OPEN_MS` (500 ms) between
the lift getting home and the robot reaching the loader. A robot that is already
parked against a loaded loader when the claw opens will not take until the
beam clears.

**Close claw (`3`)** exists because the lift boots in MATCHLOAD (the robot
starts physically in the matchload pose), which opens the claw on the second
control tick: without it the preload is dropped, and the only other way to keep
the claw closed at home (a `lift_height`) also raises the cascade. The grab holds the
claw closed in MATCHLOAD until the lift primes, `return_home()` or `stop()`
runs, or a driver button is pressed, and while it holds, the loader sensor is
not armed. A later `lift_height` primes from the closed claw as usual. It is
ignored (and logged) while PRIMED or SCORING. The claw holds closed through the
first lift update, which runs before the first action point is read, so a grab
on the first action point keeps it closed throughout with no open blip.

`lift_state` and `lift_height` in the same point is a conflict, except for a
score: the firmware takes `lift_state` and logs the height as ignored. Split them
across two points.

**Score at a height (`1` plus a `lift_height`).** For a goal the tower sensor
cannot read: the slanted centre goal reads 127–150 mm (and `9999`) with the
piece still about 490 deg up, so a plain score fires on arrival and drops the
piece from height (boots 0676, 0679), or, reading just over the threshold,
waits until the `wait_lift` timeout. `Lift::score_at` lowers to the given
height regardless of the sensor, then opens the claw and releases as a normal
score; reaching home counts as arrived, and a lift already at or below the
height releases where it is. A `wait_lift` on the point holds until the
release, as usual.

**Lift while closing (`5`).** A plain matchload holds the winch for
`CLAW_CLOSE_MS` (300 ms) after the loader sensor fires, so the lift never pulls
the piece off the loader before the claw has it. `5` skips that hold: the lift
starts up to `MATCHLOAD_PRIME_HEIGHT_DEG` on the tick the sensor fires, with
the claw closing on the way. It covers **the next take only**. The automatic
take after the following score (the lift rests armed in MATCHLOAD) closes
first again. If that take should be fast too, put another `5` on a later point,
after the score has released (a `5` before the release cancels the score like
any matchload). A pending fast take is dropped by a plain `2`, by
`-1`, by anything that primes the lift first, and by the driver's down
button. `lift_height` from the matchload pose already closes and raises at
once, so it needs no override.

**Matchload with the claw closed (`6`).** The same take as `2`, on the same
loader rules (read clear, then a piece seated within `LOADER_SEATED_MM`, or
arriving on the drive-speed lead), but the claw waits closed instead of open, so
there is nothing to close: the lift starts up to the raise height on the tick
the sensor fires. The setting sticks, so the automatic take after the following
score also waits closed; `2`, `4` and `5` set it back to open, and every
autonomous run starts open. Driver control has its own toggle (A) for the same
setting.

## `lift_height` (slot 5)

Winch degrees above home, `0` to `Lift::MAX_HEIGHT_DEG` (2000). Non-zero is
itself the command: the lift runs to that height and holds. From the matchload
pose it primes first (claw closes on the piece) and climbs straight there.
Already `PRIMED`, it goes straight there.

Degrees, not inches: nothing in either repo calibrates winch rotation against
cascade travel, so a height is only meaningful against the numbers in
`lift.hpp` and what a run logs:

| Constant | Value | Used for |
|---|---|---|
| `MATCHLOAD_HEIGHT_DEG` | 0 | the matchload (rest) pose; every return and the down button stop here |
| `MATCHLOAD_PRIME_HEIGHT_DEG` | 100 | matchload macro: rise after the claw closes on the loader piece |

These are copied from `lift.hpp` at the time of writing; the header wins if
they disagree.

A target inside `HOME_TOLERANCE_DEG` of home is turned into a `return_home()`.

## `intake_lift` (slot 6)

| Value | Effect |
|---|---|
| `0` | no command: the intake stays where it is |
| `1` | `Intake::raise_lift()` — solenoid up; the intake drops itself (see below) |
| `-1` | `Intake::lower_lift()` — force it down and disarm the auto-drop |

`0` has to be "no command" rather than "down": every point writes every slot, so
a `0`-means-down would drop a raised intake at the very next action point.

A raised intake arms its distance sensor after `Intake::LIFT_ARM_DELAY_MS`
(750 ms) and drops `LIFT_DROP_DELAY_MS` (300 ms) after the sensor first reads
under `LIFT_DROP_DISTANCE_MM` (100 mm). The auto-drop runs off `Intake::update()`,
which `followTrajectory()` and `waitForLift()` tick, so it works mid-segment and
during a lift wait but not while nothing is running between segments. Use `-1`
when a route needs the intake down whether or not a piece came through.

Raising is independent of the intake *state* (slot 0) and of the PTO: the
solenoid is its own port, so a lift move does not delay it.

## `roller` (slot 7)

| Value | Effect |
|---|---|
| `0` | no command: the mech stays as the last point left it |
| `1` | enable: from here on, a roller in view showing the wrong colour gets turned |
| `2` | enable + force PTO: as `1`, but also takes the PTO off a lift that is mid-move. The lift is cancelled (`Lift::stop`), not resumed, and loses its winch hold, so a raised lift falls. Lasts until the next `lift_state`/`lift_height` command or a `-1` |
| `-1` | disable: stops any turn in progress and ignores the sensor |

Autonomous starts with the roller **disabled** (`autonomous()` in `main.cpp`);
driver control always has it enabled. The colour it turns to is not in the
route: it is `RollerMech`'s target, set in `main.cpp` `initialize()` (to be
moved into the brain-screen auton selector), so one route serves either target.

Enabled, the mech is fully automatic off its optical sensor: nothing happens
until a roller is in view (`PROXIMITY_THRESHOLD`) showing a known colour that
is not the target. It then spins the PTO motors until the target colour
appears and `OVERRUN_DEG` past it. In autonomous it shifts the PTO to INTAKE
itself when it sees such a roller (driver control never does), but it has the
lowest PTO priority: it waits while the lift wants the PTO or the intake state
(slot 0) is non-zero. Code `2` drops the lift from that rule; the intake still
wins. A lift command on the same point as a `2` takes the PTO straight back. The intake rollers turn with it, since they share the
motors. Like the intake, it only runs while `followTrajectory()` or
`waitForLift()` is ticking.

## `wait_lift` (slot 8)

`1` makes the robot hold at the point until the lift is done, then carry on with
the rest of the trajectory, so a route no longer needs a guessed `wait_time` for
a lift move. "Done" is:

- **a score:** once the claw lets go — the lift is in `SCORING` with the clamp
  open (`Lift::score_released`). The score macro itself only goes home after the
  tower sensor clears, which needs the robot to drive away, so waiting for the
  lift to settle would never finish.
- **anything else** (matchload, `lift_height`, home, close claw): once the lift
  settles (`Lift::is_settled`).

A lift that is already done ends the hold on the next tick (10 ms). After
`LIFT_WAIT_TIMEOUT_MS` (4000 ms, in `robot.cpp`) the route moves on anyway and
logs `wait_lift TIMED OUT`; a normal hold logs `wait_lift done after N ms`.

The lift command and the wait can share a point (`lift_state = 1` with
`wait_lift = 1`): the command is issued first. At a node the order is stop →
hold for the lift → `turn` → `wait_time` → drive on, so any `wait_time` left on
the node becomes extra dwell after the lift.

The hold is only possible at rest:

- A node with `wait_lift` stops by itself (`Node.auto_stop`), unless its stop
  is forced off.
- An action point only stops when its *Stop at Node* is ticked.

Where the robot would still be moving, the planner's Action Check warns and the
firmware ignores the wait (it only holds below `LIFT_WAIT_MAX_SPEED`, 3 in/s).
While holding, the drive is commanded to the stop pose at zero velocity through
RAMSETE, and the lift, intake and roller keep ticking.

The planner's path time can't know how long a lift takes, so the status bar
shows the profile time plus the number of lift waits (`+ 2 lift waits`).

## `chainbar` (slot 9)

| Value | Pose | Claw |
|---|---|---|
| `0` | no command: the bar holds the last pose | — |
| `1` | `STACK` (pickup) | open |
| `2` | `PIN` (pickup) | open |
| `3` | `ALLIANCE_EMPTY` (prime) | closed |
| `4` | `ALLIANCE_PIN` (prime) | closed |
| `5` | `NEUTRAL_PIN` (prime) | closed |
| `-1` | `DORMANT` (the pose driver control holds) | open |

The chainbar is one 5.5W motor swinging a bar with a piston claw on the end
(`OverrideAutonomous/include/hardware/chainbar.hpp`). It has its own motor, so
unlike the lift it never takes the PTO and never stops the intake. Each pose is a
bar angle (`STACK_DEG`, `PIN_DEG`, ... in `chainbar.hpp`, degrees from where the
bar was at boot) held by a PID.

- A **pickup** or **dormant** opens the claw and swings at once.
- A **prime** commanded while the claw is open closes the claw first and holds
  the bar for `CLAW_CLOSE_MS` (250 ms) before it swings, so it never leaves
  without the piece. With the claw already closed it swings at once.

The robot starts with the bar at the dormant angle, which is angle 0. It boots
in a `BOOT` pose there (claw closed) and holds it until the
route or driver control commands something else; driver control commands
`DORMANT` as it starts. Commands are non-blocking and `wait_lift` does not wait
on the chainbar, so give the bar time (a `wait_time`, or a slower segment)
where the route depends on it having arrived. A move that has not settled after
`MOVE_TIMEOUT_MS` (2000 ms) logs `Chainbar move to X TIMED OUT` and holds where
it got to.

## `sticks` (slot 10)

The sticks are one piston on ADI port 1 (`sticks` in `OverrideAutonomous/src/main.cpp`).
**Down is retracted**, and the robot boots with them down. `1` extends them (up),
`-1` retracts them (down), and `0` leaves them as the last point set them. The
codes are explicit rather than a toggle so a route always knows where the sticks
end up. The piston moves at once and nothing waits on it. Driver control still
toggles them with DOWN.

## Sequencing

Lift commands are **non-blocking**: an action point arms the move and the lift
advances one step per `followTrajectory()` tick. Nothing ticks the lift between
trajectories, so a command issued at the last action point of a segment does not
finish until the next segment runs or `Robot::waitForLift()` is called. Within a
segment, `wait_lift` (slot 8) is the way to sequence drive after a lift move.

The lift owns the PTO while it moves, so the intake stops for as long as a lift
move lasts (`Robot::followTrajectory` inhibits it) and resumes at its commanded
state afterwards. The planner warns when one point asks for both.

## Editing behaviour

`src/utilities/action_spec.py` holds the planner half: the editor for each known
slot, how a value is labelled in the node menu, and the export-time checks
(`lint`) behind the "Action Check" dialog. Adding a slot means: append it to
`config.yaml` `robot.actions`, add a spec there, add the slot and its handling
to `Robot::processAction`, and update this table.
