# Motion profiling: how it works and what changed

This document explains the motion profiler in `src/motion_profiling_v2/motion_profile_generator.py`
after the correctness pass and the drivetrain-model work. It covers what was broken, why,
what replaced it, and how to tune it.

Read the **Background** section first if you want the changes to make sense; each fix below
depends on knowing how the profiler is structured.

---

## Background: how the profiler is put together

### Units

Everything downstream of the GUI is in **feet**. Route files store node positions in inches;
`PathWidget.update_spline` converts them (`feet = inches / 12`). So:

| quantity | unit |
|---|---|
| `max_velocity` | ft/s |
| `max_acceleration`, `max_deceleration` | ft/s² |
| `track_width` | ft (config stores **inches** under `robot.track_width`; `gui_manager` divides by 12) |
| curvature `κ` | 1/ft |
| arc length | ft |

(The old `Constraints` comments said "meters". They were wrong; they now say ft.)

### The two-stage pipeline

The profiler works in **distance** first, then converts to **time**. This matters for
understanding most of the bugs below.

1. **`forward_backward_pass`** samples the path along its arc length and produces a
   velocity *as a function of distance*, `v(s)`. It does a forward sweep (how fast can I be
   here, given how fast I was one sample back and how hard I can accelerate?) and a backward
   sweep (…and given that I have to be slow enough to make the next corner or stop?). The
   result respects every velocity and acceleration limit at every distance sample.

2. **`generate_motion_profile`** inverts `v(s)` into `v(t)` and resamples it onto a uniform
   10 ms grid, which is what gets exported.

The inversion works because acceleration is constant between two distance samples, so
crossing one takes `2·ds / (v_i + v_{i+1})` — finite even when one end is stopped, as long
as the other isn't. Accumulating those gives the time at each distance sample.

That "as long as the other isn't" is why `MIN_PROFILE_SPEED` exists: stop nodes are floored
to a small non-zero speed (0.001 ft/s) rather than a true zero, so the robot doesn't take
infinite time to leave a stop.

### Reverse nodes

A reverse node is where the robot changes direction. The path is **split into a separate
spline segment** at every reverse node, and the new segment's tangent points roughly
backwards. Two consequences that drive several of the bugs:

- The spline's **heading and curvature jump discontinuously** at a reverse node — they
  belong to two different splines.
- The robot must be **at rest** exactly there. You cannot reverse at speed.

The robot's *physical* heading is continuous through a reverse node (it stops, then drives
backwards while still facing the same way). The exported heading is
`path_tangent ± π·is_reversed`, and those two π's cancel — but only if `is_reversed` flips at
exactly the same instant the path tangent flips.

---

## The eight changes

### 1. The angular-acceleration constraint was disabled

**What was wrong.** In both the forward and backward sweeps, `max_ang_acc_vel` was computed
and then commented out of the `min(...)` that picks the speed limit:

```python
max_linear_vel = min(max_vel_ang, max_vel_kin, max_curve_vel)  # , max_ang_acc_vel
```

It had been switched off in commit `e1939af` ("faster motion profiling") — but it isn't an
optimisation knob, it's a physical constraint.

**Why it matters.** Steering through a curve whose curvature is *changing* costs angular
acceleration: `α = κ·a + v²·(dκ/ds)`. That `v²·dκ/ds` term is paid out of the same wheel
force budget as linear acceleration. Without the constraint, the profiler let the robot
enter a curvature ramp far too fast, and then `max_accel_at_curvature` — which *does* account
for `dκ/ds` — clamped the allowed linear acceleration to zero. Too late: by then the
`v²·dκ/ds` term alone already exceeded what the wheels could produce.

The result was a profile demanding **69 rad/s² of angular acceleration** against a 13.5 limit,
and ~50 ft/s² at the wheels against a 7.3 limit, on 4.7% of samples. Not at reversals — in
ordinary curved driving.

**The fix.** Re-enabled in both sweeps. Angular-accel violations went from 4.7% of samples to
essentially zero.

> **This is why the old profile "fit in 15.05 s".** It didn't. The robot could never have
> executed it. Any time comparison against that number is a comparison against fiction.

---

### 2. Reverse nodes stepped through a velocity discontinuity instead of stopping

**What was wrong.** `forward_backward_pass` sampled distance on a **uniform grid** of `dd`
(0.005 ft) and floored a stop node's velocity at the *first grid sample at or past* the node.
But `generate_motion_profile` flipped `is_reversed` when the spline **parameter** wrapped —
at the node's *true* distance, which can be up to `dd` earlier.

So the profile's zero and the direction change landed at **different distances**.

Near a stop, `v ≈ √(2·a·Δs)`, so even a 0.003 ft gap is worth real speed. Measured on the
first reversal of `sawp.txt`:

```
true node crossing      s = 0.72707 ft   ← is_reversed flips here, |v| = 0.162 ft/s
floored "stop" sample   s = 0.73000 ft   ← profile is only actually at rest here
```

The exported velocity therefore jumped straight from **−0.29 to +0.22 ft/s in one 10 ms
tick** — 50.7 ft/s², seven times the limit. None of the twelve reversals ever reached zero.

Confirmed by sweeping `dd`: the step tracked `2·√(2·a·dd)` almost exactly, so shrinking `dd`
only softens it as `√dd` (and costs linear time). Not a fix.

**The fix.** The distance grid is **no longer uniform**. Every node and action point is pinned
to a sample of its own:

```python
dists = np.unique(np.concatenate([
    np.arange(0.0, total_dist, delta_dist),   # the uniform grid
    [b[0] for b in boundaries],               # + every node / action point, exactly
    [total_dist],
]))
```

Now the stop's zero-velocity sample, the spline's heading/curvature jump, and the direction
change all sit at the same distance. The signed velocity passes smoothly through zero at the
deceleration limit.

Because sample spacing is now variable, both sweeps and the distance→time inversion use a
per-interval `ds` instead of a constant `dd`, and `curvature_derivs` uses `np.gradient` (which
handles non-uniform spacing).

`forward_backward_pass` now returns `(velocities, node_samples, dists)` instead of just
`velocities`, and node transitions in `generate_motion_profile` are driven off **distance**
rather than the parameter wrap.

> **A false start worth knowing about.** My first attempt moved the *sign flip* onto the
> floored sample instead. That fixed the velocity but broke the heading — it introduced ±180°
> jumps, because the heading's π-correction has to stay aligned with where `get_heading`
> actually jumps (the spline split, at the true node distance). A reverse node drives *two*
> things that must fire together. Pinning the node to a sample is what makes them coincide;
> moving one of them does not.

---

### 3. The exported `accelerations` array was not the derivative of the exported velocity

**What was wrong.** Acceleration was computed by differencing the **unsigned** speed and then
applying the direction sign:

```python
accel = (next_vel - current_vel) / dt          # both unsigned
accelerations.append(accel * (1 if not is_reversed else -1))
```

Across a reversal the sign flips *between* the two samples, so this reports a small, bounded
acceleration across a step in velocity that no drivetrain could follow. The array capped out
at a tidy 7.30 while the actual `d(v)/dt` of the array sitting next to it hit 50.7.

Max disagreement: **43.4 ft/s²**. Anything consuming `accelerations` as feedforward was being
lied to at every reverse node.

**The fix.** Acceleration is now differenced from the **signed** `linear_vels`, after the loop,
once the signs are known:

```python
for i in range(len(linear_vels) - 1):
    accelerations[i] = (linear_vels[i + 1] - linear_vels[i]) / dt
```

`times` is uniformly spaced by `dt` (turn and wait blocks included), so a plain difference is
valid across the whole profile. Disagreement is now **exactly zero, by construction**.

---

### 4. `max_deceleration` was silently ignored

**What was wrong.** In the backward sweep:

```python
constraints.max_acc = max_accels[boundary_map[i] - 1]
constraints.max_dec = max_accels[boundary_map[i] - 1]   # ← max_accels, not max_decs
```

`max_accels` only ever held *acceleration* limits, so at every node boundary the deceleration
limit was overwritten with the acceleration value. Setting `max_dec` to anything had **zero
effect on the output** — I confirmed this by sweeping it from 7.3 to 11.0 and getting a
byte-identical duration.

**The fix.** The sweeps no longer mutate `constraints` at all. Per-node overrides are held in
an `accel_caps` list of `Optional[float]` (`None` = "no override, use the global limit"), and
each sweep asks for its own limit:

- forward: `constraints.accel_limit(current_vel, cap)`
- backward: `constraints.decel_limit(current_vel, cap)`

Storing `None` rather than eagerly resolving to `max_acc` is precisely what keeps the two
ramps distinct.

---

### 5. Acceleration is now a function of speed (the drivetrain model)

This is the big one, and it's the reason the route got *faster* despite every fix above making
it more conservative.

**The problem with a single `max_acceleration`.** A motor's torque falls off linearly with
speed. For your drivetrain (4× V5 11 W, 450 RPM, 2.75" wheels, ~14 lb):

| v (ft/s) | acceleration the motors can actually produce |
|---|---|
| 0 | 27.6 |
| 1 | 22.5 |
| 2 | 17.4 |
| 3 | 12.3 |
| **4** | **7.2** |
| 4.25 | 5.9 |

A constant `max_acceleration = 7.3` is what the drivetrain can do at **~4 ft/s — near top
speed**. It's a single number calibrated at the one operating point this route almost never
visits. `sawp.txt` averages 1.44 ft/s and spends 37% of its time under 1 ft/s, exactly where
the motors are three to four times stronger.

So the flat constant was wrong in **both** directions: badly conservative through the eleven
stop-and-go ramps that make up 95% of the run, and slightly *optimistic* at 4.25 ft/s (where
the motors deliver 5.9, not 7.3).

**The model.** `Constraints` gained three optional fields. If they're absent, everything falls
back to the flat `max_acc`/`max_dec` and reproduces the old behaviour **exactly** — this is
opt-in and verified.

```python
stall_accel: float = None     # ft/s^2 available at a standstill
free_speed: float = None      # ft/s at which motor torque reaches zero
traction_accel: float = None  # ft/s^2 the tyres can transmit (mu * g)
```

Three things bind, and all three always apply:

```
accel_limit(v) = min( stall_accel · (1 − v/free_speed),   ← what the motors CAN do
                      max_acceleration,                    ← what we're WILLING to ask for
                      traction_accel )                     ← what the tyres can TRANSMIT
```

Braking is different, and this is the subtle part:

```
decel_limit(v) = min( stall_accel, max_deceleration, traction_accel )   ← note: no v term
```

Driving reverse voltage into a motor that's still spinning forwards puts the back-EMF *in
series* with the supply, so the controller pins itself to its current limit and holds roughly
stall torque all the way down. **Torque is never what limits braking** — `max_deceleration` and
traction are. That's why braking is flat with speed while acceleration isn't.

**The angular budget scales too.** Steering out of a curvature ramp is paid from the same wheel
force, so `limit_velocity_by_ang_accel` had to change. With a flat budget `A`, the speed cap is
just `√(A / |dκ/ds|)`. Under the model, `A` itself falls with `v`, so the cap solves a quadratic
instead:

```
k·v² + (A₀/v_free)·v − A₀ = 0        where  A₀ = 2·stall_accel / track_width,  k = |dκ/ds|
```

Traction adds a second, flat bound; whichever binds first wins. Both sweeps use the
*accelerating* budget for the speed cap, so they agree with each other and the profile can't
oscillate between them — while the backward sweep spends the larger, flat *braking* budget on
the decel ramp itself.

---

### 6. Both curvature constraints were skipped at inflection points

**What was wrong.** Both sweeps special-cased near-zero curvature:

```python
if abs(curvature) < 1e-6:
    max_linear_vel = constraints.max_vel
    max_accel = avail_accel          # ← the full budget, nothing reserved
else:
    ...                              # ← the only branch that consults dκ/ds
```

That branch threw away *both* curvature constraints — `limit_velocity_by_ang_accel` and
`max_accel_at_curvature`. Neither of them needs `κ ≠ 0`. They bind on **dκ/ds**, not `κ`.

**Why it matters.** `α = κ·a + v²·(dκ/ds)`. At `κ = 0` the first term vanishes but the second
does not — the robot still has to *start* rotating, and the wheels still have to find the force
to do it. And an inflection point is exactly where `κ` passes through zero while `|dκ/ds|` is at
its **maximum**: the middle of every S-curve, and the entry and exit of every bend. The guard
disabled the constraints at precisely the samples that needed them most.

> ### ⚠️ How much does this matter in practice? Less than it looks.
>
> **On `sawp.txt` this fix changes precisely nothing** — the before and after profiles are
> byte-identical. The guard's threshold is `1e-6`, which is so tight that a distance sample
> essentially never lands on it: only **2 of 4514 samples** on `sawp.txt` satisfy `|κ| < 1e-6`,
> and both are at a standstill where `v ≈ 0`, so the constraint it skipped cost nothing.
>
> A real path's curvature *crosses* zero **between** samples rather than landing on it. The
> demonstration below hits the branch only because the sample grid falls exactly on the zero
> crossing of a synthetic sinusoid.
>
> So this is a **latent** fix. The branch fires when a path has a genuinely straight run
> (`κ ≡ 0` over a stretch, not merely passing through it) leading into a bend — the profiler
> would then enter the bend with no angular budget reserved. That's a real and reachable
> configuration, and the dead-code removal is worth keeping on its own merits. But it is not
> what is wrong with `sawp.txt`. **See §7.**

Measured on a smooth S-curve (`κ = 3·sin(2πd/20)`, so `κ` crosses zero at `d = 10` where
`|dκ/ds|` peaks at 0.94 /ft²) — a case *engineered* to land a sample on `κ = 0`, driving the
real `forward_backward_pass`:

```
                        BEFORE     AFTER
velocity                 3.551     3.551   ft/s
linear accel            +7.533    +1.095   ft/s²
outer wheel NEEDS       14.009     7.538   ft/s²
outer wheel HAS          7.490     7.526   ft/s²
over budget by          +6.519    +0.012   ft/s²
                          (+87%)      (0%)
```

Note the velocity is **unchanged**. Only one sample per inflection actually satisfies
`|κ| < 1e-6`, so its neighbours were already clamping the speed. The damage was entirely in the
**acceleration**: that one sample was handed the full 7.53 ft/s², when the `v²·(dκ/ds)` term
alone was already eating 6.44 ft/s² of the wheels' force budget.

**The fix.** Both constraints are now asked for unconditionally:

```python
max_linear_vel = min(
    constraints.max_speed_at_curvature(abs(curvature)),
    constraints.limit_velocity_by_ang_accel(curvature_derivs[i], cap),
)
max_accel = min(
    constraints.max_accel_at_curvature(
        current_vel, curvature, curvature_derivs[i], avail_accel
    ),
    avail_accel,
)
```

The `κ ≈ 0` guard existed only because two of the constraints divide by `κ`: `max_vel_ang` and
`max_accel_ang`. Both turn out to be **dead — they can never bind**:

| | cross-multiplies to | |
|---|---|---|
| `max_vel_kin ≤ max_vel_ang` | `tw·κ ≤ 2 + κ·tw` ⟺ `0 ≤ 2` | always true |
| `max_accel_wheel ≤ max_accel_ang` | `A·κ·tw ≤ 2A + A·κ·tw` ⟺ `0 ≤ 2A` | always true |

So deleting them removed the *only* reason for the guard, and everything that remains is
well-defined at `κ = 0`. That also removed a duplicate — `max_vel_kin` is algebraically
identical to `max_speed_at_curvature(κ)` (verified equal to 6 decimals at every κ) — and the
redundant trailing speed clamps, which re-applied that same expression using the **neighbouring**
sample's curvature. In the backward sweep that off-by-one was live: nothing runs after it to
restore acceleration feasibility.

Net effect on the sweeps: 112 lines deleted, and the two are now near-mirror images, which makes
it visible at a glance that they agree on the speed cap.

**It costs nothing.** On the S-curve above, path duration was 9.986 s before and 9.986 s after
(+0.0%); peak velocity 3.619 → 3.615 ft/s. The old profile wasn't going faster — it was lying
about what the wheels could deliver for one sample per inflection, at exactly the moment the
robot has to start rotating the other way.

---

### 7. The sweeps wrote velocities nothing had checked

This was the **dominant** constraint violation on every real route — up to **+14.4 ft/s² over an
11.0 ft/s² budget, on 22% of samples**. It was found while trying to confirm §6 against
`sawp.txt`, and it is a different bug entirely. §6 turned out to be a no-op there; this is what
was actually wrong.

**Background.** The wheels' force budget covers two things at once:

```
outer wheel accel  =  |a|·(1 + |κ|·tw/2)   +   v²·|dκ/ds|·tw/2   ≤   accel_limit(v)
                      └── linear ramp ──┘       └── steering ──┘
```

`limit_velocity_by_ang_accel` sets a speed ceiling from the steering term alone.
`max_accel_at_curvature` then gives whatever linear acceleration is left over. Both were correct.
The bug was in *how the sweeps used them*.

**Cause 1 — the ceiling landed on the wrong sample.** It was computed from `curvatures[i]` and
then applied to `velocities[i+1]` going forward, and to `velocities[i-1]` going backward:

```python
next_vel = min(max_linear_vel, math.sqrt(...))      # forward: ceiling[i] caps v[i+1]
velocities[i + 1] = min(velocities[i + 1], next_vel)

velocities[i - 1] = min(prev_vel, velocities[i - 1], max_linear_vel)   # backward: caps v[i-1]
```

**A sample was therefore bounded by its neighbours' ceilings and never by its own.** Where the
ceiling has a sharp notch — a `dκ/ds` spike at a node — the sample at the bottom of the notch was
bounded only by its two looser neighbours and sailed straight over its own limit. On `sawp.txt`
at `d = 7.690`: own ceiling 1.000 ft/s, neighbours' 1.309 and 2.594, actual velocity **1.033**.

**Cause 2 — the ceiling was a hard clamp *inside* the recursion.** The sweeps' `√(v² + 2·a·ds)`
recursion is the only thing that makes a velocity change feasible. Clamping to the ceiling inside
the loop bypasses it and writes a velocity step nothing has checked. So the profile tracked a
noisy ceiling sample-to-sample — **1.31 → 1.00 → 2.59 ft/s across three consecutive samples** at
one node — and the acceleration implied by that zig-zag was whatever the noise dictated, bounded
by nothing.

**Cause 3 — acceleration was only ever checked at one end of the interval.** The acceleration
held across `[i, i+1]` is still in force when the robot *arrives* at `i+1`, where `dκ/ds` may be
far worse. Nothing checked it there, so the profile could accelerate hard into a curvature ramp
with no room for it.

**The fix.**

1. **Resolve the ceiling per sample, and apply it to `velocities` *before* the sweeps.** The
   sweeps now do nothing but the recursion — no clamping at all. A ceiling notch propagates
   outwards through the recursion at the accel/decel limit, which is what makes it *reachable*.
2. **Bound the acceleration at both ends of each interval** (`Constraints.max_accel_across_interval`).
   The far end's velocity isn't independent — `v₁² = v₀² + 2·a·ds` — and substituting that in
   leaves an inequality that is **linear in `a`**, so it solves exactly:

   ```
   a  ≤  (A − v₀²·|κ'|·tw/2) / (1 + |κ|·tw/2 + ds·|κ'|·tw)
   ```

   Solving it *exactly* is what makes this work. The obvious shortcut — bound `v₁` above and reuse
   `max_accel_at_curvature` — charges the robot for speed it has not reached yet. At a standstill
   (`v₀ = 0`, where the steering debit must vanish) that drives the limit to zero and strands the
   profile at the stop it is trying to leave. It cost `high_8ball.txt` **180% of its runtime**
   before I caught it.
3. **Resolve the per-node acceleration cap once, per sample.** This also fixed the separate bug
   where the backward sweep read a sentinel past the last boundary, so the final node's
   `max_acceleration` override never reached the braking ramp — exactly the "brake gently into the
   endpoint" case it exists for.

**Results.** Across 12 real routes, every constraint violation is gone, for **+0.8% total runtime**:

| route | before (samples over / worst) | after | duration |
|---|---|---|---|
| `sawp.txt` | 1007 / +14.41 | 94 / **+0.08** | 12.93 → 13.04 s (+0.9%) |
| `sawp2.txt` | 583 / +3.85 | 100 / +0.08 | 11.41 → 11.44 s (+0.3%) |
| `sawp3.txt` | 305 / +20.06 | 52 / +0.07 | 14.56 → 14.68 s (+0.8%) |
| `nineball.txt` | 537 / +3.64 | 81 / +0.07 | 9.49 → 9.55 s (+0.6%) |
| `low_7ball.txt` | 537 / +6.23 | 43 / +0.07 | 8.20 → 8.26 s (+0.7%) |
| `high_8ball.txt` | 516 / +20.30 | 116 / +0.08 | 8.37 → 8.41 s (+0.5%) |
| `fourballhigh.txt` | 243 / +28.11 | 5 / +0.05 | 5.60 → 5.67 s (+1.1%) |
| **total (12 routes)** | | | 95.48 → 96.25 s **(+0.8%)** |

Nothing now exceeds **+0.08 ft/s²**, i.e. 0.7% of the budget — and what is left is the genuine
first-order discretisation residual, which halves exactly when `dd` halves (2.00×, 1.96×, 2.00×,
1.99×) and vanishes below `dd = 0.0025`. That is the signature of a converging numerical scheme
rather than a bug, which is precisely what the old behaviour was *not*.

**Re-verified since, across every route in `routes/`** (39 of 41 load; `lowsafe.txt` is not valid
JSON and `straight_line.txt` fails `build_path`). The worst wheel-budget overshoot anywhere is
**+0.0825 ft/s²** (`goal_to_matchloader.txt`); no route exceeds the 4.25 ft/s wheel-speed limit;
and every route sits on the same first-order convergence curve, halving with `dd` — including the
ones with action points and per-node velocity caps, which the original 12-route table did not
cover. Use the corrected budget-pairing rule below, or five of them will look like they violate
the budget by 1–4 ft/s² when they do not.

---

### 8. The torque-speed model was evaluated at the chassis speed, not the wheel's

**This one only exists on arcs.** §1–§7 were all found and fixed on `sawp.txt`-style routes,
where it is invisible, and on a straight line it is *exactly* a no-op. It took the first real
curved route (`fun_curve.txt`, logged in `OverrideAutonomous/tests/logs/07132026-5.txt`) to
surface it.

**What was wrong.** `accel_limit(v)` models the motors' torque roll-off as
`stall_accel · (1 − v/free_speed)`, and both sweeps handed it `current_vel` — the **chassis**
velocity. But torque falls off with back-EMF, and back-EMF is set by how fast the **motor** is
turning. On a straight line the chassis and the wheels move at the same speed, so the two are
identical. On an arc the outer wheel runs much faster than the robot's centre:

```
outer wheel speed = v · (1 + |κ|·tw/2)
```

On `fun_curve.txt`'s tight arc, the chassis is at 2.5 ft/s while the **outer wheel is at 4.0
ft/s — against a 5.2 ft/s free speed.** It is running right up against the speed where motor
torque goes to zero, while the profiler still believed it had most of its budget in hand:

| | chassis | outer wheel |
|---|---|---|
| speed | 2.54 ft/s | **4.00 ft/s** |
| `accel_limit` at that speed | 5.63 ft/s² | **2.54 ft/s²** |

A **2.2× overestimate**, on 87 of 316 samples. The profile then demanded an acceleration the
outer motor could not produce, the wheel railed its voltage at 127, fell ~11 in/s behind its
setpoint, and the robot under-rotated: only **83% of the commanded turn differential** was
delivered, so it ran wide with a −1.74 in one-sided cross-track bias.

This is the same species of error as the `stall_accel` mistake in §5 — a profile that is
infeasible for the drivetrain — just hiding one level down, in *which speed* the model is
asked about.

**The fix.** The forward sweep evaluates the roll-off at the outer wheel:

```python
avail_accel = constraints.accel_limit(
    constraints.outer_wheel_speed(current_vel, curvatures[i]), caps[i]
)
```

`decel_limit` needs no such change — it has no velocity term at all (§5: braking holds roughly
stall torque all the way down), so asking it at the wheel rather than the chassis would change
nothing.

**The companion: the outer wheel had no headroom for feedback.** `max_speed_at_curvature` caps
the outer wheel at exactly `max_vel`, so on a hard arc the *reference* pegs the outer wheel at
its budget and RAMSETE has nothing left to correct with — the same "no authority" shape as the
`max_velocity: 4.25` bug in the config reference above, relocated from the straight to the arc.

Turning also costs voltage (scrub — see `KV_ANGULAR` in the firmware), and a motor cannot spend
the same volt twice, so the outer wheel's *reachable* top speed falls as the robot turns harder.
Charging the speed budget against a **wider effective half-track** captures this exactly and
keeps the constraint in the same algebraic form:

```
v ≤ max_vel / (1 + |κ|·lever)        lever = tw/2 + steering_speed_penalty
```

`steering_speed_penalty` is in **feet**, and the units really are a length: it is ft/s of outer-
wheel speed budget forgone per rad/s of turn. Its value comes straight from the firmware —
`KV_ANGULAR / KV = 4.0 / 2.0 = 2.0 in/s per rad/s = 2.0 in = 0.167 ft`.

> Keep `outer_wheel_speed()` and `wheel_speed_lever()` distinct, and do not "simplify" one into
> the other. The first is a **physical rotation rate** (what sets back-EMF) and must stay
> geometric. The second is a **budget** (what the wheel can afford to reach) and carries the
> scrub penalty. They happen to have the same shape; they are not the same quantity.

**Results.** Predicting the outer wheel's voltage through the firmware's own feedforward,
`V = kS + kV·v_outer + kA·a_outer + KV_ANGULAR·|ω|`:

| | peak outer-wheel voltage | samples predicted to rail |
|---|---|---|
| before | **129.3** / 127 | 5 |
| after | **111.9** / 127 | **0** |

which leaves ~15 motor units for RAMSETE. On `fun_curve.txt` the outer wheel's reference drops
from 48.0 in/s (pegged at the budget on 269 samples) to 46.8 (**0 samples** at the budget).

Across all 40 loadable routes: **no route's outer wheel exceeds the firmware's 55.28 in/s cap**,
and total runtime costs **+3.0%** (worst single route +6.3%). Straight-line profiles are
bit-identical — at `κ = 0` the outer wheel *is* the chassis and the lever penalty multiplies
zero, so §1–§7's results are untouched.

> The same chassis-vs-wheel flaw is still present in `limit_velocity_by_ang_accel`, which solves
> its quadratic with `A(v) = A₀·(1 − v/v_free)` at the **chassis** `v`. Fixing it properly means
> re-deriving that quadratic with `v_outer = v·(1 + |κ|·tw/2)` substituted in. It was left alone
> deliberately: `accel_limit` is the dominant term, and that constraint binds mainly near spline
> splits where `v ≈ 0` and the error vanishes. Revisit it if a residual shows up on a curved log.

---

## Configuration reference

Your `config.yaml` under `motion:`. (The GUI rewrites this file with `yaml.dump`, which strips
comments — hence this section.)

```yaml
motion:
  stall_accel: 11.0            # ft/s^2 at a standstill — MEASURED, see below
  free_speed: 5.2              # ft/s, where motor torque hits zero
  traction_accel: 11.0         # ft/s^2, the friction budget
  max_acceleration: 11.0       # ceiling
  max_deceleration: 8.0        # ceiling — deliberately lower
  max_velocity: 4.0            # leaves RAMSETE headroom under the firmware's cap
  steering_speed_penalty: 0.167 # ft — the outer wheel's speed budget pays for scrub (§8)
  max_jerk: 16.0               # currently unused by the profiler
```

**`stall_accel: 11.0` — this is measured, and it replaced a datasheet estimate that was 2×
too strong.** The old value of 22.0 came from datasheet math (4 motors × 0.467 N·m at the
wheel ÷ 0.0349 m radius ÷ 6.35 kg = 27.6 ft/s², derated to 80%). The robot does not deliver
it. Fitting `a(v)` over every full-voltage (≥120/127) tick of the straight 95-inch drive in
`OverrideAutonomous/tests/logs/07132026.txt` gives:

```
a(v) = 132.7 − 2.13·v      (in/s², v in in/s)
   => stall_accel ≈ 11.1 ft/s²      (the model claimed 22.0)
   => free_speed  ≈ 5.19 ft/s       (the model claimed 5.4)
```

In the 2.5–4.5 ft/s band the profile actually operates in, the robot produced roughly **half**
the acceleration the old model promised (e.g. at 3.2 ft/s: 40 in/s² measured against 111
predicted). The profile was a faithful execution of the model — reference acceleration tracked
`min(22·(1 − v/5.4), 11.0)` to within 2% at every sample — so this was the model being wrong,
not the profiler.

What it cost: the drivetrain railed its voltage at ~2.5 ft/s and stayed there, actual velocity
fell up to 8 in/s behind the reference through the rest of the ramp, and the robot ended the
run with 2.7 in of along-track lag it never recovered.

> ⚠️ This is fitted from **one 2.26 s straight run**, and the intercept is extrapolated from
> data in the 30–55 in/s band (below that the profile was capped at `max_acceleration` and the
> voltage never railed, so the robot's true standstill capability was never exercised). Treat
> 11.0 as a conservative working value, not a measurement. A dedicated straight-line
> acceleration sweep from rest would pin it down properly.

**`free_speed: 5.2`** — the fit above says 5.19, and the firmware's own feedforward agrees
independently: at the 127 rail, `v = (127 − kS)/kV = (127 − 2.55)/2.0 = 62.2 in/s = 5.19 ft/s`.
The old 5.4 was the no-load figure (450 RPM × π × 2.75 in). `kV` is well calibrated; it was
`stall_accel` that was off.

**`max_velocity: 4.0` — chosen to leave RAMSETE some authority, not for speed.** The firmware
desaturates wheel setpoints to `Config::MAX_WHEEL_VELOCITY` = 55.276 in/s. The old 4.25 ft/s
is 51.0 in/s, i.e. **92% of that cap** — leaving RAMSETE just 4.3 in/s of correction range.
On `07132026.txt` it spent all of it: chasing the along-track lag above, the commanded wheel
speed sat pinned at the desaturation cap for 20 consecutive ticks, voltage railed at 127, and
the wheels went effectively open-loop (mean velocity error 8.1 in/s while saturated, against
2.2 in/s unsaturated). 4.0 ft/s = 48.0 in/s leaves 7.3 in/s of headroom instead.

This costs almost nothing. Per *Known residuals* below, the route is acceleration-bound and
essentially never reaches `max_velocity` anyway — raising it from 4.25 to 6.0 saved 0.07 s, so
lowering it is nearly free.

**`traction_accel: 11.0` — this is the binding limit on your robot, not motor torque.** Omni
wheels on foam tile break away somewhere near μ ≈ 0.8 (~25.7 ft/s²). Because your drive
encoders are also your odometry, a slipping wheel isn't just lost grip — it's a pose error
that never comes back, and encoder odometry has no way to detect or correct it. 11.0 spends
about 43% of breakaway.

**`max_deceleration: 8.0` is lower than `max_acceleration: 11.0` on purpose.** Under braking,
weight transfer unloads the rear wheels, so an individual wheel can lock and corrupt odometry
*even while the robot as a whole still has grip*. Whole-robot friction isn't the failure mode;
one lying encoder is.

### Tuning it

> ⚠️ The durations in this table were measured with `max_deceleration: 8.0`, before §7. The config
> above now has `11.0`, under which `sawp.txt` runs in **13.040 s**. Treat the *shape* of this
> table as valid and the absolute numbers as stale.
>
> Since §7, `traction_accel` is a number the profiler actually honours (to within 0.7%), so
> tuning it now does what it says. Before §7 it did not — the outer wheel overshot it by up to
> 131% — which means any friction budget you arrived at by tuning against the old behaviour was
> calibrated against a profile the robot was not executing. **Re-tune it.**

| friction budget | as μ | % of omni breakaway | `sawp.txt` duration |
|---|---|---|---|
| 16.0 | 0.50 g | 62% | 11.25 s |
| 13.0 | 0.40 g | 51% | 12.08 s |
| **11.0** | **0.34 g** | **43%** | **13.96 s** ← current |
| 9.0 | 0.28 g | 35% | 14.13 s |
| 8.0 | 0.25 g | 31% | 14.94 s |
| 7.0 | 0.22 g | 27% | 15.94 s ✗ over budget |

You have room to be *more* conservative if odometry drifts — 9.0 still fits in 14.13 s.

**Measure the real breakaway.** The 0.8 g is a community figure, not your robot. Drive a
straight segment at increasing commanded acceleration and compare encoder-integrated distance
against a tape measure. Where they diverge is your true `traction_accel`; take about half of
it. That one measurement replaces every estimate in this document.

---

## Results on `sawp.txt`

14 nodes, 12 spline segments, 22.502 ft, 11 mandatory stops (every reverse node).

Measured with the config below (`max_vel 4.25`, `max_acc 11.0`, `max_dec 11.0`, `stall 22.0`,
`free_speed 5.4`, `traction 11.0`), auditing the profiler's own invariant on the distance grid it
enforces it on — no resampling, so no 10 ms-grid artifacts.

| | before §6/§7 | after §6 | after §6 + §7 |
|---|---|---|---|
| duration | 12.932 s | 12.932 s | **13.040 s** (+0.8%) |
| peak &#124;v&#124; | 4.250 ft/s | 4.250 | 4.250 (limit 4.25) |
| peak wheel speed | 4.250 ft/s | 4.250 | 4.250 (limit 4.25) |
| **worst wheel-budget overshoot** | **+14.41 ft/s²** | +14.41 | **+0.08** |
| samples over budget by >0.05 | 1007 (22.3%) | 1008 | **94 (2.1%)** |
| samples above their own speed ceiling | 7 | 7 | **0** |
| samples where steering alone > budget | 4 | 4 | **0** |
| `accelerations` vs `d(v)/dt` | exact | exact | exact (err 0.0e+00) |
| positions monotonic | ✓ | ✓ | ✓ |
| final position vs arc length | exact | exact | 22.5015 / 22.5018 |
| peak heading step | — | — | 3.0°/tick |
| reversals passing through zero | 11 / 11 | 11 / 11 | 11 / 11 |

**Fix §6 is a no-op on this route** — identical in every column. See the callout in §6: only 2
samples land in the `|κ| < 1e-6` window, and both are at a standstill.

**Fix §7 is the one that mattered.** The outer wheel was being asked for up to 131% of its force
budget on nearly a quarter of the route; it is now within 0.7%, and what remains converges away
as `dd` shrinks. It cost 0.8% of runtime.

The rows that were already healthy — exact acceleration, exact final position, wheel speeds inside
the limit, clean reversals — are the ones fixed by §1–§5, and they have held up throughout.

---

## Known residuals and limitations

None of these are believed to be bugs, but you should know they exist.

**Peak linear acceleration reads ~0.2 ft/s² over the ceiling** (11.05 against 11.0), on exactly
one sample per reversal. This is `MIN_PROFILE_SPEED`: the signed velocity can't pass through a
true zero, so it steps from −0.001 to +0.001, which differences to `2·0.001/0.01 = 0.2 ft/s²`.
Harmless. Lowering `MIN_PROFILE_SPEED` shrinks it further at negligible cost in duration.

**Wheel-acceleration overshoot is now a genuine discretisation residual — and only that.**

> ⚠️ This section used to claim the outer wheel overshot its budget by ~13% and attribute it to
> discretisation. **Both halves of that were wrong.** The real overshoot was **+14.4 ft/s², 131%
> of the budget, on 22% of samples**, and it did not converge as `dd → 0` — so it could not have
> been discretisation. It was bug **§7**, now fixed.

What remains is first-order in `dd`: each sweep evaluates its limit at the interval's *starting*
sample while the binding speed is at the faster end. Measured on `sawp.txt`, it halves exactly
when `dd` halves — the signature of a converging scheme:

| `dd` (ft) | worst overshoot | ratio | samples over 0.05 |
|---|---|---|---|
| 0.0200 | +0.306 ft/s² | — | 130 |
| 0.0100 | +0.159 | 1.93× | 157 |
| **0.0050** | **+0.081** | **1.96×** | 94 ← default |
| 0.0025 | +0.041 | 2.00× | **0** |
| 0.00125 | +0.020 | 1.99× | **0** |

At the default `dd = 0.005` that is 0.081 ft/s² — **0.7%** of an 11 ft/s² budget. Halve `dd` and
it disappears entirely, at the cost of roughly twice the profiling time. Worth knowing when you
size `traction_accel`, but not a margin you need to design around.

**`dκ/ds` is unbounded at spline splits.** The path is split into separate splines at reverse and
turn nodes, so `κ` is *discontinuous* there, and `curvature_derivs = np.gradient(curvatures,
dists)` estimates its derivative as `jump / (2·dd)` — a number with no limit as the grid refines.
Every constraint derived from `dκ/ds` inherits that.

In practice it is harmless: spline splits only occur at reverse and turn nodes, which are
**always stops**, so `v ≈ 0` and the `v²·dκ/ds` term vanishes exactly where `dκ/ds` misbehaves.
Since §7 was fixed, `dd` refines cleanly (the table above), so this is no longer the trap it was.
But it is why the speed ceiling craters to near-zero for the sample or two either side of a
reversal — harmless there, and worth recognising if you ever see it in a plot.

**Wheel acceleration reads high (~27 ft/s²) within ±2 samples of a reversal.** This is a finite
difference of `ω = v·κ` across a genuine curvature discontinuity at |v| ≈ 0. The wheels are
barely turning; it isn't a torque the drivetrain has to make. Ignore it when auditing.

**`motion_profile_angle` (in-place turns) still uses a flat `constraints.max_acc`** and does not
consult the drivetrain model.

**`max_jerk` is configured but unused.** The profiler does not currently bound jerk.

**The path is almost entirely acceleration-bound.** 95% of the run; the robot never gets near
`max_velocity`. Raising `max_velocity` from 4.25 to 6.0 saves **0.07 s** — don't bother. The
levers that matter are the friction budget and the number of reversals.

**Curvature is not the bottleneck.** `sawp.txt` has a 2-inch turn radius (|κ| = 5.9 /ft) between
nodes 5 and 6, tighter than the robot's 6.5-inch half-track. It looks alarming and costs almost
nothing — a variant with a 1-inch radius ran within 0.02 s. Don't chase it. (It *is* the highest
slip-risk spot in the route, though, so it's worth watching if odometry drifts.)

---

## Still outstanding

Known bugs, not yet fixed:

- **`motion_profile_angle` (in-place turns) ignores the drivetrain model** and still uses a flat
  `constraints.max_acc`.
- **`max_jerk` is configured but unused.** The profiler does not bound jerk.
- **`src/motion_profiling_v2/test.py` is stale.** It calls `limit_velocity_by_ang_accel` with a
  signature that no longer exists; the suite errors out.
- **Stray `print("Curvatures:", ...)`** in `spline_manager.py:174`.

Fixed 2026-07-15, previously listed here:

- ~~The final node's `turn` and `wait_time` are silently dropped~~ — `generate_motion_profile`
  now flushes any transitions that never fired inside the sampling loop (always at least the
  final node, which `forward_backward_pass` never assigns a sample), emitting the node's
  `nodes_map` marker followed by its turn/wait samples. The caller-side
  `nodes_map.append(len(times))` in `path.py` was removed — the generator's entry replaces it
  and lands *before* the trailing turn/wait samples instead of blindly at the very end. A
  second crash in the same family was found and fixed along the way: `build_path` indexed
  `points[i + 1]` when the *final* node had a turn/reverse (the `i < len(points) - 1` guard
  existed for `next_length` but not `next_vector`), so a final-node turn crashed before the
  profiler even ran.
- ~~A route whose first node has a turn crashes~~ — the first-node turn block now derives the
  starting pose from the path itself (`-get_heading(0)`, reversed-shifted and normalized, and
  `get_point_at_parameter(0)` for coords) instead of reading `headings[-1]`/`coords[-1]` off
  the still-empty result arrays. A first-node wait that follows a first-node turn now holds
  the post-turn heading rather than resetting to the path heading.

Fixed in an earlier pass, previously listed here:

- ~~The backward sweep ignores the last node's acceleration override~~ — fixed by §7, which
  resolves the per-node cap once per sample instead of walking it differently in each sweep.
- ~~`ConfigManager` silently overwrites `config.yaml` with defaults, destroying any key it does
  not know about~~ — **the diagnosis was wrong, and the real cause is now fixed.** `_load_config`
  does merge correctly: it backfills missing defaults and never deletes unknown keys. The actual
  culprit was `gui_manager.py`, which resolved the config path against the **current working
  directory** (`os.getcwd() + "/../config.yaml"`). That only lands on the real file when the app
  is launched from `src/`; from anywhere else — the repo root, an IDE, a packaged build — it
  pointed *outside* the repo, `ConfigManager` found no file there and wrote a fresh default one.
  The defaults carry no `stall_accel` / `free_speed` / `traction_accel`, so the drivetrain model
  silently switched off, `max_acceleration` dropped to 8.0 and `track_width` to 12.5 — which is
  exactly the reset observed during this work. Now resolved relative to `__file__`, so the launch
  directory no longer matters.

---

## How to re-verify after a change

There is no headless entry point *in the repo*, but one is straightforward to write and is how
every number in this document was measured. `Node` and `ActionPoint` are Qt graphics items, but
the profiler only reads plain attributes off them, so a ~30-line stand-in plus the route-file
JSON is enough to drive `build_path` → `forward_backward_pass` → `generate_motion_profile`
without Qt. The one thing to get right is the coordinate conversion: route files store inches,
and `load_nodes` → `convert_point` → `update_spline` nets out to **feet = inches / 12**.

**Audit on the distance grid, not the resampled 10 ms grid.** The 10 ms profile finite-differences
`ω = v·κ` across genuine curvature discontinuities at spline splits, which manufactures huge
apparent angular accelerations that the drivetrain never has to produce. That artifact is what
produced the bogus "~13% overshoot, it's just discretisation" conclusion this document used to
carry. Check the invariant on the grid the sweeps actually enforce it on.

The properties that should always hold:

- `|v| ≤ max_velocity`, and `|v| + |ω|·track_width/2 ≤ max_velocity` (wheel speeds)
- `accelerations[i] == (linear_vels[i+1] − linear_vels[i]) / dt` — exactly
- `positions` monotonic, ending at the spline's total arc length
- no heading step larger than a few degrees per 10 ms tick
- signed velocity passing smoothly through zero at every reverse node, at the decel limit
- **the outer wheel stays inside its force budget:**
  `|a|·(1 + |κ|·tw/2) + v²·|dκ/ds|·tw/2 ≤ accel_limit(v)` (or `decel_limit(v)` when braking)

The middle two catch reverse-node regressions, and they're the ones that broke during changes
§1–§3.

**The last one is the one that caught §7, and it is the one to keep running.** None of the others
catch it: the velocity is fine, the heading is fine, the wheel speeds are fine, and the
acceleration array is an exact derivative of the velocity. It is the *velocity and acceleration
together*, against the curvature rate, that were infeasible — which is why this bug survived every
check in the original list for as long as it did.

Three things to get right when you check it, all of which I got wrong first:

- **Check each sample against its own state.** Pairing `v` from one sample with `dκ/ds` from its
  neighbour over-reports badly (it is what produced a fictitious "+11.6 ft/s²" before I corrected
  the audit). At sample `i`, use `v[i]`, `κ[i]`, `κ'[i]`, and the acceleration in force there —
  which comes from the interval either side of it.
- **Score each adjacent interval against *its own* budget — accelerating or braking.** Do **not**
  collapse the two intervals into `max(|a[i-1]|, |a[i]|)` and compare that against a single limit.
  Under the drivetrain model `accel_limit` and `decel_limit` diverge sharply (at 3.7 ft/s: 6.93 vs
  11.0, because braking has no `v` term — see §5), so a braking interval charged against the
  *accelerating* budget reports a violation that isn't there. The check is:

  ```python
  for j in (i - 1, i):                    # the two intervals acting on sample i
      a = a_interval[j]
      limit = accel_limit(v[i]) if a >= 0 else decel_limit(v[i])
      over = abs(a) * (1 + |κ[i]|*tw/2) + v[i]**2 * |κ'[i]| * tw/2  -  limit
  ```

  This fires wherever a decel ramp meets a cruise plateau — i.e. any node or action point with a
  `max_velocity` cap. Collapsing the intervals reported **+4.07 ft/s² on `low4ball.txt`** (37% over
  budget, and *not* converging as `dd` shrank — the exact signature of a real bug) when the profile
  was in fact braking legally at −10.94 into a 3.70 ft/s action-point cap. Five routes tripped it.
- **Apply it at *every* sample, including where `κ ≈ 0`** — that is precisely where §6 was
  skipping it.

> **`sawp.txt` alone cannot catch the mistake above.** It has no action points and no per-node
> velocity caps, so it never forms a decel-into-plateau corner. Audit a route that has one —
> `low4ball.txt`, `skills6.txt`, `skills7.2.txt`, `skills8.1.txt`, `low_elim_split.txt` — as well.

On `sawp.txt` today the worst violation is +0.08 ft/s² (0.7% of budget) and converges away as
`dd` shrinks. If you ever see that number jump back into the units, something has regressed.
