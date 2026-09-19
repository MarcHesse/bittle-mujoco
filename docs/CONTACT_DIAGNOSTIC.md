# Contact & Actuation Diagnostic — Design

Status: **implemented and self-checked** (2026-09-19) — `diagnose_contact.py`

## Why

Open-loop gaits (`trF`, `wkF`) look like the feet slide over the floor
instead of stepping, and the simulated Bittle walks worse than the real
one. The same model is used by MH-FLOCKE (`creatures/bittle/bittle.xml`,
synced from this repo), so whatever causes it affects FLOCKE as well.

So far this is an impression from watching the viewer. Before any
parameter in `bittle.xml` is changed, the behaviour has to be measured,
so that every later change can be judged against a number instead of
by eye.

## Goal

One script, `diagnose_contact.py`, that plays a gait open-loop exactly as
`example_trot.py` does and reports per leg and per actuator:

1. how far the feet slide while they are loaded (stance slip)
2. where the model actually touches the floor (which geom, how far from
   the foot site)
3. how well the joints follow their targets (tracking error)
4. how much torque the actuators produce
5. what the body does (forward travel, drift, height, fall)

Output: console summary, a JSON summary for comparing runs, optional
per-step CSV.

## Non-goals

- No change to `bittle.xml`. The script only reads the model; what-if
  experiments use runtime overrides (see below) and never write a file.
- No change to gaits, `gaits.py`, or the examples.
- No closed-loop control. The question is how the *model* behaves under
  a fixed input, not how well a controller can compensate.
- No statement about the real servo's specification. Reference values
  for the P1S (torque, speed) are passed in as parameters and must come
  from the datasheet or a measurement, never from a default in the code.

## Playback (identical to `example_trot.py`)

- Reset to keyframe `stand`, `mj_forward`.
- Gait frames at 50 Hz (`FRAME_DT = 0.020`), amplitude ramp over the
  first 400 physics steps: `ctrl = STAND_CTRL + (gait[i] - STAND_CTRL) * scale`.
- Measurement starts after the ramp plus one full gait cycle (settling)
  and runs for `--cycles` cycles (default 10).
- Fully deterministic: no randomness anywhere, same input gives the same
  numbers.

Scenarios via `--gait`:

| Value | Purpose |
|---|---|
| `trF` | trot, visually verified gait |
| `wkF` | walk, visually verified gait, used by MH-Neuron's Bittle demo |
| `stand` | constant stand control — **control scenario**: slip should be near zero and the vertical contact force should sum to body weight |

## Metrics

### Leg assignment

Every collision geom is assigned to a leg (RF, LF, RR, LR) by walking up
its body chain to the first body whose name contains `rf`, `lf`, `rr` or
`lr`. Geoms of torso, head and battery are assigned to `body`. Only
contacts between the `floor` geom and a non-floor geom are evaluated.

### 1. Stance slip

For each floor contact in each physics step:

- contact point `p` and normal `n` from `data.contact[i]`
- velocity of the material point of the leg at `p`, via
  `mj_jac(model, data, jacp, jacr, p, body)` and `v = jacp @ qvel`
- tangential slip speed `|v - (v·n) n|`
- normal force from `mj_contactForce`

Per leg, over the measurement window:

- stance time (steps with at least one floor contact of that leg)
- slip distance = sum of force-weighted mean slip speed × dt
- mean and 95th-percentile slip speed during stance
- **slip ratio** = slip distance / forward travel of the torso over the
  same window. A foot that really steps has a slip ratio near 0; a foot
  that is dragged along has a ratio near 1.

### 2. Contact location

Per leg:

- share of contact time per touching geom (e.g. 80 % `shank_rf_1`,
  20 % `servos_rf_1`). Contacts on thigh or torso geoms are listed
  explicitly — they should not occur in a clean gait.
- mean and max distance of the contact point from that leg's foot site.
  A large or strongly varying distance means the contact point moves
  along the convex hull of the shank mesh instead of sitting at a tip.

### 3. Tracking

Per actuator: RMS and max of `ctrl - qpos` (joint position), and the lag
in physics steps that maximises the cross-correlation between the two
signals.

### 4. Actuator torque

Per actuator: peak and RMS of `data.actuator_force`. If
`--servo-torque-nm` is given, the share of steps above that value is
reported. Without it, only the raw numbers are printed.

### 5. Body

- forward travel (torso x) per cycle, lateral drift (y), yaw drift
- mean and min torso height, max torso tilt, and a fall flag (tilt above
  60°). A height threshold was used first (0.04 m from
  `test_mass_stand.py`) but flagged every gait as fallen: the torso rides
  about 10 mm lower while walking than while standing.
- sum of vertical contact force vs. `total mass × g` (sanity check)

## Runtime overrides (what-if, no file changes)

Applied to the loaded `MjModel` before the run, printed in the summary
and stored in the JSON so a result can never be mistaken for the
unmodified model:

| Flag | Model field |
|---|---|
| `--damping X` | `dof_damping` of the 8 leg joints |
| `--armature X` | `dof_armature` of the 8 leg joints |
| `--kp X` | `actuator_gainprm[:,0]` and `actuator_biasprm[:,1] = -X` |
| `--forcerange X` | `actuator_forcerange = ±X`, `actuator_forcelimited = 1` |
| `--friction X` | `geom_friction[:,0]` of floor and all collision geoms |

This makes it possible to see which parameter moves which metric before
anything is changed in `bittle.xml`.

## Output

- Console: one table per leg (stance, slip, contact geoms, distance to
  foot site), one per actuator (tracking, torque), body summary, list of
  active overrides.
- `--json PATH`: all summary numbers plus model file hash, MuJoCo
  version, gait, cycles, overrides. Two JSON files can be compared
  directly (baseline vs. change).
- `--csv PATH`: per-step values for plots (optional, large).

## Validation of the diagnostic itself

The script is only useful if its numbers are right. Checks before the
first real result is trusted:

1. `--gait stand`: vertical force sum within 2 % of body weight, slip
   speeds near zero, only foot-region geoms in contact (this matches
   `test_mass_stand.py`, which reports four foot contacts).
2. Deliberate slip: `--friction 0.05` on `trF` must raise the slip ratio
   clearly compared to the default.
3. Determinism: two identical runs produce identical JSON.

## Comparison with the real robot

The simulation numbers need a real counterpart to say "worse than real"
in numbers. Simplest measurement on hardware: forward speed of `trF` and
`wkF` on a known floor (distance over time, from a video with a ruler in
frame). The script reports forward travel per cycle in the same unit.
Floor material should be noted, since friction on wood, tile and carpet
differs.

## After the baseline

Not part of this script, listed so the order is clear:

1. Baseline JSON for `stand`, `trF`, `wkF` with the unmodified model.
2. What-if runs with overrides to find which parameter explains the
   slip.
3. Only then a change in `bittle.xml` (e.g. dedicated foot collision
   geoms), with a CHANGELOG entry in the same style as the head
   collision change, a note that results are not comparable across the
   change, and the 10k regression in MH-FLOCKE after syncing.

Note from the README: joint damping 1.5 was tuned for a *stable*
open-loop trot, not for realism. Changing it may trade stability for
realism; the diagnostic should show that trade-off in numbers.

## Implementation order

1. Playback + body metrics + JSON (runs, no contact analysis yet)
2. Leg assignment + contact location
3. Stance slip
4. Tracking + torque
5. Overrides
6. Validation checks 1–3 above

Each step is run and checked before the next.

## Results 2026-09-19 (model aaea9ba991b043ea, MuJoCo 3.5.0)

Self-check passed (force balance 1.0000, no slip in stand, 3.9x slip at
friction 0.05, deterministic).

### Real robot reference (measured by hand, stand pose `kbalance`)

- shoulder servo horn 90 mm above floor (walking: ~80 mm)
- knee 50 mm above floor
- shoulder horn -> knee: 50 mm
- front foot tip -> rear foot tip: 100 mm

### Findings

1. **Foot sites are misplaced.** In the stand pose every foot site sits
   29 mm straight below the knee axis and 10 mm above the floor; the
   actual floor contact is at the shank tip, 33.7 mm forward, 13 mm
   inward, 10 mm lower. Instrumentation only; `FOOT_SITES` is not used
   in MH-FLOCKE code.
2. **Stand pose is 11 mm too low.** Model: shoulder 79 mm, knee 39 mm
   above floor. The shoulder-to-knee height (40 mm) matches the robot;
   the whole difference is below the knee. A constant **+25 deg offset on
   all knee commands** reproduces 90 / 50 mm exactly (shank then ~21 deg
   from vertical instead of 46 deg). Not yet independently confirmed:
   predicted shank length ~55 mm and shank angle ~21 deg on the robot.
   The conversion constant is `REST_LOWER = -55` in `gaits.py` and
   `rest_knee_deg = -55.0` in MH-FLOCKE `src/body/bittle.py`.
3. **Knee offset improves the trot strongly** (trF, 10 cycles): travel
   729 -> 1148 mm, yaw drift 39.4 -> 0.6 deg, max tilt 19.5 -> 7.2 deg,
   left/right symmetric. Absolute slip unchanged (595 -> 598 mm).
4. **Joint model: damping 1.5 compensates for unlimited servo strength.**
   Lag is 36-38 ms = damping / kp. Peak torques 5-9 Nm. With damping
   0.05 / armature 0.001 and no force limit the robot falls over (tilt
   135 deg). With damping 0.05 / armature 0.001 / forcerange 0.5 Nm and
   knee +25 it trots stably at the best speed seen so far (travel
   1161 mm, yaw 0.3 deg, tilt 8.9 deg, lag 6-12 ms). 1.0 Nm: stable but
   more slip; 0.3 Nm: too weak to trot. With damping 1.5 any force limit
   <= 1 Nm stalls the joints (max joint speed = limit / damping).
5. **Slip is not explained yet.** Not friction (+67 % friction -> -12 %
   slip), not servo strength (realistic force limits do not reduce it),
   not the knee offset. Slip ratio stays around 0.15-0.3 in every stable
   variant. Remaining candidates: contact model / foot geometry (hard
   mesh contact, no rubber compliance), or slip inherent to the
   open-loop gait that the real robot shows as well. Needs a real
   reference before further tuning.
6. **impratio** (not set in `bittle.xml`, default 1; MuJoCo recommends
   Newton + elliptic cone + large impratio against soft-contact slip,
   Menagerie quadrupeds use 100): only ~13 % less slip at 10 or 100 on
   the unmodified model, no effect on drift/yaw. Right setting, small
   lever here.
7. **Slip is load-bearing, not scuffing.** Split at 10 % body weight per
   leg: light-contact slip (touchdown, lift-off, toe drag) is 0.3-4 mm
   per leg over 10 cycles; nearly all slip happens under load. With knee
   +25 and impratio 100: loaded slip ratio 0.09-0.15. The real robot does not
   visibly slip on a hard indoor floor, so a gap remains; its size needs a
   measured real slip value (slow-motion video with ruler).
8. **Compliance helps a little.** The real robot is noticeably wobbly. Softer
   servo (kp 40 -> 5, damping 0.05, armature 0.001, forcerange 0.5):
   loaded slip ratio front 0.15 -> 0.11, rear 0.16-0.17 -> 0.15-0.16.
   Softer foot contact (solref 0.05) makes the rear legs worse (0.22).
9. **Remaining slip is fore-aft, mostly in the rear legs.** kp 5 variant:
   forward component 114-119 mm (front) / 162-173 mm (rear), lateral
   20-33 mm. Body roll is not the main source; front and rear strides do
   not match. Candidates: remaining leg-geometry difference (thigh 46 mm
   in the model vs 50 mm measured), knee offset not identical front and
   rear, or the gait table itself. Accepted as good enough for now: the
   robot has plastic servo gears and some play, and its own gait is not
   ideal either.

### Open

- Robot: shank length (knee horn centre -> foot tip), shank angle in
  `kbalance`, trot speed (distance over time), ideally a slow-motion
  video of the feet to see whether the real feet slide.
- P1S datasheet: stall torque and speed, to replace the 0.5 Nm guess.
- Applied on 2026-09-19: `impratio="100"` in `bittle.xml` (all copies, MH-FLOCKE
  v0.8.2.3). Not yet changed: knee conversion in `gaits.py`, joint model, foot
  sites. Each of these changes the physics: CHANGELOG entry, results not
  comparable across the change, 10k regression in MH-FLOCKE after syncing.
