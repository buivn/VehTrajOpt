# 21 — DWA (Dynamic Window Approach) for diff-drive

> **Status: ruled out, not implemented.** See §4 for why. Kept as-is — the theory
> and design were sound, the decision not to build it is itself the useful
> record, and it's the kind of trade-off worth being able to explain in an
> interview.

Doc 20 named the target as "M1 + lattice/DWA + collocation" in one breath — splitting
that into one doc per method, per the usual pace: **this doc is DWA only** (theory +
method selection + design). Lattice-based generation and collocation optimization
become docs 22/23. No code yet — checking design before implementing, as usual.

---

## 1. General idea / theoretical aspect

DWA is a **reactive, receding-horizon local planner**: every control tick, it
searches directly in **velocity space** `(v, ω)` rather than in **position space**
`(x, y)`. Concretely, each tick:

1. Compute the **dynamic window** — the `(v, ω)` pairs actually reachable one
   control period from now, given the robot's *current* velocity and its
   acceleration limits.
2. For each candidate `(v, ω)` in that window, **forward-simulate** a short
   constant-velocity arc (~1-3 s).
3. **Score** each arc (goal progress, obstacle clearance, speed) and execute the
   `(v, ω)` of the best-scoring one for one tick.
4. Repeat next tick with the new measured velocity.

Where this sits in the pipeline: A\*→Pure Pursuit/MPC/MPPI in this repo is a clean
**global generation → local tracking** split — the global path is already
collision-free, the controller's only job is to follow it. 
DWA **collapses generation and control into one step**: it doesn't consume a pre-vetted path, it
re-derives a short local trajectory and picks a control from it, every tick, while
looking at obstacles itself. That's the conceptual line to hold onto for the
comparison in doc 20 §2: DWA is being tested here as a **generation+control** method,
not a pure generator that then needs a separate tracker.

---

## 2. Method selection — why DWA, why now, why not the alternatives

**Why DWA is the right thing to build next:** it's the only cell in doc 20's matrix
that's a clean **direct fit** for M1 *and* self-contained (doesn't need pairing with
a separate optimizer to produce a usable pipeline) — Fox, Burgard & Thrun's original
1997 formulation is literally built around a holonomic-in-heading robot with
independent `(v, ω)` limits, which is exactly M1.

**Why not Hybrid A\*/lattice for this doc:** those are *global* graph searches over a
discretized state space — correct but too expensive to re-run as a 10-20 Hz reactive
loop, and M1 doesn't even need Hybrid A\*'s bounded-turning-radius primitives (per
doc 20 §1, plain lattice is the better fit for M1 anyway). That's a generator that
needs pairing with an existing tracker (PP/MPC) — different shape of pipeline,
covered in doc 22.

**Why not just reuse MPC or MPPI, which already exist:** they're already being
benchmarked, and DWA's value here is being a **genuinely different algorithm**, not
another example of the same idea:

|              | DWA | MPPI (doc 15) | MPC (doc 13)  |
|--------------|-----|---------------|---------------|
| Search space | discrete grid over `(v, ω)` window | many random control-sequence rollouts | continuous, gradient-based NLP |
| Horizon per rollout | short (~1-3 s), constant `(v, ω)` arc | longer, time-varying stochastic sequence | longer, optimized sequence |
| How the best action is chosen | max of a weighted-sum score | softmax-weighted average over rollouts | solver converges to a local optimum |
| Needs a solver? | no | no (just rollouts + weighting) | yes (NLP/QP) |
| Obstacle-aware? | yes, built into the score | only if you add a cost term | only if you add a cost term |

The DWA vs. MPPI row is worth sitting with: both sample and forward-simulate, but
DWA restricts sampling to the **kinematically-reachable window** and scores with a
simple weighted sum, while MPPI samples more broadly around a nominal sequence and
combines rollouts via softmax rather than taking a single arg-max. Different enough
to be a real second data point, not a rename of MPPI.

**Why not the `src/GP` sampling planner (`MPTreePlanner`/`FollowPlanner`):** built
around `MotionEqs` for the second-order car/trailer model (`acc`, `steer_vel`
states); porting it to M1 means writing a new, simpler `MotionEqs` subclass — real
work, and it's a global tree search, not a reactive local one. Separate step (also
noted as "Adapt" in doc 20 §1), not this doc.

---

## 3. Coding — design

### New files, following the repo's existing split
Every controller in `vto_control` is split into a **framework-free core** (pure
Python/numpy, unit-testable, no ROS imports) and a **thin ROS node adapter** — e.g.
`mpc_core.py` / `mpc_controller_node.py`
([mpc_core.py:22](../ros2_ws/src/vto_control/vto_control/mpc_core.py#L22),
[mpc_controller_node.py](../ros2_ws/src/vto_control/vto_control/mpc_controller_node.py)),
same for `mppi_core.py`/`mppi_controller_node.py` and
`pursuit_core.py`/`pure_pursuit_node.py`. DWA follows the same pattern:

- **`vto_control/dwa_core.py`** — `DWACore` class, pure computation, testable with
  `test/test_dwa_core.py` (mirrors `test_mpc_core.py`).
- **`vto_control/dwa_controller_node.py`** — ROS adapter: subscriptions, timer,
  publishes `TwistStamped` on `/diff_drive_controller/cmd_vel` (same topic
  convention as the existing controllers, per the Jazzy gotcha in `CLAUDE.md`).

**Why a class (`DWACore`) and not a free function**, matching `MPCCore`: the dynamic
window depends on *state carried across ticks* (current measured velocity), and the
tunable weights/limits (accel bounds, scoring weights) are configuration that's
naturally instance data — a function would need all of that re-passed every call.

### Data structures and why

- **Velocity samples — a small fixed-resolution grid over `(v, ω)`, regenerated
  every tick.** Not a persistent structure: the dynamic window's bounds change every
  tick (they depend on current measured velocity), so there's nothing to cache
  between ticks. A `numpy.meshgrid` over e.g. 7×13 samples is enough — DWA's own
  literature uses windows this small because the *scoring*, not the sample count,
  does the work.
  - Window = intersection of absolute limits `[v_min,v_max]×[w_min,w_max]` and the
    **reachable set** given current `(v_meas, ω_meas)` and one control period's
    acceleration limits: `[v_meas - a_max·dt, v_meas + a_max·dt] × [ω_meas -
    α_max·dt, ω_meas + α_max·dt]`. This reachable-set intersection is *the* defining
    idea of DWA — it's what guarantees every candidate is actually executable next
    tick, unlike naively sampling the full `[v_min,v_max]` range.
- **Per-sample rollout — closed-form arc, not stepwise integration.** Unlike
  `MPSnakeSimulator::SimulateOneStep` (RK4 over `MotionEqs`,
  [MPStandardSimulator.cpp:40-58](../src/GP/MPStandardSimulator.cpp#L40-L58)) or
  MPC/MPPI's stepwise rollouts, a **constant** `(v, ω)` unicycle trajectory has an
  exact closed-form pose at time `t` (arc of radius `v/ω`, or a straight line if
  `ω≈0`). No integration loop needed — cheaper per sample than any of the other
  three pipelines.
- **Costmap — reuse, don't reinvent.** None of the existing controllers subscribe to
  an occupancy grid at all (`mpc_controller_node.py` has no `OccupancyGrid`
  subscription) — they trust the global A\* path is already collision-free. DWA is
  the first controller in this repo that needs to judge obstacle proximity itself,
  since it isn't just tracking a pre-vetted path. Reuse, not reinvent: the
  `OccupancyGrid` subscription + world↔cell conversion already exists in
  `benchmark_node.py`
  ([benchmark_node.py:112-118](../ros2_ws/src/vto_bench/vto_bench/benchmark_node.py#L112-L118)),
  and the inflate-by-robot-radius step already exists from doc 10's C-space
  inflation. Store as a 2D numpy array; clearance lookup per rollout point is O(1).
- **Scoring — weighted sum, classic DWA objective.** `heading` (alignment to a local
  target), `clearance` (min distance-to-obstacle along the rolled-out arc), `speed`
  (prefer faster, to avoid a degenerate "always brake" optimum). New tunable
  weights, same role as MPC's `q_pos`/`q_theta`/`r_v`/`r_w`.

### Interface — goal-seeking with a coarse global hint, not pure reactive

Pure goal-seeking DWA is well known to get stuck in box-canyon local minima — a real
risk in a maze of dead-end corridors. Design: give it the **next lookahead point
along the existing A\* path** as its local target (reusing
`pursuit_core.py`'s lookahead logic, doc 05/10) rather than the final goal directly
— cheap way to inherit the global planner's guarantee of a feasible route while
still letting DWA do the actual local, obstacle-aware velocity selection.

### Complexity / real-time

~7×13 = 91 velocity samples × ~15 rollout steps (1.5 s horizon at dt=0.1) × O(1)
costmap lookup ≈ ~1400 cheap ops per tick — trivially real-time at 10-20 Hz, **no
solver**. Worth contrasting directly against MPC's NLP solve time
(`compute_ms_mean`/`compute_ms_max`, already in `vto_bench`'s metrics) once this is
benchmarked — that comparison is a good interview data point on its own.

---

## Interview angle

- **Q: What is the "dynamic window" and why does it matter?** It's the intersection
  of the robot's absolute velocity limits with the velocities *reachable* from the
  current speed given one control period's worth of acceleration. Sampling outside
  it would pick a `(v, ω)` the robot physically can't be at next tick — the window
  is what makes DWA's output always dynamically executable.
- **Q: DWA vs. MPPI — aren't they both "sample and score"?** Both forward-simulate
  candidates, but DWA samples directly in the *reachable velocity window* with a
  short constant-arc rollout and picks the arg-max of a weighted-sum score; MPPI
  perturbs a nominal control *sequence* over a longer horizon and combines rollouts
  via a softmax weighting rather than picking a single best one. DWA is cheaper and
  more myopic; MPPI handles longer horizons and general (nonlinear) dynamics better.
- **Q: Why does DWA need a costmap when this repo's MPC/Pure Pursuit don't?**
  MPC/PP trust that the upstream global planner already produced a collision-free
  path and only optimize tracking of it. DWA isn't handed a vetted path — it must
  judge obstacle proximity itself as part of choosing a velocity.
- **Q: Known failure mode of DWA?** Local minima / box canyons — since it only
  reasons over a short horizon, it can drive itself into a dead end that looks fine
  locally. Standard mitigation: pair it with a global path or waypoint sequence
  (as this design does) rather than running it purely goal-seeking.

---

## 4. Q&A — how the sampling actually works, and the local-minima problem

Two follow-up questions came up while reviewing this design, worth keeping attached
to the doc since they're what led to the decision in §5.

### How DWA samples `(v, ω)`

Not random sampling — three constraint sets intersected, then laid out on a uniform
grid:

1. **Absolute limits `Vs`** — the robot's hard bounds: `[v_min, v_max] × [ω_min,
   ω_max]` (forward-only convention here, like MPC's `v_min=0.0`).
2. **Dynamic window `Vd`** — velocities reachable *one control period from now*,
   given the current measured `(v₀, ω₀)` and acceleration limits:
   `[v₀ - a_max·dt, v₀ + a_max·dt] × [ω₀ - α_max·dt, ω₀ + α_max·dt]`. This is the
   same idea as `mpc_controller_node.py`'s `dv_max=0.15`/`dw_max=0.30` — this repo
   already has an equivalent rate-limit bound, DWA would just reuse it.
3. **Admissible set `Va`** (classic Fox/Burgard/Thrun, *not* in §3's design above) —
   a candidate is only admissible if the robot could still brake to a stop before
   the nearest obstacle on that arc: `v ≤ √(2·dist_to_obstacle·a_brake)`. §3 folded
   obstacle-awareness into the *score* (clearance term) instead of this hard filter
   — simpler to implement, but a genuine gap versus textbook DWA: a soft cost can
   still pick a too-close arc if nothing else scores higher, where a hard `Va`
   filter never would.

Final search space is `Vs ∩ Vd (∩ Va)`, discretized as a uniform `n_v × n_ω` grid
(the 7×13 in §3) — deterministic, not random, which matters for reproducible
benchmarking.

### Without global guidance, does DWA get stuck?

Yes — this is DWA's best-documented weakness, and this maze is exactly the
environment that triggers it:

- The horizon is short (1-3 s ≈ 1-2 corridor segments) — DWA has **no knowledge of
  maze topology** beyond that.
- If the straight-line-to-goal heading points into a dead-end corridor, every step
  *looks* like local progress (heading score improving, clearance fine) — it drives
  all the way in.
- Once jammed at the dead-end wall, the heading term still pulls toward the goal
  (behind the wall) while clearance pushes back — vanilla DWA has no memory of
  "already tried this," so it stalls or oscillates rather than escaping.
- The maze generator (`generate_maze.py`, recursive-backtracker) produces exactly
  this dead-end/box-canyon topology throughout.

The mitigation proposed in §3 (target = next lookahead point along the A\* path,
not the true goal) does fix this — but at a cost, which is what §5 is about.

---

## 5. Decision: not proceeding with DWA for M1

Two independent reasons converge here:

1. **The mitigation removes DWA's distinctness as a pipeline.** Once DWA needs the
   A\* path just to pick its local target, it's no longer a self-contained
   "generation+control" method (§1's whole reason for building it first) — it
   becomes another *tracker* sitting downstream of A\*, architecturally the same
   shape as Pure Pursuit/MPC/MPPI, just with a different local optimization scheme.
   That's a much weaker comparison point for doc 20's matrix than originally framed.
2. **Un-mitigated (pure reactive) DWA will fail routinely in this environment.**
   Per §4, a recursive-backtracker maze is close to a worst case for a short-horizon
   reactive planner with no topology awareness — this isn't a tuning problem, it's
   structural.

Net: skip DWA as an M1 pipeline. `dwa_core.py` will not be built. The theory and
design in §1-§3 stay as the record of *why* it was considered and *why* it was
ruled out — a legitimate trade-off call, not a mistake, and a good interview answer
in its own right ("I considered DWA, but its core assumption — short-horizon
reactive planning with no global topology awareness — doesn't hold in a
maze-dense environment, so I ruled it out rather than force a mitigation that
defeats its own purpose").

**Next:** doc 22 picks up the lattice-based generator for M1 (doc 20 §1), designed
from the start as a *generator paired with an existing tracker* (PP/MPC/MPPI)
rather than aiming for DWA's collapsed generation+control shape.
