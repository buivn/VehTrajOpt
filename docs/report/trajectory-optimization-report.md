# Trajectory Optimization on a Differential-Drive Robot — Comparative Report

**Status:** living document — updated as each planner/controller lands.
**Last updated:** 2026-09-19.

Goal: compare **global planners** × **path-tracking controllers** for a differential-
drive robot in a procedurally generated maze, on a common `Path → cmd_vel` interface,
under one simulator (Gazebo Harmonic) and one map.

---

## 1. What we compare

**Planners (global — "which way?")**
- **P1. A\*** — grid search on an inflated occupancy grid, clearance-aware cost. *(done)*
- **P2. Advisor's sampling-based kinodynamic planner** — RRT-family motion planning
  that respects robot dynamics (from `src/GP/*`, wrapped via the Phase-1.5 C++ bridge). *(TODO)*
- **P3. State lattice** — heading-aware search over a precomputed diff-drive motion-
  primitive library (1 straight + 4 arcs + 2 pivots, docs/22), same clearance-cost
  shaping as A\* but primitives respect the robot's turning behavior instead of 8
  fixed grid moves. *(done)*

**Controllers (local — "what wheel commands?")**
- **C1. Pure Pursuit** — geometric tracker + adaptive lookahead, turn-in-place,
  curvature slowdown. *(done)*
- **C2. MPC** — receding-horizon optimal control with constraints; solved via
  CasADi/IPOPT, then re-derived for **acados/SQP real-time-iteration** to bound
  per-tick solve time (docs 27–29). *(done)*
- **C3. MPPI** — sampling-based model-predictive control (GPU rollouts). *(done)*

**Matrix (9 cells):**

| | C1 Pure Pursuit | C2 MPC | C3 MPPI |
|---|---|---|---|
| **P1 A\*** | ✅ measured (§4.1) | ✅ measured, headless, acados (§4.2) | ✅ measured (§4.3) |
| **P2 Sampling (advisor)** | ⬜ | ⬜ | ⬜ |
| **P3 State lattice** | ✅ measured (§4.8) | ✅ measured, acados (§4.8) | ✅ measured (§4.8) |

---

## 2. Test setup

- **Robot:** diffbot, footprint ~0.40×0.30 m, wheel radius 0.08 m, separation 0.34 m,
  `v_max` 0.5 m/s, `ω_max` 1.5 rad/s. Now carries a 360-beam 2D lidar.
- **Environment:** procedural maze, 20×30 m, 1.5 m corridors (recursive backtracker,
  seed 7), 191 wall boxes. Occupancy map 200×300 @ 0.1 m.
- **Sim:** Gazebo Harmonic + ros2_control diff_drive_controller (100 Hz).
- **Localization:** Phase 1.5 = static `map→odom` (no drift correction). **Phase 2 =
  AMCL** (in progress) — needed for long-range validity (see §5).

---

## 3. Metrics

| Metric | Meaning | Where measured |
|--------|---------|----------------|
| **Success rate** | reaches goal without collision | Gazebo ground truth |
| **Time-to-goal** | wall-clock from goal → arrival | — |
| **Path length** | planned path length (m) | planner |
| **Min wall clearance** | closest the *path* comes to a wall (m) | planner + map |
| **Tracking error** | max deviation of robot from path (m) | controller |
| **Plan compute time** | ms per global plan | planner |
| **Control compute time** | ms per control tick | controller |
| **Comfort** | speed vs curvature / jerk | controller |

---

## 4. Results

### 4.1 P1 A\* + C1 Pure Pursuit  *(measured — offline + sim)*

**Planner (A\*), 0.1 m grid, spawn → far corner:**
| quantity | value |
|---|---|
| plan compute time | ~99 ms (clearance precomputed once) |
| path length | 47 pruned waypoints (~30 m) |
| min wall clearance (clearance cost `w=6`, `R=8`) | **0.70 m** (corridor centred) |
| C-space inflation guarantee (robot 0.18 m + 0.10 m margin → 3 cells) | ≥ **0.30 m** from every wall |

**Controller (Pure Pursuit), offline unicycle sim:**
| quantity | value |
|---|---|
| corner max-deviation, textbook fixed-Ld vs upgraded | 0.15 m → **0.06 m** |
| adaptive lookahead | `Ld = clamp(1.0·v, 0.5, 1.0)` m |
| anti-stall floor | `min_linear = 0.12` m/s |

**Benchmark sweep (vto_bench goal tour, 30 legs × 2 repeats, AMCL, GUI matched tour):**
| metric | Pure Pursuit (58/60 reached) |
|---|---|
| success rate | **97 %** (58/60) |
| time-to-goal | 56.8 ± 15.6 s |
| cross-track mean / max | **0.024 / 0.109 m** |
| cross-track rms | 0.031 m |
| len ratio (driven/planned) | 0.956 (barely wanders) |
| comfort dω rms | 0.043 |
| mean \|ω\| | 0.235 rad/s |
| min wall clearance | 0.395 m |
| compute / tick | **0.069 ms** (µs-scale) |

Pure Pursuit is reliable, tight (2.4 cm mean tracking), smooth, and essentially free to
compute. Head-to-head vs MPC in §4.5. (Pure Pursuit is timing-robust — headless and GUI
numbers match, unlike MPC.)

### 4.2 P1 A\* + C2 MPC — headless, acados/SQP_RTI (100/100 reached)

**History of this result (docs/14, 27–29).** The first *headless* MPC sweep failed
30/53 with AMCL diverging (cte up to 15 m). "Reverse-driving causes it" was
**disproven** (identical reverse-enabled MPC ran clean in the GUI); "compute
starvation" was **also disproven** (headless RTF ≈ 1.0, MPC held 20 Hz). A 220-leg
direct-confirmation rerun (docs/14) then found the divergence **did not reproduce**,
downgrading that theory to *rare/unconfirmed* — but it did surface a real, smaller
effect: occasional IPOPT solve-time spikes (up to 271 ms against the 50 ms control
period) that briefly degraded tracking. Root cause: IPOPT's interior-point iteration
count varies with the active constraint set even when warm-started, so its
*worst-case* solve time is unbounded even though its *mean* is fine (docs/27). Fix:
re-derived the same cost/dynamics for **acados' SQP real-time-iteration** solver
(state-augmented control-rate cost, wrapped-heading cost via per-stage parameters,
warm-started inner QP — docs/28–29), and made the ROS control loop itself
non-blocking (background-thread solve, graceful degrade to replaying the last
computed control sequence if a solve ever runs long, hard stop only once that
sequence is exhausted — docs/28).

**Headless benchmark, 100-leg matched seeded tour (`seed=1`), before vs after:**
| metric | IPOPT (`mpc_headless_confirm2`) | **acados/SQP_RTI** |
|---|---|---|
| success rate | 99 % (99/100) | **100 %** (100/100) |
| time-to-goal | 42.5 ± 13.0 s | 42.6 ± 10.5 s |
| cross-track mean / max | 0.025 / 0.124 m | 0.027 / 0.133 m |
| len ratio | 0.963 | 0.964 |
| min wall clearance | 0.395 m | 0.392 m |
| **compute / tick, mean** | 19.2 ± 1.1 ms | **1.96 ± 0.08 ms** (~10×) |
| **compute / tick, worst leg** | 62.5 ms (run max **270.9 ms**) | **4.7 ms** (run max **23.9 ms**, ~11×) |
| **legs with a tick over the 50 ms control period** | **47 / 100** | **0 / 100** |
| comfort dω rms | 0.049 | 0.114 |
| comfort dv rms | 0.021 | 0.032 |

The bug is fixed: zero legs now exceed the control period (was 47/100), with
tracking, success rate, clearance, and time-to-goal statistically unchanged — the
fix didn't trade away effectiveness. The one real cost, found by measuring rather
than assuming: command smoothness got worse (dω rms roughly doubled). Likely cause —
SQP_RTI does exactly one Gauss-Newton step per tick from the current linearization,
versus IPOPT iterating an exact-Hessian NLP to a tight tolerance every tick; RTI
trades solution refinement for a bounded worst case. Accepted for now — there's a
~25× compute margin left (2 ms of a 50 ms budget) to spend on more SQP iterations per
tick if the smoothness needs recovering later.

### 4.3 P1 A\* + C3 MPPI — measured (headless, clean)
| metric | MPPI (59/60 reached) |
|---|---|
| success rate | **98 %** (59/60) |
| time-to-goal | 45.3 ± 17.5 s |
| cross-track mean / max | 0.035 / 0.156 m |
| len ratio | 0.942 |
| mean \|ω\| | 0.359 rad/s |
| min wall clearance | 0.378 m |
| compute / tick | **3.6 ms** (spike-free, std 0.04) |

Sampling-based control (K=1000 numpy rollouts). Slightly looser tracking than MPC/PP
(3.5 cm — sampling noise). Notably **ran clean headless** (max pose jump 0.06 m) at a
time when MPC's headless runs were still under investigation — see docs/14 §6, and
§4.4 for how this compute/accuracy comparison against MPC looks now that MPC uses
acados (spoiler: MPPI is no longer the cheaper of the two).

### 4.4 Headless matched three-way: IPOPT-MPC vs acados-MPC vs MPPI
Same seeded tour (`seed=1`), all three **headless**, first 60 legs (MPPI's run length,
§4.3) so all three are directly comparable. **Bold = best.**

| metric | IPOPT-MPC | acados-MPC | MPPI |
|--------|-----------|------------|------|
| success | 98 % (59/60) | **100 %** (60/60) | 98 % (59/60) |
| time-to-goal (s) | 42.1 ± 14.3 | **41.9 ± 10.7** | 45.3 ± 17.6 |
| cross-track mean (m) | **0.025** | 0.027 | 0.035 |
| cross-track max (m) | **0.120** | 0.136 | 0.156 |
| len ratio | 0.964 | **0.966** | 0.942 |
| dω rms (jerk) | **0.048** | 0.109 | 0.265 |
| min clearance (m) | 0.390 | **0.392** | 0.378 |
| **compute mean (ms)** | 19.1 | **1.97** | 3.61 |
| **compute worst-tick (ms)** | 270.9 | **23.9** | 11.0 |

**Reading — this changes the project's original story (§6).** With IPOPT, the
trade-off documented below (§4.6) held: MPC = accurate-but-heavy, MPPI =
cheap-but-looser. With **acados, MPC is no longer the expensive option** — its mean
compute (1.97 ms) is now *below* MPPI's (3.61 ms), its worst case is bounded and
comparable, and it still tracks tighter (2.7 cm vs 3.5 cm mean) and is smoother on
the angular channel (0.109 vs 0.265 dω rms) than MPPI. MPPI's only clear win here is
`dv_rms` (linear-command smoothness) and simplicity of implementation (no solver
toolchain). The classic "accuracy vs. compute" spectrum was a property of *IPOPT
MPC specifically*, not of gradient-based MPC in general.

### 4.5 P2 Sampling planner + {C1,C2,C3} — ⬜ TODO

### 4.6 Three-way (A\* planner; PP+MPC in GUI, MPPI headless-clean) — historical, IPOPT-era
> Pure Pursuit here is GUI/30-leg; MPC is the old IPOPT solver; see §4.4 for the
> current all-headless, acados comparison. Kept for the Pure-Pursuit comparison point,
> which §4.4 doesn't include.

| metric | Pure Pursuit | MPC | MPPI |
|--------|-------------|-----|------|
| success | 58/60 | 59/60 | 59/60 |
| time-to-goal (s) | 56.8 | **40.0** | 45.3 |
| cross-track mean (m) | **0.024** | **0.024** | 0.035 |
| cross-track max (m) | **0.109** | 0.168 | 0.156 |
| min clearance (m) | **0.395** | 0.366 | 0.378 |
| compute/tick (ms) | **0.069** | 22.65 | 3.6 |

**Reading:** three points on a spectrum, **as of the IPOPT-solver era** (see §4.4 for
how the MPC point moves once acados is used).
- **Pure Pursuit** — cheapest (µs), tightest peaks, most clearance, smoothest; **slowest**.
- **MPC (IPOPT)** — **fastest** (preview), tight mean tracking; **heaviest + spiky compute**.
- **MPPI** — nearly MPC's speed with **~6× less, spike-free compute** and no gradient
  solver; pays with **looser tracking** (sampling noise). A practical middle ground
  *between Pure Pursuit and IPOPT-MPC* — no longer between Pure Pursuit and MPC in
  general, now that acados-MPC undercuts MPPI's compute too (§4.4).

### 4.7 Head-to-head: Pure Pursuit vs MPC (matched GUI tour, 58 legs both reached)
Same map, same seed-1 goal tour, both in the GUI, **IPOPT-solver era** (see §4.4 for
the current headless/acados numbers for MPC). **Bold = winner.**

| metric | Pure Pursuit | MPC |
|--------|-------------|-----|
| success | 58/60 | **59/60** |
| **time-to-goal** | 56.8 ± 15.6 s | **40.0 ± 10.5 s  (−30 %)** |
| cross-track mean | 0.024 m | 0.024 m *(tie)* |
| cross-track max | **0.109 m** | 0.168 m |
| cross-track rms | **0.031 m** | 0.037 m |
| len ratio | 0.956 | 0.966 *(tie)* |
| comfort dω rms | **0.043** | 0.048 |
| angular effort mean·\|ω\| | **0.235** | 0.350 |
| min wall clearance | **0.395 m** | 0.366 m |
| compute / tick | **0.069 ms** | 22.65 ms *(~330×)* |

**Reading:** the trade-off is **speed-and-preview vs. precision-and-economy**, *not*
"MPC tracks tighter" (mean tracking is a tie at 2.4 cm).
- **MPC wins time-to-goal by 30 %** — its N-step preview lets it **carry speed through
  corners**, where Pure Pursuit slows and pivots.
- **Pure Pursuit wins peak tracking, clearance, smoothness, angular effort, and compute
  (~330×)** — MPC buys its speed by cornering more aggressively (higher peak deviation,
  closer to walls, more steering).
- Both ≈98 % success and `len_ratio ≈ 0.96` (neither wanders).

### 4.8 P3 State Lattice + {C1, C2, C3} — measured (headless, seed=1, 60 legs)

Same seeded goal tour used for A*'s three-way (§4.4: `seed=1`, 60 legs), run
against the **state-lattice planner** (docs/22, diff-drive) instead of A* —
the tour is planner-independent (built from `/map` + `seed` only, verified
in `benchmark_node.py`), so this is directly comparable leg-for-leg against
A*, not just controller-for-controller (§4.9). Run immediately after fixing
a lattice goal-tolerance bug (docs/30): the search accepts any state within
a tolerance radius of the goal rather than an exact hit (an arbitrary
continuous goal generally isn't reachable by an exact combination of fixed
primitives), and without a docking fix that left 3/5 smoke-test legs
stopped just short of the benchmark's own separate goal tolerance. Verified
fixed (docs/30) before running this at full scale.

| metric | Lattice+PP (60/60) | Lattice+MPC-acados (60/60) | Lattice+MPPI (59/60) |
|---|---|---|---|
| success rate | 100 % | 100 % | 98.3 % |
| time-to-goal (s) | 62.3 ± 15.9 | 43.8 ± 10.9 | 46.8 ± 12.0 |
| cross-track mean / max (m) | 0.027 / 0.200 | 0.028 / 0.261 | 0.039 / 0.528 |
| len ratio | 0.970 | 0.980 | 0.953 |
| dω rms | 0.058 | 0.113 | 0.271 |
| min wall clearance (m) | 0.20 | 0.10 | 0.20 |
| compute/tick mean (ms) | 0.19 | 2.06 | 3.82 |
| compute/tick worst (ms) | 2.4 | 7.5 | 20.1 |

**Reading:** same shape as A*'s own controller comparison (§4.4/§4.7) —
MPC-acados fastest to goal, Pure Pursuit cheapest/smoothest compute, MPPI
the noisiest on this tour (dω rms 0.271, cross-track max 0.528 m). All
three controllers reach ≥98% success on the lattice planner, matching their
A* numbers — the docs/30 fix closed the gap that mattered for reliability.

### 4.9 P1 A\* vs P3 State Lattice — same seeded tour, per controller

Both planners ran on the *identical* `seed=1`, 60-leg tour, isolating the
planner's effect from the controller's: everything below holds the
controller fixed and swaps only the global planner. (Pure Pursuit excluded:
A*'s only PP baseline is a GUI/30-leg run, §4.1 — not the same scale/mode as
this headless/60-leg tour. A fair PP comparison needs a matching headless
A*+PP run, not yet done.)

| metric | A\*+MPC-acados (§4.4) | Lattice+MPC-acados | A\*+MPPI (§4.3) | Lattice+MPPI |
|---|---|---|---|---|
| success | 100 % | 100 % | 98 % | 98.3 % |
| time-to-goal (s) | 41.9 ± 10.7 | 43.8 ± 10.9 | 45.3 ± 17.5 | 46.8 ± 12.0 |
| cross-track mean (m) | 0.027 | 0.028 | 0.035 | 0.039 |
| cross-track max (m) | **0.136** | 0.261 | **0.156** | 0.528 |
| len ratio | 0.966 | **0.980** | 0.942 | **0.953** |
| dω rms | **0.109** | 0.113 | **0.265** | 0.271 |
| min wall clearance (m) | **0.392** | 0.10 | **0.378** | 0.20 |
| compute/tick mean (ms) | 1.97 | 2.06 | 3.61 | 3.82 |

**Reading:** the planner swap is close to a wash on the metrics that matter
most for whether the robot gets there well — success rate, time-to-goal,
mean cross-track error, and comfort (dω rms) are all within noise between
planners for both controllers. Two metrics move consistently, in opposite
directions:
- **Len ratio favors lattice** (0.953–0.980 vs A*'s 0.942–0.966, closer to
  1.0 = driven length tracks planned length more closely) — plausible since
  lattice primitives are curvature-continuous by construction, giving the
  tracker less to correct for versus A*'s piecewise-straight, corner-pruned
  grid path.
- **Worst-case wall clearance favors A\*, clearly** (0.378–0.392 m vs
  lattice's 0.10–0.20 m, roughly 2–4× tighter). Checked whether this traces
  to the docs/30 docking hop (the one segment of a lattice path that isn't
  clearance-cost-shaped or collision-checked) — it doesn't: in every leg
  checked, the worst-clearance tick lands well before the goal region, not
  on the final segment. More likely explanation, not yet confirmed: the
  lattice's fixed primitive library (2 curvature levels only, docs/22) can't
  shape an arc through some corners as precisely as A*'s per-cell search
  can, so it occasionally threads a turn closer to the wall despite using
  the identical clearance-cost penalty. No single repeated trouble spot
  found (a different leg is worst-case for each controller) — reads as a
  property of the primitive set's coarseness on tight turns generally, not
  one bad map location. Worth a closer look (e.g. a denser or
  tighter-radius primitive set) before trusting lattice paths in a
  tighter-than-1.5 m corridor.

---

## 5. Known limitations

- **Odometry drift — FIXED (Phase 2, AMCL).** A static `map→odom` let wheel-odometry
  drift accumulate; AMCL now publishes a live `map→odom`, and the planner + follower run
  in the `map` frame via TF, so a ~100 m drive stays aligned. This is what makes the
  benchmark possible.
- **Headless MPC divergence — RESOLVED (docs/14, 27–29).** The original 30/53 headless
  failure never reproduced over 220 confirmation legs, downgrading "solve spikes cause
  AMCL divergence" to rare/unconfirmed — but a real, smaller effect was found (spikes up
  to 271 ms briefly degrading tracking). Root cause: IPOPT's iteration count isn't
  bounded under active-set changes even when warm-started. Fixed by re-deriving MPC for
  acados' SQP real-time-iteration solver (bounded per-tick solve) plus a non-blocking
  ROS control loop (§4.2). Headless MPC is now 100/100 reached with zero control-period
  violations (was 47/100) — headless numbers are trustworthy again.
- Pure Pursuit corner-cutting is mitigated (inflation + turn-in-place), not eliminated.
- No replanning yet if the robot leaves the path (once diverged, no recovery).
- **Lattice goal-tolerance gap — FIXED (docs/30).** The lattice search accepts any
  state within a tolerance radius of the goal (not an exact hit), which — before a
  docking-hop fix — left the final waypoint short of the true goal by more than pure
  pursuit's own separate stop tolerance, causing 3/5 smoke-test legs to time out.
  Verified fixed at full scale (§4.8): 60/60 (PP), 60/60 (MPC-acados), 59/60 (MPPI).
- **Lattice worst-case wall clearance is tighter than A\*'s** (§4.9: 0.10–0.20 m vs
  0.378–0.392 m on the same tour) on at least one leg per controller — not traced to
  a specific cause yet (ruled out the docking hop); likely the primitive library's
  limited curvature options on tight turns. Didn't cause a collision or failure in
  60 legs × 3 controllers, but worth tightening before trusting lattice paths in
  corridors narrower than this maze's 1.5 m.

---

## 6. Interpretation (to expand as data lands)

- **Pure Pursuit vs MPC, IPOPT era (measured, §4.7):** the expected story ("MPC tracks
  tighter") did **not** hold — mean cross-track is a **tie (2.4 cm)**. The real result
  was a trade-off: MPC **30 % faster** (preview lets it carry speed through corners)
  while Pure Pursuit kept **tighter peaks, more clearance, smoother steering, and ~330×
  less compute**. *Speed-and-preview vs. precision-and-economy.*
- **MPC vs MPPI — the trade-off changed with the solver (measured, §4.4).** Under
  IPOPT, MPC was the accurate-but-heavy option and MPPI the cheap-but-looser one
  (§4.6). Under **acados/SQP_RTI, that's no longer true**: MPC's mean compute (1.97 ms)
  dropped *below* MPPI's (3.61 ms) while its tracking stayed tighter and its worst-case
  compute stayed bounded — the classic spectrum was a property of *that specific
  solver*, not of gradient-based MPC vs. sampling-based MPPI in general. The lesson
  generalizes: before concluding "method A trades accuracy for cost vs. method B,"
  check whether the *implementation* of A is actually solver-optimal — a bounded,
  RTI-style solver can dominate a sampling method on both axes at once.
- **A\* vs sampling (next):** A\* is optimal on the grid and ideal for this 2D maze; the
  advisor's sampling planner earns its cost in higher-dimensional / kinodynamic settings
  (the car-like model) — that comparison will make *when each wins* concrete.
- **A\* vs state lattice — a near-wash on this map, with one real trade (measured,
  §4.9).** Matching A*'s own claim in §4.6 that "three points on a spectrum" analyses
  can hide where the real difference is: here, success rate, time-to-goal, mean
  tracking, and comfort are all within noise of each other across two controllers on
  an identical seeded tour — the heading-aware primitive search doesn't measurably
  help *or* hurt the outcomes that matter most in this particular maze. What actually
  moved: lattice's driven-vs-planned length ratio is consistently closer to 1.0 (its
  primitives are curvature-continuous, A*'s grid path isn't), and its worst-case wall
  clearance is consistently 2–4× tighter (not yet root-caused, see §4.9). Net: state
  lattice isn't obviously *better* than A\* for this diff-drive robot in this 2D maze —
  its real selling point (docs/22, docs/26) is respecting kinematic constraints for
  robots A\* can't plan for directly (e.g. the Ackermann model, docs 24–25/31–34),
  not raw path quality here where A\* already fits the robot well via turn-in-place.

---

## 7. Reproduce

```bash
# maze + map already generated (vto_simulation/maps/maze.{pgm,yaml})
ros2 launch vto_bringup maze_astar.launch.py                     # A* + Pure Pursuit + RViz
ros2 launch vto_bringup maze_astar.launch.py controller:=mpc     # A* + MPC (acados/SQP_RTI)

# benchmark (goal tour, 7 metrics -> results/<label>_summary.csv); see docs/14
ros2 launch vto_bench bench.launch.py controller:=pursuit num_legs:=30 repeats:=2
ros2 launch vto_bench bench.launch.py controller:=mpc     num_legs:=30 repeats:=2
ros2 launch vto_bench bench.launch.py controller:=mppi    num_legs:=30 repeats:=2

# headless matched 100-leg MPC comparison (§4.2), same seed -> directly comparable runs:
ros2 launch vto_bench bench.launch.py controller:=mpc num_legs:=100 repeats:=1 seed:=1 label:=mpc_acados_confirm

# state lattice (§4.8-4.9), same seed=1/60-leg tour as A*'s three-way (§4.4) for a
# direct planner-vs-planner comparison:
ros2 launch vto_bench bench.launch.py planner:=lattice controller:=pursuit num_legs:=60 repeats:=1 seed:=1 label:=lattice_pursuit_confirm
ros2 launch vto_bench bench.launch.py planner:=lattice controller:=mpc     num_legs:=60 repeats:=1 seed:=1 label:=lattice_mpc_acados_confirm
ros2 launch vto_bench bench.launch.py planner:=lattice controller:=mppi   num_legs:=60 repeats:=1 seed:=1 label:=lattice_mppi_confirm

# offline unit tests:
python3 ros2_ws/src/vto_planning/test/test_astar.py
python3 ros2_ws/src/vto_control/test/test_pursuit_core.py
python3 ros2_ws/src/vto_bench/test/test_metrics.py
docker compose -f docker/docker-compose.yml run --rm vto bash -lc \
  'python3 ros2_ws/src/vto_control/test/test_mpc_core_acados.py'    # acados MPC core (needs acados)
```

---

## 8. References

*(Foundational works for the methods used; verify exact venue/year before formal citation.)*

**Global planning**
- P. E. Hart, N. J. Nilsson, B. Raphael, "A Formal Basis for the Heuristic Determination
  of Minimum Cost Paths," *IEEE Trans. Systems Science and Cybernetics*, 1968. **(A\*)**
- D. Dolgov, S. Thrun, et al., "Path Planning for Autonomous Vehicles in Unknown
  Semi-structured Environments," *IJRR*, 2010. **(Hybrid A\*)**
- T. Lozano-Pérez, "Spatial Planning: A Configuration Space Approach," *IEEE Trans.
  Computers*, 1983. **(C-space inflation)**
- S. M. LaValle, "Rapidly-Exploring Random Trees: A New Tool for Path Planning," TR, 1998;
  S. Karaman, E. Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning" (RRT\*),
  *IJRR*, 2011. **(sampling planners — advisor's approach)**

**Path-tracking control**
- R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm,"
  CMU-RI-TR-92-01, 1992. **(Pure Pursuit)**
- J. B. Rawlings, D. Q. Mayne, M. Diehl, *Model Predictive Control: Theory, Computation,
  and Design*, 2017. **(MPC)**
- J. Andersson et al., "CasADi: a software framework for nonlinear optimization and
  optimal control," *Math. Prog. Computation*, 2019; A. Wächter, L. Biegler, "On the
  Implementation of an Interior-Point Filter Line-Search Algorithm…" (IPOPT), *Math.
  Programming*, 2006. **(original MPC solver stack)**
- M. Diehl, H. G. Bock, J. P. Schlöder, "A Real-Time Iteration Scheme for Nonlinear
  Optimization in Optimal Feedback Control," *SIAM J. Control Optim.*, 2005;
  R. Verschueren et al., "acados: a modular open-source framework for fast embedded
  optimal control," *Math. Prog. Computation*, 2021; G. Frison, M. Diehl, "HPIPM: a
  high-performance QP framework for MPC," *IFAC World Congress*, 2020.
  **(current MPC solver stack — SQP_RTI, docs 27–29)**
- G. Williams, A. Aldrich, E. Theodorou, "Model Predictive Path Integral Control: From
  Theory to Parallel Computation," *JGCD*, 2017; G. Williams et al., "Information-Theoretic
  MPC…," *IEEE T-RO*, 2018. **(MPPI)**

**Localization / SLAM**
- S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, 2005. **(MCL, motion/measurement
  models, likelihood field — the core text)**
- F. Dellaert, D. Fox, W. Burgard, S. Thrun, "Monte Carlo Localization for Mobile Robots,"
  *ICRA*, 1999; D. Fox, "Adapting the Sample Size in Particle Filters Through
  KLD-Sampling," *IJRR*, 2003. **(MCL + the "adaptive" in AMCL)**
- M. Montemerlo, S. Thrun, et al., "FastSLAM…," *AAAI*, 2002; G. Grisetti, C. Stachniss,
  W. Burgard, "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle
  Filters," *IEEE T-RO*, 2007. **(FastSLAM / gmapping)**
- W. Hess et al., "Real-Time Loop Closure in 2D LIDAR SLAM," *ICRA*, 2016 (Cartographer);
  S. Macenski, I. Jambrecic, "SLAM Toolbox," *JOSS*, 2021. **(graph SLAM)**

**System**
- S. Macenski et al., "The Marathon 2: A Navigation System" (Nav2), *IROS*, 2020.
