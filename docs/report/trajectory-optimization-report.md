# Trajectory Optimization on a Differential-Drive Robot — Comparative Report

**Status:** living document — updated as each planner/controller lands.
**Last updated:** 2026-07-25.

Goal: compare **global planners** × **path-tracking controllers** for a differential-
drive robot in a procedurally generated maze, on a common `Path → cmd_vel` interface,
under one simulator (Gazebo Harmonic) and one map.

---

## 1. What we compare

**Planners (global — "which way?")**
- **P1. A\*** — grid search on an inflated occupancy grid, clearance-aware cost. *(done)*
- **P2. Advisor's sampling-based kinodynamic planner** — RRT-family motion planning
  that respects robot dynamics (from `src/GP/*`, wrapped via the Phase-1.5 C++ bridge). *(TODO)*

**Controllers (local — "what wheel commands?")**
- **C1. Pure Pursuit** — geometric tracker + adaptive lookahead, turn-in-place,
  curvature slowdown. *(done)*
- **C2. MPC** (CasADi/acados) — receding-horizon optimal control with constraints. *(TODO)*
- **C3. MPPI** — sampling-based model-predictive control (GPU rollouts). *(TODO)*

**Matrix (6 cells):**

| | C1 Pure Pursuit | C2 MPC | C3 MPPI |
|---|---|---|---|
| **P1 A\*** | ✅ measured (§4) | ✅ GUI measured (§4.2); headless artifact under investigation | ⬜ |
| **P2 Sampling (advisor)** | ⬜ | ⬜ | ⬜ |

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

### 4.2 P1 A\* + C2 MPC — GUI run clean; headless failure under investigation
| metric | MPC (GUI, 59/60 reached) |
|---|---|
| success rate | **98 %** (59/60) |
| time-to-goal | 41.4 ± 14.9 s |
| cross-track mean / max | **0.024 / 0.167 m** |
| len ratio | 0.966 |
| mean \|ω\| | 0.349 rad/s |
| min wall clearance | 0.366 m |
| compute / tick | **22.7 ms** (vs Pure Pursuit 0.058 ms) |

MPC tracks as tightly as Pure Pursuit (2.4 cm mean) and reaches 59/60 — at ~**400×** the
compute cost. That's the real trade-off: comparable/again-tighter tracking for a much
heavier per-tick solve.

> **Caveat (see docs/14 §6).** The first *headless* sweep failed (30/53, cte 15 m) because
> AMCL diverged. My initial "reverse causes it" diagnosis was **disproven**: the identical
> reverse-enabled MPC ran clean in the GUI (reverse dropped from 30.7 % to 2.2 %), so
> reverse was a *symptom* of the divergence, not its cause. A second hypothesis
> (compute/real-time starvation) was **also disproven** — headless RTF ≈ 1.0 and MPC held
> 20 Hz. Mechanism **unresolved**; it only appears in the long continuous headless sweep,
> so it's likely intermittent/cumulative (future work). The GUI numbers above are the
> trustworthy ones.

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
(3.5 cm — sampling noise), but nearly MPC's speed at **~6× less compute and no solve
spikes**. Notably **ran clean headless** (max pose jump 0.06 m) where MPC diverged — see
docs/14 §6; this is what points the headless failure at MPC's *compute spikes*.

### 4.4 P2 Sampling planner + {C1,C2,C3} — ⬜ TODO

### 4.6 Three-way (A\* planner; PP+MPC in GUI, MPPI headless-clean)
| metric | Pure Pursuit | MPC | MPPI |
|--------|-------------|-----|------|
| success | 58/60 | 59/60 | 59/60 |
| time-to-goal (s) | 56.8 | **40.0** | 45.3 |
| cross-track mean (m) | **0.024** | **0.024** | 0.035 |
| cross-track max (m) | **0.109** | 0.168 | 0.156 |
| min clearance (m) | **0.395** | 0.366 | 0.378 |
| compute/tick (ms) | **0.069** | 22.65 | 3.6 |

**Reading:** three points on a spectrum.
- **Pure Pursuit** — cheapest (µs), tightest peaks, most clearance, smoothest; **slowest**.
- **MPC** — **fastest** (preview), tight mean tracking; **heaviest + spiky compute**
  (headless-fragile).
- **MPPI** — nearly MPC's speed with **~6× less, spike-free compute** and no gradient
  solver; pays with **looser tracking** (sampling noise). A practical middle ground.

### 4.5 Head-to-head: Pure Pursuit vs MPC (matched GUI tour, 58 legs both reached)
Same map, same seed-1 goal tour, both in the GUI. **Bold = winner.**

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

---

## 5. Known limitations

- **Odometry drift — FIXED (Phase 2, AMCL).** A static `map→odom` let wheel-odometry
  drift accumulate; AMCL now publishes a live `map→odom`, and the planner + follower run
  in the `map` frame via TF, so a ~100 m drive stays aligned. This is what makes the
  benchmark possible.
- **Headless MPC divergence (under investigation).** In *headless* runs AMCL diverged
  under MPC (not Pure Pursuit); in the GUI the identical MPC is stable (59/60). Leading
  suspect: MPC's heavy/spiky compute starving the control loop under headless sim timing.
  Benchmark MPC in the mode whose numbers you trust (GUI), and confirm the mechanism
  (real-time factor, `/cmd_vel` rate, solve spikes) as future work.
- Pure Pursuit corner-cutting is mitigated (inflation + turn-in-place), not eliminated.
- No replanning yet if the robot leaves the path (once diverged, no recovery).

---

## 6. Interpretation (to expand as data lands)

- **A\* vs sampling:** A\* is optimal on the grid and ideal for this 2D maze; the
  advisor's sampling planner earns its cost in higher-dimensional / kinodynamic settings
  (car-like model) — the comparison will make *when each wins* concrete.
- **Pure Pursuit vs MPC (measured, §4.5):** the expected story ("MPC tracks tighter") did
  **not** hold — mean cross-track is a **tie (2.4 cm)**. The real result is a trade-off:
  MPC is **30 % faster** (preview lets it carry speed through corners) while Pure Pursuit
  keeps **tighter peaks, more clearance, smoother/less-aggressive steering, and ~330× less
  compute**. *Speed-and-preview vs. precision-and-economy.* MPC's advantage is anticipation
  and dynamic feasibility, not raw tracking — and it costs a heavy per-tick solve.
- **MPPI (next):** sampling under nonconvex costs; GPU rollouts. Expect it to sit between —
  MPC-like anticipation without the gradient solve, at high (parallel) compute.
- **A\* vs sampling (next):** A\* is optimal on the grid and ideal for this 2D maze; the
  advisor's sampling planner earns its cost in higher-dimensional / kinodynamic settings
  (the car-like model) — that comparison will make *when each wins* concrete.

---

## 7. Reproduce

```bash
# maze + map already generated (vto_simulation/maps/maze.{pgm,yaml})
ros2 launch vto_bringup maze_astar.launch.py                     # A* + Pure Pursuit + RViz
ros2 launch vto_bringup maze_astar.launch.py controller:=mpc     # A* + MPC

# benchmark (goal tour, 7 metrics -> results/<controller>_summary.csv); see docs/14
ros2 launch vto_bench bench.launch.py controller:=pursuit num_legs:=30 repeats:=2
ros2 launch vto_bench bench.launch.py controller:=mpc     num_legs:=30 repeats:=2

# offline unit tests:
python3 ros2_ws/src/vto_planning/test/test_astar.py
python3 ros2_ws/src/vto_control/test/test_pursuit_core.py
python3 ros2_ws/src/vto_bench/test/test_metrics.py
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
  Programming*, 2006. **(MPC solver stack)**
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
