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
| **P1 A\*** | ✅ measured (§4) | ⬜ | ⬜ |
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

**Qualitative (Gazebo):** short/medium goals reached without collision, path centred,
pivots at sharp corners. **Faraway goals FAIL** — see §5.

### 4.2 P1 A\* + C2 MPC — ⬜ TODO
### 4.3 P1 A\* + C3 MPPI — ⬜ TODO
### 4.4 P2 Sampling planner + {C1,C2,C3} — ⬜ TODO

---

## 5. Known limitations

- **Odometry drift (Phase 1.5).** With only a static `map→odom`, wheel-odometry drift
  accumulates over distance; the controller closes the loop on the *belief*, so RViz
  "reaches" the goal while the real robot drifts into a wall. **Fix = AMCL (Phase 2)**,
  in progress. All *long-range* success-rate numbers are pending AMCL.
- Pure Pursuit corner-cutting is mitigated (inflation + turn-in-place), not eliminated.
- No replanning yet if the robot leaves the path.

---

## 6. Interpretation (to expand as data lands)

- **A\* vs sampling:** A\* is optimal on the grid and ideal for this 2D maze; the
  advisor's sampling planner earns its cost in higher-dimensional / kinodynamic settings
  (car-like model) — the comparison will make *when each wins* concrete.
- **Pure Pursuit vs MPC vs MPPI:** geometric (no model/constraints) → optimization with
  constraints (MPC) → sampling under nonconvex costs (MPPI). Expect MPC/MPPI to cut
  tracking error and handle dynamics/comfort at higher compute cost.

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
