# 14 — Benchmarking controllers (Pure Pursuit vs MPC, and beyond)

How we measure controllers fairly and reproducibly to fill the report. The harness is
`vto_bench`; it runs a **goal tour** and logs 7 metrics per leg. Same node measures MPC,
Pure Pursuit, and later MPPI / the advisor's planner.

---

## 1. The 7 metrics

| # | metric | source | reveals | expected winner |
|---|--------|--------|---------|-----------------|
| 1 | **cross-track error** (mean/max/rms) | dist(pose, `/plan`) per tick | tracking tightness | MPC (esp. max, at corners) |
| 2 | **time-to-goal** | leg duration | overall speed | ~tie / PP |
| 3 | **len ratio** driven/planned | ∫\|Δpose\| ÷ plan length | wandering / overshoot | MPC |
| 4 | **command smoothness** dω/dv rms | rate of `/cmd_vel` change | jerk / comfort | MPC |
| 5 | **control effort** ∫(v²+ω²), mean\|ω\| | `/cmd_vel` | aggressiveness / energy | MPC |
| 6 | **min wall clearance** | dist(pose, nearest wall) via map | closest brush | MPC |
| 7 | **compute time / tick** | `/controller/compute_ms` | cost of the method | **PP** (µs vs ~ms) |

Plus **success** (reached vs timeout). The story: MPC wins accuracy/comfort/clearance,
Pure Pursuit wins compute — the classic quality-vs-cost trade.

---

## 2. Fair comparison: the goal TOUR

To compare fairly, both controllers must face the **same map, same goals, same starts**.
Two ways to get many start-goal pairs:

- **Teleport (independent):** reset the robot to each start, re-seed AMCL, wait for
  re-convergence. Exact starts, but heavy automation + slow (reset/reconverge per pair).
- **Tour (chained) — what we use:** send goal₁, on reach send goal₂ (start of leg *i* =
  end of leg *i*−1), … Each **leg is one start-goal pair**. One continuous run per
  controller, AMCL stays converged, no teleport. Caveat: a leg's start is within
  `goal_tolerance` (~0.25 m) of the previous goal, so starts aren't bit-identical between
  controllers — negligible given the large effect sizes (tighten tolerance if needed).

The goal list is **seeded** (sampled well-clear, A\*-reachable maze cells) → **identical
every run and every controller**. Default: **30 legs × 2 repeats**, scalable to 100 by
raising `num_legs` (repeats matter because AMCL is slightly stochastic).

---

## 3. The harness (`vto_bench`)

- **`metrics.py`** (pure, host-tested): `cross_track`, `path_length`, `summarize(ticks)`
  → the 7-metric row.
- **`benchmark` node**: on `/map`, builds the seeded tour (A\*-verified reachable). Then
  publishes goals, and a 20 Hz timer samples per tick — pose (TF `map→base`), `/cmd_vel`,
  cross-track vs `/plan`, wall clearance, `/controller/compute_ms`. On reach/timeout it
  writes a **per-leg summary row** + the **per-tick** rows to CSV, then advances.
- **compute-time**: each controller publishes its per-tick solve time on
  `/controller/compute_ms` (Pure Pursuit ~µs, MPC ~ms) — so metric 7 is real, not guessed.

Outputs (in `output_dir`, default `ros2_ws/results/`):
`<controller>_summary.csv` (one row/leg) and `<controller>_ticks.csv` (raw per-tick).

---

## 4. How to run

```bash
# in the container (recreated so it has casadi):
cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch vto_bench bench.launch.py controller:=pursuit   # -> results/pursuit_summary.csv
ros2 launch vto_bench bench.launch.py controller:=mpc       # -> results/mpc_summary.csv
# scale up:  ... num_legs:=100 repeats:=3
```
Headless by default; each run drives the whole 30×2 tour unattended.

---

## 5. Analysis → report

Aggregate each `*_summary.csv` (mean ± std over reached legs) into the report's matrix
cells (A\*×Pure-Pursuit, A\*×MPC). Compare column-by-column; the per-tick CSV backs up any
plot (e.g. cross-track vs. time through a corner).

---

## Interview angle

- **Q: How do you compare two controllers fairly?** Same map/goals/starts; identical
  seeded goal list; measure multiple metric axes (accuracy, efficiency, comfort, safety,
  compute), repeat for noise. Don't compare on a single run or a single metric.
- **Q: Why a tour instead of independent resets?** Avoids per-pair teleport + AMCL
  re-convergence; one continuous run yields many overlapping start-goal legs cheaply.
- **Q: Why repeats if the goals are deterministic?** AMCL (a particle filter) and physics
  contacts are stochastic, so the same leg varies run-to-run; repeats estimate that noise.
- **Q: What single number would you NOT rely on?** Time-to-goal alone — a corner-cutting
  controller can be "faster" while tracking worse and coming closer to walls.
