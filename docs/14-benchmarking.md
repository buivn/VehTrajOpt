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

## 6. Finding: an MPC AMCL divergence — and a corrected diagnosis

The first (**headless**) MPC sweep **failed on ~half the legs** (30/53 vs Pure Pursuit's
58/60) with *physically impossible* metrics — cross-track 15 m, driven 8–40× planned,
clearance 0. Per-tick traces showed the **pose estimate teleporting 37 m**: AMCL diverged
and MPC then drove on a garbage pose. Not a real "MPC is worse" result.

**First (WRONG) hypothesis — reverse causes it.** MPC reversed 30.7 % of ticks (Pure
Pursuit 0 %); I concluded the backing-up/spinning stressed AMCL into diverging, and
constrained MPC forward-only (`v_min=0`).

**Disproof (a clean experiment).** Re-running the *identical* reverse-enabled MPC
(`mpc_v_min:=-0.5`) **in the GUI** ran clean: **59/60 reached, 2.4 cm tracking, and only
2.2 % reverse.** Same code, same bounds — so reverse was **not** the cause. The 30.7 % vs
2.2 % reverse gap is the tell: a diverged/jumping pose makes MPC *think it overshot* and
command reverse constantly, so **reverse was a *symptom* of divergence, not its cause**.
Causality was backwards.

**What actually differs: headless vs GUI, and it's MPC-specific.** Pure Pursuit is fine
headless; MPC fails headless but not in the GUI.

**Second hypothesis — compute/real-time starvation — also DISPROVEN.** A headless
diagnostic measured **RTF ≈ 1.000** (sim not running fast) and **MPC `/cmd_vel` = 19.97 Hz**
(control loop keeping up fine). So neither "sim too fast" nor "MPC can't keep up" holds.

**Third experiment — MPPI headless (the informative one).** MPPI is the *same* kind of
heavy predictive controller as MPC but ~10× cheaper per tick and, crucially, **spike-free**
(constant ~3.6 ms sampling, std 0.04). Run on the *same headless sweep*, it stayed **clean:
59/60, max pose jump 0.06 m** (vs MPC's 37 m). So:
- headless + heavy predictive control is **not** inherently the problem (MPPI is fine),
- the distinguishing factor is MPC's **occasional IPOPT compute spikes (200 ms+)**, not its
  mean load (a 10 s snapshot showed MPC at 20 Hz / RTF 1 — the spikes are *intermittent*).

**Leading conclusion (well-supported, not yet directly proven):** MPC's **intermittent
solve spikes** momentarily gap the control loop; over a 90-min / 60-leg run these
accumulate into an AMCL divergence, then cascade. MPPI (no spikes) and Pure Pursuit (µs)
never gap, so both are headless-robust. **Direct confirmation** would still log `map→odom`
jumps vs. per-tick solve time over a full run to catch a spike immediately preceding the
first jump. For the report: Pure Pursuit + MPC use GUI numbers (MPC is headless-fragile);
MPPI's headless numbers are trustworthy (it didn't diverge).

**Lessons (these hold regardless of the final mechanism):**
- Correlation ≠ causation — a rampant behavior (reverse) can be a *symptom* of the failure,
  not its cause. A cheap controlled experiment (flip one variable) beats a confident story.
- Test controller + estimator **together, at scale**; and beware that **headless vs GUI can
  change real-time behavior** — benchmark in the mode you'll trust the numbers from.
- Pure Pursuit's microsecond compute is robust to timing; a heavy optimal controller (MPC)
  must be shown to keep up under the *actual* run conditions.

### What actually happened
Not a code bug — an emergent, closed-loop system failure:
```
MPC allowed reverse (v∈[-0.5,0.5]) — used 30.7% of ticks + ~2× angular rate
   → robot backs up / spins  ──(Gazebo physics)──► wheel SLIP
       ├─ wheel odometry ≠ true motion  → AMCL motion model mispredicts
       └─ scan changes fast between updates → scan-match ambiguous
   → AMCL loses lock → jumps to a WRONG look-alike corridor (perceptual aliasing)
       → pose estimate teleports 37 m
   → MPC reads garbage pose → commands garbage → hits walls
   → more slip/spin → AMCL worse → runaway → leg times out
```

### Why reversing/spinning breaks AMCL specifically
AMCL (docs/11) has two engines; aggressive motion sabotages both:
- **Motion model (odometry):** reversing / direction-changes / spinning cause **wheel
  slip** (Gazebo simulates it faithfully), so the odometry delta AMCL propagates its
  particles with no longer matches the true motion.
- **Measurement model (scan match):** fast rotation makes the 10 Hz scan change a lot
  between updates → matching to the map becomes ambiguous.
- **Perceptual aliasing:** the maze's near-identical corridors mean that once the cloud
  drifts, a *wrong* pose matches the scan just as well → the filter commits to a
  look-alike corridor tens of metres away.

### Why Pure Pursuit was immune
Forward-only (`v ≥ 0`) + turn-in-place → smooth, predictable, mostly-forward motion →
little slip, slow scan change → AMCL stays locked (58/60).

### Why our earlier tests missed it
- `mpc_core` test integrates the model with a **perfect pose** — no AMCL/physics; it's
  **open-loop w.r.t. localization**, so reverse was harmless there.
- The 3-leg smoke was too short; divergence is **statistical + cumulative** — 60 legs
  over 90 min made it near-certain.
- It only emerges **at scale, in closed loop, with the full stack**.

### Root cause & fix
Symmetric velocity bounds let the optimizer **exploit reverse** whenever it lowered the
*tracking* cost (overshoot correction, 3-point turns) — nothing penalized
localization-unfriendly motion (**cost/constraint misspecification**). **Fix:** constrain
MPC **forward-only** (`v_min = 0`), matching Pure Pursuit. Next levers if spinning alone
still stresses AMCL: soften angular aggressiveness (`r_w`/`s_w` up) or harden AMCL (more
particles).

### Reproduce it (watch it live)
```bash
# BUG (reverse enabled) — watch AMCL diverge in RViz vs the true robot in Gazebo:
ros2 launch vto_bringup maze_astar.launch.py controller:=mpc mpc_v_min:=-0.5
# FIX (forward-only, default):
ros2 launch vto_bringup maze_astar.launch.py controller:=mpc
```
Send a faraway 2D Nav Goal; with reverse on, after some driving the **RViz robot +
`/particle_cloud` jump to a wrong corridor** while the Gazebo robot is elsewhere.

### Lesson (interview-grade)
Controller and estimator form a feedback loop — test them **together, at scale**, not in
isolation. "Worked offline/in-sim" validates a component, not the system. An optimizer
exploits any unconstrained freedom, even motions that break other subsystems. And a
**high-fidelity simulator (Gazebo's slip physics) is an asset** — it exposed a failure a
perfect-odometry sim would have hidden until hardware.

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
