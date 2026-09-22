# 20 — Trajectory generation & optimization roadmap: matrix + metrics

Scoping pass before writing any code. Goal: a **realistic** method × robot-model matrix
(not every method makes sense for every model) and a **metrics list**, so the eventual
comparison has a fixed yardstick instead of being decided ad hoc after the fact.

---

## 0. What "3 robot models" actually means in this repo

Before matching methods to models, it matters what the models *are* mechanically —
this changes which methods are even valid.

- **M1 — diff-drive.** The real robot in `ros2_ws` (Gazebo). State `(x, y, θ)`,
  controls `(v, ω)`. First-order, **holonomic in heading** — it can pivot in place (zero minimum turning radius). Already has A\*, Pure Pursuit, MPC, MPPI (docs 07, 05, 13, 15).
- **M2 — car-like.** `src/GP/MPSnakeSimulator` with `SnakeNrLinks = 0`
  ([MPSnakeSimulator.cpp:49](../src/GP/MPSnakeSimulator.cpp#L49)). State
  `(x, y, θ, v, steer_angle)`, controls `(acc, steer_vel)` — **second-order**
  (acceleration/steering-*rate* inputs, not velocity/angle directly) bicycle model ([MPSnakeSimulator.cpp:154-171](../src/GP/MPSnakeSimulator.cpp#L154-L171)). Nonholonomic, bounded turning radius.
- **M3 — "snake"-like.** The **same class**, `SnakeNrLinks > 0`. This is important: it is **not** a biologically-actuated, independently-jointed snake robot. It's the M2 tractor towing `m_nrLinks` **passive** trailer links, each obeying the classic N-trailer recursion
  `θ̇_i = coeff · sin(θ_{i-1} - θ_i)` ([MPSnakeSimulator.cpp:173-181](../src/GP/MPSnakeSimulator.cpp#L173-L181)).
  Only the tractor is actuated; trailer angles are a consequence of the tractor's path, not free controls. That reclassification matters a lot for section 2 — this corrects what I said last message about a serpenoid-curve-style independently actuated snake.

The existing kinodynamic sampling planner (`MPTreePlanner` /
`FollowPlanner` — RRT-style tree with `EXTEND_NORMAL/COLLISION/TARGET/GOAL`
status, [MPTreePlanner.hpp:20-25](../src/GP/MPTreePlanner.hpp#L20-L25), biased by waypoints via `Follow`) already targets M2/M3 natively through `MotionEqs`. It does **not** target M1 — diff-drive would need a new, much simpler `MotionEqs` (first-order, no `acc`/`steer_angle` states).

---

## 1. Generation-method matrix

| Method | M1 diff-drive | M2 car-like | M3 car+trailer ("snake") |
|---|---|---|---|
| **Hybrid A\* / lattice** | Adapt — plain lattice with arc + pivot-in-place primitives is the natural fit; "Hybrid A\*" proper targets bounded-turning-radius vehicles, which M1 isn't | **Direct fit** — the textbook use case (Dolgov et al.) | Adaptation, non-trivial — trailer angles must be added to the search state and primitives must respect the N-trailer recursion; documented in the literature (car-with-trailer parking) but a real step up in complexity |
| **Dubins / Reeds-Shepp curves** | **Not applicable** — both are derived from a bounded-turning-radius, no-pivot-in-place car; forcing them onto M1 misrepresents what it can actually do | **Direct fit** — literally the Dubins/RS car | Usable with a caveat — generate the *tractor's* path with RS, then forward-propagate trailer angles through the N-trailer recursion; RS says nothing about trailer jackknifing, so you inherit a stability check you must add yourself |
| **DWA** | **Ruled out** — see [doc 21](21-dwa-diffdrive.md#5-decision-not-proceeding-with-dwa-for-m1): the maze's dead-end topology defeats short-horizon reactive planning, and the fix (A\*-lookahead target) collapses DWA into just another tracker, losing its distinctness | Adapt — "Ackermann-DWA" (search a `(v, steer)` window respecting curvature bounds) exists but is less standard | **Skip / stretch** — trailer jackknife makes short-horizon reactive control risky without an explicit stability margin in the cost; not a standard combo in the literature |
| **Sampling (src/GP `MPTreePlanner`/`FollowPlanner`)** | Adapt — needs a new, simpler `MotionEqs` subclass (no `acc`/`steer` states) | **Direct fit** — native model (`SnakeNrLinks=0`) | **Direct fit** — native model, this is what the class was built for |

**Bold "Direct fit" cells only** are the ones I'd actually build first; the "Adapt" cells are real but each is its own sub-project, and the "Skip" cell I'd genuinely leave out rather than force.

---

## 2. Trajectory-optimization pipelines

Aiming for 3 core pipelines that are **maximally different in theory** (so the
comparison teaches something), not 5 variations on the same idea:

1. **Direct collocation / multiple shooting (NLP, e.g. CasADi)** — general nonlinear program over the full horizon; dynamics as equality constraints. Applies to all three models by swapping the dynamics constraint. This is the family your MPC (doc 13) already uses, so it's the natural baseline.
2. **iLQR / DDP** — exploits the sequential structure (Riccati-like backward pass) instead of a generic NLP solve. Different algorithmic family from #1 — good "why not just use collocation everywhere" interview contrast. Applies to all three models.
3. **Differential-flatness-based generation** — parameterize a smooth flat output (e.g. a B-spline for the tractor's `(x,y)` path), then get states/controls in *closed form* by differentiation, no solver needed. This is the standout choice for **M3 specifically**: car-with-N-trailers is a textbook differentially flat system (Murray & Sastry / Rouchon) — a strong, well-grounded pipeline that most candidates won't know, and it also degrades gracefully to M2 and M1 (both are flat too, just less interesting a story there).

**Optional, only if you want 4-5:**
4. **MPPI** — only if you keep it *out* of the generation list in §1 and *out* of your existing `vto_control` MPPI controller's story; otherwise it's the same
   method counted twice. It's model-agnostic (needs only a forward rollout, which `SimulateOneStep`/`MotionEqs` already gives you for M2/M3).
5. **acados (real-time SQP/RTI)** — same theoretical family as #1, just a
   production-grade embedded solver instead of a research NLP solver. Worth adding only if the "research solver vs. real-time-capable solver" engineering contrast is itself something you want to speak to in interviews — otherwise it's not a conceptually distinct pipeline from #1 and I'd skip it.

---

## 3. Metrics — extend, don't reinvent

`vto_bench` already has a working per-leg metrics pipeline
([metrics.py:88-92](../ros2_ws/src/vto_bench/vto_bench/metrics.py#L88-L92)):
`reached, ticks, time_to_goal, cte_mean/max/rms, driven_len, planned_len, len_ratio, dw_rms, dv_rms, effort, mean_abs_w, mean_v, min_clearance, compute_ms_mean/max`.

That already covers **tracking accuracy** (cte_*), **control smoothness**
(dw_rms/dv_rms/effort), **path efficiency** (len_ratio), **safety** (min_clearance), and **compute cost** (compute_ms_*). Reuse this harness rather than building a parallel one. What's genuinely missing for this comparison:

- **Dynamic-feasibility violation** — max/mean violation of the model's own bounds (`max_acc`, `max_steer_vel`, curvature limit) along the *generated* trajectory, before it ever reaches a controller. This is the one metric that's specific to judging a *generation/optimization* method rather than a controller.
- **Success rate across seeds** — `reached` aggregated over N trials/maze
  configurations, not a single run (sampling and MPPI methods are stochastic).
- **Jackknife margin (M3 only)** — min over the trajectory of `|θ_{i-1} - θ_i|`
  distance to the recursion's singular point, since Dubins/RS-generated tractor
  paths don't guarantee this on their own (§1).

---

## 4. Suggested sequencing

Given the "one step at a time" pace: build **M1 first**, end-to-end (one generation method → one optimization method → compare, using the existing `vto_bench` harness), before touching M2/M3. That validates the whole pipeline/metrics loop cheaply, since M1 is the only model actually running in Gazebo today. Then extend to M2 (mostly reuses `src/GP` as-is), then M3 (the hardest — new state, new safety metric).

---

## Interview angle

- **Q: Why can't you just use Dubins curves for a differential-drive robot?**
  Dubins curves assume a bounded minimum turning radius and no reverse; diff-drive can pivot in place (zero-radius), so a Dubins path is *stricter* than what the robot can actually do — you'd be planning for the wrong kinematic model.
- **Q: What's the difference between your "car-like" and "snake-like" models here?**
  Same dynamics class, one parameter (`SnakeNrLinks`). Car-like is the bare
  actuated bicycle model; the "snake" is that same tractor towing passive trailer links via the classic N-trailer recursion — only the tractor is controlled, the rest follows kinematically.
- **Q: Why is differential flatness attractive for a car-with-trailers system?**
  You parameterize a smooth flat output and get every state/control in closed form
  by differentiation — no NLP solve, no local minima, and it's exact for this
  system's structure (a known result), unlike generic collocation.
- **Q: iLQR vs. direct collocation — when would you pick one over the other?**
  iLQR exploits the sequential/Markov structure via a Riccati-style backward pass — cheaper per iteration, but needs a smooth, typically unconstrained-ish cost. Collocation handles general inequality constraints (obstacles, actuator limits) more naturally at the cost of a bigger, denser NLP.
- **Q: Why measure "dynamic-feasibility violation" separately from tracking error?**
  Tracking error is a controller property (can it *follow* the trajectory);
  feasibility violation is a generation-method property (was the trajectory even
  drivable in the first place). Conflating them hides which stage of the pipeline is actually at fault.

---

Does this matrix/metrics scope look right, or do you want to cut/add anything before we write doc 21 for the first concrete pipeline (M1 + lattice/DWA + collocation)?
