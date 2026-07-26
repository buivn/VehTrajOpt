# 13 — MPC controller (design)

Model Predictive Control as the second path-tracking controller, to compare against
Pure Pursuit on the same `Path → cmd_vel` interface.

---

## 0. Where it sits

Same seam as Pure Pursuit — a **drop-in** follower, so we can A/B them on identical goals:
```
 /plan (map frame) ─┐
 TF map->base       ─┼─► MPC (optimize N controls) ─► /cmd_vel (v, ω)
 /odom twist (v,ω)  ─┘
```

---

## 1. Theory

MPC = **receding-horizon optimal control**. Each tick:
1. read current state,
2. use a **model** to predict the next **N** steps for any control sequence,
3. **solve an optimization** for the controls that minimize a cost s.t. constraints,
4. **apply only `u₀`**, discard the rest, re-solve next tick.

**Model — kinematic unicycle** (diff drive):
```
ẋ = v·cosθ,   ẏ = v·sinθ,   θ̇ = ω        state (x,y,θ), control u=(v,ω)
```
- **State passed in:** `(x, y, θ)` from TF `map->base`. Velocity is the *control*, not a
  state, in the kinematic model. We still hand in the current `(v, ω)` (from `/odom`
  twist) as an **anchor** `u₋₁` for the acceleration/comfort terms and command continuity.
- **Dynamic-model alternative** (later): state `(x,y,θ,v,ω)`, control `(a,α)` — then
  velocity *is* a state and is required.

**Horizon vs path.** `N` = number of prediction **steps** (a small design constant, e.g.
20), covering time `T = N·dt`. The A\* path has its *own* (larger) waypoint count. Each
cycle MPC tracks a **window of N reference points sampled from the path** ahead of the
robot (spaced ≈ `v·dt`); the window **slides one step per cycle** (because only `u₀` is
applied) — that's the "receding" horizon.

**Control sequence** `U = [u₀ … u_{N-1}]` is found by **optimization** (gradient-based),
**not** random sampling — random sampling of control sequences is **MPPI** (the C3
controller), a different way to solve the same predict-and-optimize problem.

---

## 2. Method selection

**MPC vs Pure Pursuit** (the comparison):
| | Pure Pursuit | MPC |
|---|---|---|
| lookahead | one carrot | **N-step preview** window |
| constraints | none (clamp after) | **inside the optimization** (v, ω, accel) |
| objective | implicit | **explicit tunable cost** (track / effort / comfort) |
| model | none | **unicycle model** |
| cost | ~free | an NLP solve per tick |

**Solver — CasADi + IPOPT** (chosen), vs. acados (real-time C-gen; heavier setup; the
"make it real-time" follow-up) vs. hand-rolled iLQR/QP (no dep, less general, more code).
CasADi/IPOPT reads like the math and is flexible — best for learning + comparison.

**CasADi + IPOPT principle.** CasADi is a **symbolic + automatic-differentiation**
framework: you write the model/cost/constraints symbolically, and it computes **exact**
gradients/Jacobians/Hessians by AD (not finite differences). It assembles a nonlinear
program `min f(w) s.t. g(w)=0, w_lo≤w≤w_hi` (`w` = states + controls). **IPOPT** is the
NLP solver: a Newton-type **interior-point** method that keeps iterates strictly feasible
via a log-barrier and steps toward the KKT optimum using CasADi's exact derivatives. We
pose it as **multiple shooting**: both `X` (states, N+1) and `U` (controls, N) are decision
variables, with the dynamics `xₖ₊₁=f(xₖ,uₖ)` as equality constraints.

---

## 3. Cost functions

State `pₖ=(xₖ,yₖ,θₖ)`, reference `p_ref,k`, control `uₖ=(vₖ,ωₖ)`. Weights are the tuning knobs.

**Tracking** — stay on the reference:
```
J_track = Σₖ ‖pₖ − p_ref,k‖²_Q = Σₖ [ q_x(xₖ−x_ref)² + q_y(yₖ−y_ref)² + q_θ(θₖ−θ_ref)² ]
```
**Effort** — penalize command magnitude:
```
J_effort = Σₖ ‖uₖ‖²_R = Σₖ [ r_v·vₖ² + r_ω·ωₖ² ]
```
**Comfort** — penalize command rate (jerk-like smoothness):
```
J_comfort = Σₖ ‖uₖ − uₖ₋₁‖²_S = Σₖ [ s_v·(vₖ−vₖ₋₁)² + s_ω·(ωₖ−ωₖ₋₁)² ]
```
(cousin of curvature-regulated speed: alternatively `Σ (vₖ·κₖ)²`.)
**Terminal** — pin the horizon end:
```
J_terminal = ‖p_N − p_ref,N‖²_Qf        (Qf ≥ Q)
```
**Full problem each tick:**
```
min_{X,U}  J_track + J_effort + J_comfort + J_terminal
s.t.  xₖ₊₁ = xₖ + [vₖcosθₖ, vₖsinθₖ, ωₖ]·dt      (dynamics)
      x₀ = current pose (TF)                       (initial condition)
      |vₖ|≤v_max, |ωₖ|≤ω_max                        (limits)
      |vₖ−vₖ₋₁|≤Δv_max, |ωₖ−ωₖ₋₁|≤Δω_max          (optional accel; v₋₁ = current speed)
```
`Q/R/S/Qf` = the controller's personality (aggressive vs. economical vs. smooth) — exactly
what Pure Pursuit lacks.

---

## 4. Design (code)

Same framework-free split as `astar.py` / `pursuit_core.py`:
- **`mpc_core.py`** (no ROS): build the CasADi NLP **once** (symbolic `X`, `U`, params =
  current state + `u₋₁` + the N reference points); `solve(state, u_prev, ref)` returns
  `(v, ω)`, **warm-started** from the last solution. Host-testable on a toy trajectory.
- **`mpc_controller_node.py`**: identical interface to Pure Pursuit — subscribe `/plan`,
  pose from **TF map->base**, current `(v,ω)` from `/odom` twist, **extract the N-point
  reference window** from the path (sample at ≈`v·dt`, clamp to goal near the end), call
  `mpc_core.solve`, publish `/cmd_vel`.

Data structures: `X` = (N+1)×3, `U` = N×2 decision vars; parameter vector packs the
current state + `u_prev` + reference. Cost weights `Q,R,S,Qf` as node parameters.

---

## 5. Build plan (one step at a time)

1. **Dockerfile + rebuild** — add `casadi`, rebuild the image. *(this step)*
2. **`mpc_core.py`** + host test — track a toy straight/curve, check it converges + obeys limits.
3. **`mpc_controller_node.py`** + package wiring (`tf2_ros`, entry point).
4. **Launch** — swap Pure Pursuit ↔ MPC (a `controller:=` arg, or `maze_mpc.launch.py`).
5. **Compare** — run both on the same goals; log tracking error / effort / time into the report.

---

## Interview angle

- **Q: MPC in one sentence?** At each step, predict N steps ahead with a model, optimize
  the control sequence to minimize a cost under constraints, apply the first control, repeat.
- **Q: MPC vs Pure Pursuit?** PP is a one-point geometric heuristic with no constraints or
  model; MPC previews N steps, uses the model, and enforces constraints inside an explicit
  optimized cost.
- **Q: MPC vs MPPI?** Same receding-horizon predict-and-optimize; MPC finds the sequence by
  gradient-based NLP (CasADi/IPOPT), MPPI by sampling many rollouts and cost-weighting them.
- **Q: What does CasADi do vs IPOPT?** CasADi models symbolically + gives exact derivatives
  by autodiff and builds the NLP; IPOPT is the interior-point solver that uses them.
- **Q: Single vs multiple shooting?** Multiple shooting makes states *and* controls decision
  variables with dynamics as equality constraints — more variables but far better numerical
  conditioning than rolling the model out inside the cost (single shooting).
- **Q: Why apply only the first control?** Feedback: re-solving from the true measured state
  each tick rejects disturbance/model error — the later horizon controls are just predictions.
