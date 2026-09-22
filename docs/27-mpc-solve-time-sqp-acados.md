# 27 — MPC tail-latency bug: diagnosis, and switching to SQP via acados

Doc 14's benchmark flagged headless MPC runs as suspicious; commit `7f2998c` first
pointed at "MPC solve spikes." This doc closes that loop: what's actually causing the
spikes, why the fix is a solver-*method* change (not a warm-start patch), and why
**acados** specifically over CasADi's built-in SQP.

---

## 0. Where it sits

Same seam as always — the fix only touches what's inside the box:

```
 /plan (map) ─┐
 TF map->base ─┼─► MPC (build NLP once, RE-SOLVE each tick) ─► /cmd_vel
 /odom twist  ─┘         ^
                          swap this: IPOPT (interior point) -> SQP (acados, RTI)
```

`mpc_core.py` is framework-free (no ROS deps) — the ROS node (`mpc_controller_node.py`)
just calls `.solve(state, u_prev, ref)`. Whatever solver backend goes in, that interface
stays the same.

---

## 1. Theory — why solve time isn't constant

MPC re-solves an optimization problem **every control tick** (dt = 0.1 s here). The
current backend is IPOPT, an **interior-point method**: it approaches the constrained
optimum by following a "central path" as a barrier parameter → 0, and the number of
Newton iterations it needs is **not fixed** — it depends on how many inequality
constraints are active at the optimum and how much that active set has to change from
the current iterate.

`mpc_core.py` already warm-starts (`_Xg`/`_Ug` cache the last solution, `mpc_core.py:81,
96-98`), and `MPCCore` is a single persistent object for the node's life
(`mpc_controller_node.py:60`) — it is never rebuilt cold. So "cold start" in the literal
sense (no initial guess) never happens in this loop.

**Data check** (`mpc_headless_confirm2_ticks.csv`, 100 legs): for each leg, where does
its worst `compute_ms` spike fall relative to leg start?

- **16/100** legs peak in the first 5 ticks — a real effect, but not "cold start": it's
  a **stale** warm start. A new leg hands the solver a discontinuous new reference
  window while `_Xg` still holds the *previous* leg's trajectory shape.
- **84/100** peak spikes are scattered deep into the leg (e.g. tick 452/646, 860/1107,
  218/1204) — far past where the warm start should already be excellent.

So the dominant mechanism is mid-trajectory: IPOPT's iteration count jumps when the
**active set changes** — e.g. a clearance or rate limit (`dv_max`/`dw_max`,
`mpc_core.py:69-74`) newly binds because of a tight maze corridor
(`min_clearance` in the benchmark runs as low as 0.2–0.4 m). A good warm start in
*state space* doesn't guarantee a similar active set, so the iteration count — and
therefore solve time — has no upper bound. At dt = 0.1 s, a 270 ms solve is a ~2.7×
control-period violation: several control cycles get skipped or delayed. That's a
correctness bug in the control loop, not benchmarking noise.

---

## 2. Method selection

The fix has to target the actual mechanism (active-set-driven iteration variance), not
just leg transitions.

| Option | Mechanism | Verdict |
|---|---|---|
| Re-seed warm start only at leg boundaries | Fixes the stale-guess case | Only ~16% of the problem |
| Lower `max_iter` | Truncates IPOPT, bounds time by force | Band-aid — degrades solution quality, doesn't fix why iterations vary |
| CasADi `sqpmethod` (generic SQP, e.g. `qrqp` QP backend) | One QP solve per SQP iteration instead of an interior-point barrier schedule; cap iterations (RTI-style) | Bounds per-tick work in principle; smallest code change (same `Opti` formulation) but the SQP loop itself still runs through CasADi's interpreter, and the QP is solved generically — doesn't exploit the shooting structure |
| **acados (SQP_RTI) — chosen** | Same SQP idea, but: (a) exploits the multiple-shooting block structure via a **Riccati recursion** in HPIPM instead of a generic QP solve, (b) splits each tick into a **preparation phase** (linearize + condense — can run before the new state arrives) and a **feedback phase** (just the final small solve) to minimize sensor-to-command delay, (c) generates the whole solver as compiled C — a solve is a compiled function call, not an interpreted loop | Directly bounds tail latency by construction (fixed QP structure size, not "iterate until converged"), and is the actual industry-standard answer for real-time NMPC (interview-relevant on its own) |

**Why acados over CasADi's own `sqpmethod`:** both give an SQP-shaped bound instead of
IPOPT's variable one, but acados' structure-exploiting Riccati/HPIPM core and RTI
prep/feedback split are what actually make the *worst case* tight — a generic SQP loop
still pays for a dense/sparse QP solve and an interpreted orchestration layer. Given the
observed problem (tail spikes up to 2.7× the control period, not a slow average), the
worst-case bound is what needs fixing, so it's worth the larger integration cost.
acados also isn't really a competitor to CasADi — it **uses CasADi underneath** to
generate the symbolic model/derivatives, then hands them to its own C solver core.

**Why not do this right away as one change:** acados isn't `pip install`-able like
CasADi — it's a C library (BLASFEO + HPIPM + the acados core) built via CMake, plus a
Python wrapper (`acados_template`) that code-generates and compiles a solver **per
problem**. That's Docker image work, a new toolchain dependency, and a real build step,
not a one-line solver-string swap. Per the "one step at a time" rule, this is broken
into stages instead of one multi-file burst.

---

## 3. Build plan (one step at a time)

1. **Docker image: install acados core + `acados_template`.** Clone acados (with the
   BLASFEO/HPIPM git submodules), build via CMake, `pip install` the Python template
   package, set `ACADOS_SOURCE_DIR`/`LD_LIBRARY_PATH`. Verify with acados' own minimal
   OCP example — **no `vto_control` code touched yet.** This step needs network access
   and adds real build time to the image, so it's a deliberate checkpoint before
   anything else.
2. **Translate the model.** Re-express the unicycle dynamics + cost + constraints
   (currently `mpc_core.py:38-74`) via `AcadosModel`/`AcadosOcp` — reusing the same
   CasADi symbolic expressions, since acados takes CasADi `SX` directly. New file
   (e.g. `mpc_core_acados.py`) implementing the **same** `solve(state, u_prev, ref)`
   interface so `mpc_controller_node.py` doesn't need to change shape.
3. **Solver options.** SQP_RTI, HPIPM as the QP solver, ERK integrator for the simple
   unicycle ODE — chosen deliberately, not left at acados' defaults.
4. **Re-benchmark.** Re-run `vto_bench` with the acados backend, compare
   `compute_ms_mean`/`compute_ms_max` directly against the IPOPT baseline in
   `mpc_headless_confirm2_summary.csv` — the same tour, same seeds, so it's a clean
   A/B.

This doc covers steps 0–2 (the "why"); step 1's actual execution (Dockerfile change +
rebuild) is next, pending check-in.

---

## Interview angle

- **Q: Why does MPC solve time vary tick to tick even with warm-starting?** Warm-starting
  gives IPOPT a good *starting point in state space*, but the number of interior-point
  iterations depends on how the *active constraint set* changes — a good state-space
  guess doesn't guarantee a similar active set, so iteration count (and time) isn't
  bounded.
- **Q: SQP vs interior-point (IPOPT) for real-time MPC?** Interior-point follows a
  barrier-parameter path to the optimum with a variable iteration count; SQP solves a
  sequence of quadratic approximations and, in the Real-Time Iteration variant, is
  deliberately truncated to *one* QP per control tick — trading a fully-converged
  solution for a bounded, predictable per-tick cost, which is what a hard real-time
  control loop actually needs.
- **Q: What does acados add over "just do SQP in CasADi"?** Structure exploitation
  (Riccati recursion over the shooting stages via HPIPM, instead of a generic QP solve)
  and a compiled-C execution path, plus the RTI prep/feedback split that overlaps the
  expensive linearization with the time before the new state arrives.
- **Q: Is acados a replacement for CasADi?** No — acados uses CasADi to build the
  symbolic model/derivatives, then solves with its own specialized core. They compose.
- **Q: Why not just cap `max_iter` on the existing IPOPT solve?** It bounds time by
  brute force but produces a worse (possibly infeasible-feeling) control at exactly the
  ticks where the active set is changing — the moments that matter most for safety
  (e.g. near a wall). SQP's bound comes from problem structure, not from giving up early.

---

## References

*(Verify exact bibliographic details before formal citation.)*

- M. Diehl, H. G. Bock, J. P. Schlöder, "A Real-Time Iteration Scheme for Nonlinear
  Optimization in Optimal Feedback Control," *SIAM J. Control Optim.*, 2005.
- R. Verschueren et al., "acados: a modular open-source framework for fast embedded
  optimal control," *Mathematical Programming Computation*, 2021.
- G. Frison, M. Diehl, "HPIPM: a high-performance quadratic programming framework for
  model predictive control," *IFAC World Congress*, 2020.
- J. A. E. Andersson et al., "CasADi: a software framework for nonlinear optimization
  and optimal control," *Mathematical Programming Computation*, 2019.
