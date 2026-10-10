# 28 — Acados model translation + non-blocking control loop

Doc 27's step 1 (Docker/acados toolchain) is done and verified. This doc covers step 2
(translate `mpc_core.py`'s unicycle model into `AcadosOcp`) plus a second, related
change: making `mpc_controller_node.py` never block on a slow solve, falling back to
the **already-computed future** of the last plan instead of freezing.

---

## 0. Where it sits

Same seam, but the box now has two moving parts instead of one:

```
 /plan (map) ─┐
 TF map->base ─┼─► [solve on a worker thread] ──► U_seq (2, N) ─┐
 /odom twist  ─┘                                                 │
                    fixed-rate timer (unchanged cadence) ────────┴──► /cmd_vel
                    always publishes SOMETHING every tick:
                    fresh U_seq[:,0] if a solve just finished,
                    else U_seq[:,k] from the LAST finished solve (k = ticks since)
```

---

## 1. Theory

**1a. Why the model needs re-deriving, not just re-pointing.** `mpc_core.py` builds one
CasADi `Opti` problem and calls a generic NLP solver on it each tick. acados instead
**code-generates and compiles a solver specific to one fixed OCP structure** (stage
count, cost type, dynamics map) — you don't hand it an `Opti` object, you describe the
OCP once (`AcadosModel` + `AcadosOcp`) and it emits C. So "switch solver" here means
re-expressing the same problem in a different modeling interface, not swapping a string.

**1b. The control-rate (comfort) cost needs a structural trick.** `mpc_core.py:50-53`
computes `du = U[:,k] - U[:,k-1]` — a cost term that reaches across two *different*
stages of the horizon. acados' per-stage `NONLINEAR_LS` cost only sees `(x_k, u_k, p_k)`
at that one stage — no cross-stage references. The standard fix (used here) is **state
augmentation**: add `(v_prev, w_prev)` as two extra state components whose "dynamics"
are trivial (`v_prev_{k+1} = v_k`, `w_prev_{k+1} = w_k` — they just *record* the control
just applied). Then `u_k - v_prev_k` is a same-stage residual, valid everywhere,
and at k=0 that recorded value is seeded from the real last-applied control — exactly
what `mpc_core.py`'s `u_prev` parameter already anchors, just carried differently.

**1c. Heading error still needs wrapping.** `mpc_core.py:47` wraps `e[2]` (the angle
*difference*) before squaring it, so a heading of `179°` vs a reference of `-179°`
reads as a 2° error, not 358°. acados' `NONLINEAR_LS` cost is `(y_expr(x,u) − yref)`
squared — the wrapping has to live *inside* `y_expr` (as `atan2(sin(θ−θ_ref), cos(θ−θ_ref))`,
with `θ_ref` carried in as a **per-stage parameter** `p`), not in a numeric `yref`.

**1d. Why blocking happens, and why "replay the plan" is the right degrade.** Covered
in doc 27 — IPOPT's iteration count (and now, in principle, even acados' bounded but
non-zero solve time) isn't a hazard *by itself*; it becomes one because
`mpc_controller_node.py:129` calls `solve()` **inline in the publish timer's callback**,
so a slow solve delays the very message that should have gone out on schedule. A
receding-horizon solve already produces a full **planned sequence** `U[0..N-1]`, not
just `U[0]` — throwing away `U[1..N-1]` every tick (current code) discards information
that's already paid for. If a solve doesn't finish in time, replaying `U[k]` (k = how
many ticks have elapsed since that plan was computed) is still the model's own optimized
answer to "what should I be doing k steps into this plan" — an open-loop continuation
of a locally-optimal trajectory, degrading gracefully instead of freezing or stopping.

---

## 2. Method selection

**Dynamics representation:** `model.disc_dyn_expr` (a discrete map `x_{k+1}=φ(x_k,u_k)`)
over `f_expl_expr` + an ERK/IRK integrator. `mpc_core.py`'s dynamics **are already**
forward-Euler applied once per stage (`mpc_core.py:57-62`) — `disc_dyn_expr` reproduces
that exactly, with zero integration-scheme mismatch against the IPOPT baseline being
compared to. An integrator would add a choice (how many RK stages/steps per `dt`) that
doesn't exist in the current code and would confound the A/B comparison.

**Control-rate cost:** state augmentation (§1b), not a slack variable or a soft
constraint pushed into `con_h_expr` alone. Augmentation keeps it a **cost** term (matches
`mpc_core.py`'s `s_v`,`s_w` weights being cost weights, not hard limits) while the
*hard* rate bounds (`dv_max`,`dw_max`) become a real nonlinear constraint
(`con_h_expr`) on the same augmented quantity — one mechanism serving both the cost and
the constraint, instead of two different ad-hoc encodings.

**Non-blocking execution — why a plain thread pool over rclpy's `MultiThreadedExecutor`:**
Both would work; a `ThreadPoolExecutor(max_workers=1)` gives an explicit, easy-to-audit
invariant — *never submit a second solve while one is in flight* — via `future.done()`.
Doing the same with executor callback groups means reasoning about rclpy's reentrancy
rules and could let two solves overlap on the same (non-thread-safe) `AcadosOcpSolver`
object if a callback group is set up wrong. The explicit route is worth the small extra
code for that guarantee.

**Fallback policy — why replay `U_seq[k]` and not the alternatives:**
- *Hold `u0` forever* (today's de facto behavior, via `diff_drive_controller`'s
  0.5s `cmd_vel_timeout`) — freezes the command at a single point instead of advancing
  through the plan's already-computed profile (e.g. a planned deceleration into a turn
  gets skipped entirely if `u0` — the pre-turn cruising speed — is just held).
- *Dead-reckon a new command from current velocity + trend* — cheaper, but throws away
  a real optimized trajectory that's already sitting in memory for free, in favor of a
  heuristic with no knowledge of the upcoming path curvature.
- **Replay `U_seq[k]` from the last completed solve — chosen.** Zero extra compute,
  uses the actual optimized answer, and degrades to "open-loop coast on stale
  information" only for as many ticks as the solve is actually late — bounded by
  `N` ticks (2.0 s at N=20, dt=0.1s) before a hard stop is the honest answer (the plan
  has run out; no data left to replay).

---

## 3. Design

**New class `MPCCoreAcados`** (`vto_control/mpc_core_acados.py`) — same
`solve(state, u_prev, ref) -> (v, w)` interface as `MPCCore`, so it's a drop-in
replacement. New public attribute `self.U_seq` (`(2, N)` ndarray, the full predicted
control sequence from the last solve) — `MPCCore` never needed to expose this because
the node only ever asked for `U[0]`; the async fallback below needs the rest of it.
A class, not a function, because it owns a stateful `AcadosOcpSolver` (persistent
compiled solver + its internal iterate) across calls, exactly like `MPCCore` owns a
persistent `Opti`.

**`mpc_controller_node.py` changes:**
- Swap `MPCCore` → `MPCCoreAcados` (the IPOPT baseline stays preserved as the
  `mpc_headless_confirm2_*` CSVs already on disk — no need to keep both wired into the
  node).
- New members: `self._executor = ThreadPoolExecutor(max_workers=1)`, `self._future`
  (pending solve or `None`), `self._seq` (last completed `(2,N)` sequence),
  `self._seq_idx` (how many ticks consumed from `self._seq` since it was produced).
- `_control()` (the timer callback, still fixed-rate — that cadence doesn't change):
  1. If `self._future` is `None` or already submitted-and-not-yet-checked this tick,
     and no solve is currently in flight: read the current pose/ref and submit a new
     solve to the executor.
  2. If the in-flight future is done: pull `(v, w, U_seq)` from it, adopt it as
     `self._seq`, reset `self._seq_idx = 0`, publish `(v, w)` (`= U_seq[:,0]`).
  3. Else (still computing): `self._seq_idx += 1`; if `< N`, publish
     `self._seq[:, self._seq_idx]`; if `>= N`, the plan is exhausted — publish
     `(0, 0)` and log a warning (a real overrun, not the common case).

This makes "how long the solve takes" and "when a command goes out" fully independent,
bounded on the safe side by `N·dt` before an explicit stop — never an indefinite block.

---

## Interview angle

- **Q: Why can't you just port the CasADi `Opti` cost straight into acados?** acados
  needs the OCP's structure fixed at code-gen time (cost type, per-stage vs cross-stage
  terms) — a cost like `U_k − U_{k-1}` that reaches across stages has no direct
  per-stage `NONLINEAR_LS` equivalent, so it has to be re-expressed as a same-stage
  residual via state augmentation.
- **Q: What is state augmentation, concretely?** Adding state components that don't
  represent physical quantities but *carry information forward* — here, "the control
  applied at the previous stage" — so a per-stage cost/constraint can reference it
  without looking at another stage.
- **Q: Why replay old plan steps instead of just recomputing faster or waiting?**
  Because "wait" is exactly the bug (a blocked publish delays every downstream
  consumer), and the previously computed steps of the plan are not stale guesses —
  they're the optimizer's own answer for exactly this point in time, just computed a
  few ticks ago. It's a principled degrade, not a guess.
- **Q: What's the failure mode if the solve is late for the entire horizon?** The
  stored sequence runs out at index `N`; the only honest fallback left is to stop
  (`v=w=0`), because replaying past the horizon means inventing motion the optimizer
  never actually computed.
- **Q: Why `ThreadPoolExecutor(max_workers=1)` instead of rclpy's own threading?**
  It makes "at most one solve in flight" an explicit, checkable invariant
  (`future.done()`) rather than something that has to be reasoned about via callback
  group / executor semantics — the underlying solver object isn't thread-safe to call
  concurrently, so that invariant matters.
