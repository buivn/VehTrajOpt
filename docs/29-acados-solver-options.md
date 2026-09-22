# 29 — acados solver options: checked, not left at defaults

Step 3 of the docs/27 plan. `MPCCoreAcados` (docs/28) already had to choose several
solver options just to compile — this doc is about the ones that took real
investigation, and the one genuine gap found and fixed.

---

## 1. Theory — the inner QP has its own iteration count

docs/27 argued IPOPT's tail latency comes from its iteration count varying with the
active set, and that RTI bounds this by doing exactly **one SQP iteration per tick**.
That's true at the *outer* (SQP) level — but that one iteration still means solving
**one QP**, and HPIPM (the QP solver) is itself a structured interior-point method.
It has its own iteration count, which is *also* not fixed a priori. So "acados bounds
tail latency" isn't automatic — it depends on that inner QP being cheap and consistent
too, which depends on solver options actually set correctly.

---

## 2. What was checked, and what was actually wrong

Rather than assume acados' defaults are fine (or assume they need tuning), each
relevant option was introspected on the installed version (`AcadosOcpOptions`) before
touching anything:

- **`print_level`** — already `0` by default. No I/O-per-solve overhead exists (the
  per-iteration tables seen in doc 27's smoke test came from an explicit
  `print_statistics()` call in that demo script, never called here). Set explicitly
  anyway, matching `mpc_core.py`'s style of stating solver behavior rather than
  relying on unexamined defaults — free, no functional change.
- **`qp_solver_iter_max`** — defaults to `50`, already a hard cap on the inner QP's
  worst case. No evidence (measured: max ~8 iterations, §3) that this needs lowering,
  and lowering it without evidence would be exactly the "truncate and hope" band-aid
  doc 27 rejected for IPOPT's `max_iter` — the difference here is the cap already
  exists and is already generous relative to what's actually used. Left alone.
- **`nlp_solver_warm_start_first_qp`** — defaults to **`False`**. This is the one real
  gap. SQP_RTI does exactly one QP solve per `solve()` call, which makes that QP "the
  first QP of the NLP solve" by acados' own accounting *every single tick* — so this
  default means **the inner QP was cold-starting every tick**, even though the outer
  trajectory guess (`X`, `U`) already persists across calls via the solver's own
  internal iterate. The persisted outer guess was never actually reaching the part of
  the solve that does the work.

## 3. The fix, and measured effect

Four options change together (`nlp_solver_warm_start_first_qp_from_nlp` requires
`qp_solver_cond_N == N` to take effect, per acados' own docstring):

```python
ocp.solver_options.qp_solver_warm_start = 2                 # HPIPM "hot start"
ocp.solver_options.nlp_solver_warm_start_first_qp = True
ocp.solver_options.nlp_solver_warm_start_first_qp_from_nlp = True
ocp.solver_options.qp_solver_cond_N = N
```

(`qp_solver_warm_start = 1` was checked and rejected — HPIPM's own docstring notes
level 1 is *identical* to level 0 for HPIPM, since primal variables live in delta
space and get reset to zero regardless; only level 2/3 actually reuse the previous
QP's multipliers/slacks.)

Measured on the same arc-tracking scenario as `test_mpc_core_acados.py`, comparing an
otherwise-identical solver built cold vs. with these four options
(`solver.get_stats("qp_iter")` per tick, 77 solves):

| | mean qp_iter | max qp_iter | mean solve (ms) | **max solve (ms)** |
|---|---|---|---|---|
| cold QP every tick (defaults) | 3.65 | 10 | 0.165 | 0.405 |
| warm-started (this fix) | 3.21 | 8 | 0.148 | **0.330** |

A real, if modest, reduction — about 18% off the worst-case solve time. Modest because
this problem is small and well-conditioned enough to converge in ~3-4 iterations even
cold; the fix matters more as a **closed gap** (the outer warm start now actually
reaches the solve) than as a large speedup on this particular problem size. It's the
worst-case column that matters for the original bug (doc 27's tail latency), and that's
the one that moved.

`self.last_qp_iter` (from `get_stats("qp_iter")`) is now also stored per solve for
future diagnostics — e.g. if step 4's Gazebo benchmark shows any remaining tail, this
is the first thing to check (iteration count spiking vs. something else, like the
async publish path itself).

---

## Interview angle

- **Q: Doesn't SQP_RTI already guarantee bounded time by doing one iteration?** One
  *SQP* iteration, yes — but that iteration still requires solving a QP, and the QP
  solver (HPIPM here) has its own iteration count. RTI bounds the *outer* loop; the
  inner solve still needs its own options checked (warm start, iteration cap) to
  actually be fast and consistent.
- **Q: Why does "the first QP" matter for a warm-started solver?** Because RTI solves
  exactly one QP per NLP-solve call — so under acados' accounting, every tick's QP
  literally *is* "the first QP" of its own one-iteration NLP solve, and the option
  that governs whether *that* QP warm-starts is a separate flag
  (`nlp_solver_warm_start_first_qp`) from the general `qp_solver_warm_start`, easy to
  miss if you assume warm-starting is "on" just because the outer iterate persists.
- **Q: Why not just crank `qp_solver_iter_max` down for a tighter bound?** It's
  already a hard cap (50) far above what's actually used (max 8, measured) — lowering
  it without evidence of it being hit is tuning a knob that isn't the bottleneck.
