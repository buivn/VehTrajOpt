# 15 — MPPI controller (design)

Model Predictive Path Integral control — the third follower (C3), to compare against
Pure Pursuit (C1) and MPC (C2) on the same `Path → cmd_vel` interface.

---

## 0. Where it sits

Drop-in, same seam (so it A/Bs on the same benchmark tour):
```
 /plan (map) ─┐
 TF map->base ─┼─► MPPI (sample K rollouts, cost-weighted average) ─► /cmd_vel
 /odom twist  ─┘
```

---

## 1. Theory — MPPI

MPPI is **sampling-based MPC**. Same receding horizon as MPC, but instead of *solving*
an optimization it **samples**:

Each control tick:
1. Start from a **nominal** control sequence `U = [u₀…u_{N-1}]` (last tick's, shifted).
2. Draw **K** noisy variants: `Uᵏ = U + εᵏ`, `εᵏ ~ N(0, Σ)` (K = e.g. 1000).
3. **Roll out** each `Uᵏ` through the unicycle model → K predicted trajectories.
4. **Score** each rollout: `Sᵏ = Σ (state cost + control cost)`.
5. **Weight** by the path-integral / softmax rule:
   `wᵏ = exp(−(1/λ)(Sᵏ − min S)) / Σⱼ …`
6. **Update** the nominal by the cost-weighted average: `U ← Σ wᵏ Uᵏ`.
7. **Apply `u₀`**, shift `U`, repeat.

**Path-integral intuition:** the optimal control is an *expectation over trajectories*
weighted by `exp(−cost/λ)` — good (low-cost) rollouts pull the average toward them. `λ`
(temperature) sets greediness: small λ → the best rollout dominates (peaky); large λ →
smooth averaging over many.

**Why it's powerful:** no gradients — the cost can be **non-convex / non-differentiable**
(a hard collision indicator, a discrete "which corridor" penalty), and the dynamics can be
anything you can *simulate forward*. And the K rollouts are **independent → embarrassingly
parallel → GPU** (the reason the project keeps a GPU for MPPI).

---

## 2. Method selection

| | Pure Pursuit | MPC | **MPPI** |
|---|---|---|---|
| how it decides | geometric carrot | **gradient** NLP (IPOPT) | **sample + weighted average** |
| cost must be | — | smooth/differentiable | **anything** (nonconvex/nondiff OK) |
| constraints | none | hard (in solver) | **soft** (via cost penalties) |
| parallel? | trivial | no (sequential solve) | **yes → GPU** |
| compute | µs | ~20 ms (one solve) | K rollouts (parallel) |

**Why MPPI here:** completes the controller trilogy; showcases *sampling* vs *gradient*
optimal control; and its nonconvex-cost freedom lets us later drop a collision/clearance
cost straight in. **Why not for everything:** stochastic (needs many samples for good
coverage), only *soft* constraints (can violate without care), and three knobs to tune
(λ, Σ, K).

**Implementation choice — CPU numpy first.** Vectorise the K rollouts as numpy array ops
(K≈1000, N≈20 → a few ms/tick); framework-free + host-testable, no new deps. **GPU
(torch/cupy) is the scaling follow-up** if we push K to 10⁴⁺ or add heavy costs.

**Apply the AMCL lesson (docs/14):** sample **forward-only** — clip `v ≥ 0` on the
sampled controls — so MPPI doesn't reverse/spin its way into localization trouble like the
first MPC did.

---

## 3. Design (code)

Same framework-free split as `astar.py` / `pursuit_core.py` / `mpc_core.py`:

- **`mppi_core.py`** (numpy, no ROS): holds the nominal `U`; `control(state, ref)`:
  samples `K` noisy sequences, rolls them out (unicycle, **vectorised over K**), scores
  vs the reference window, softmax-weights, updates `U`, returns `(v, ω) = u₀`.
  Warm-started by shifting `U` each tick. Params: `K, N, dt, lambda, sigma_v, sigma_w,
  v_min(=0), v_max, w_max`, cost weights `(w_pos, w_theta, w_ctrl)`.
- **`mppi_controller_node.py`**: identical interface to the others — `/plan`, TF
  `map→base` pose, `/odom` twist; builds the N-point reference via
  `pursuit_core.sample_reference`; publishes `/cmd_vel` + `/controller/compute_ms`.

Data structures per tick: `U` (N×2 nominal), `eps` (K×N×2 noise), `X` (K×(N+1)×3 rollout
states), `S` (K, costs), `w` (K, weights). All numpy; the rollout loops over N (time) with
each step a vectorised op over the K samples.

---

## 4. Build plan (one step at a time)

1. **`mppi_core.py`** + host test — track a straight + arc, confirm forward-only, sane
   controls, cost-weighted convergence.
2. **`mppi_controller_node.py`** + entry point.
3. **Launch** — extend `controller:=pursuit|mpc|mppi`.
4. **Benchmark** the same tour → fill the report's **A\*×MPPI (C3)** cell; compare all three.

---

## Interview angle

- **Q: MPPI vs MPC?** Both receding-horizon; MPC finds the control by *gradient
  optimization* (needs a smooth cost, gives a local optimum), MPPI by *sampling many
  rollouts and taking a cost-weighted average* (handles nonconvex/nondiff costs, GPU-
  parallel, but stochastic and only soft constraints).
- **Q: What is the temperature λ?** It sets how sharply the softmax favors low-cost
  rollouts: small λ → nearly argmin (greedy, high-variance); large λ → smooth averaging.
- **Q: Why is MPPI a good fit for GPUs?** The K rollouts are independent — sample, roll
  out, and score them all in parallel; only the weighted average is a reduction.
- **Q: How does MPPI handle constraints / obstacles?** As **costs** (soft) — e.g. a large
  penalty or indicator when a rollout hits an obstacle; no hard feasibility guarantee, so
  you weight/tune to keep violating rollouts from dominating.
- **Q: MPPI weakness?** Sample efficiency (needs many rollouts), tuning (λ, Σ, K), and
  soft-only constraints — a rare unlucky sample set can produce a poor command.

---

## References

*(Standard references for the methods above — verify exact bibliographic details before formal citation.)*

**MPPI / path-integral control**
- G. Williams, A. Aldrich, E. Theodorou, "Model Predictive Path Integral Control:
  From Theory to Parallel Computation," *J. Guidance, Control, and Dynamics*, 2017.
- G. Williams et al., "Aggressive Driving with Model Predictive Path Integral Control,"
  *ICRA*, 2016.
- G. Williams et al., "Information-Theoretic Model Predictive Control: Theory and
  Applications to Autonomous Driving," *IEEE Trans. Robotics*, 2018.
- H. J. Kappen, "Path Integrals and Symmetry Breaking for Optimal Control Theory,"
  *J. Statistical Mechanics*, 2005. (path-integral control foundation)
- E. Theodorou, J. Buchli, S. Schaal, "A Generalized Path Integral Control Approach to
  Reinforcement Learning" (PI²), *JMLR*, 2010.

**MPC (for the comparison)**
- J. B. Rawlings, D. Q. Mayne, M. Diehl, *Model Predictive Control: Theory, Computation,
  and Design*, 2nd ed., 2017.
