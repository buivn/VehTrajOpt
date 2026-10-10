# 26 — Off-road planning: method taxonomy & trade-offs

Reference doc, not a build step. Collects the comparison that came up while
building the Ackermann lattice (docs 24–25): what the main planning methods for
off-road car-like navigation are, how they decompose, and the concrete
disadvantages of each. Companion to doc 20's method × robot-model matrix — this
one goes deep on the *car-like off-road* column and on the global/local split.

---

## 1. The dominant pattern: two layers

Almost every fielded off-road stack for a car-like vehicle is:

```
   perception  ─►  traversability costmap (increasingly LEARNED)
                            │
                   ┌────────┴─────────┐
   GLOBAL layer:   kinematically-feasible route over the costmap
                   (Hybrid A* / state lattice / sampling-based)
                   slow: ~1–10 Hz or event-triggered
                            │  reference path + cost-to-go
                            ▼
   LOCAL layer:    dynamics-aware trajectory + control
                   (MPPI / NMPC), handles slip, load transfer, terrain
                   fast: 20–50 Hz
                            │
                            ▼
                     actuator commands
```

- **Global** owns *where to go* — completeness, obstacle/dead-end reasoning,
  kinematic feasibility (turning radius). Cannot see dynamics.
- **Local** owns *how to move* — dynamic feasibility (friction ellipse, rollover,
  actuator limits), terrain reaction, disturbance rejection. Horizon-limited, can
  get trapped without the global cost-to-go.

The layers are **complements, not competitors** — this is why doc 20 marks both
lattice *and* sampling as "Direct fit" for M2.

---

## 2. Global planners compared

| Method | Feasibility it guarantees | Runtime | Replanning | Main disadvantages |
|---|---|---|---|---|
| **Grid A\* / D\* Lite** (2D, position only) | none (path may be undrivable for a car) | fastest | D\* Lite = clean incremental | ignores heading + turning radius entirely; jagged output; a car often *can't* drive it |
| **State lattice** | kinematic (exact `κ ≤ κ_max`, exact node connection) | fast (precomputed primitives, pure graph search) | clean on the fixed graph (D\* variants) | resolution-complete only; frozen maneuver set; offline BVP primitive generation; sub-cell drift / hand-tuned headings; memory for the table; curvature-discontinuous with Dubins primitives |
| **Hybrid A\*** | kinematic (approx — endpoints float) | slower (continuous successors, 2× heuristics incl. a full 2D Dijkstra per replan) | graph rebuilt per query — incremental is awkward | resolution-complete only, and the one-node-per-cell dedup can *discard the better continuation*; no real optimality; very tuning-brittle; path quality lives in the CG smoother (nonconvex, can lose clearance); analytic RS-shot rarely fires in clutter |
| **Sampling-based kinodynamic** (kino-RRT\*, SST, BIT\*) | kinematic *and* (with a dynamics model) partial dynamic | anytime, but high **variance**; slow to a good-cost path | typically restart; some anytime repair | non-deterministic / non-repeatable (bad for test & certification); narrow-passage difficulty; solution quality needs long runtime; jerky raw output needs smoothing |

### Disadvantages shared by all four

- **Kinematic feasibility at best** — no tire forces, slip, rollover, load
  transfer. Needs the local layer.
- **Curse of dimensionality** — adding velocity, terrain pitch/roll, or trailer
  angles multiplies the state space.
- **Static-world assumption** — dynamic obstacles ⇒ re-search; putting time in the
  state costs a dimension.
- **Costmap-limited** — the plan is only as good as the traversability estimate.
- **Discretization artifacts** — heading bins, cell size, sample density.

### Lattice vs. Hybrid A\* — the one-line distinction

Lattice solves the exact node-to-node BVP **offline** (primitives land on nodes,
graph is fixed and reusable). Hybrid A\* forward-integrates fixed controls
**online** (endpoints float, dedup by cell, graph rebuilt per query). Lattice
trades harder offline work for a cleaner, faster, incrementally-repairable online
search; Hybrid A\* trades that for implementation simplicity and native reverse.

---

## 3. Local planners compared

| Method | How it optimizes | Strengths | Disadvantages |
|---|---|---|---|
| **MPPI** (sampling-MPC) | forward-sample thousands of control sequences, weight by exp(−cost/λ) | handles non-convex cost, discrete traversability, contact/slip; GPU-parallel; no gradients / derivatives needed | needs many samples (GPU for real-time); local optimum; sensitive to temperature λ and sampling covariance; can chatter without filtering |
| **NMPC** (iLQR / DDP / acados) | gradient/Newton on a smooth cost, constrained | fast convergence, smooth output, hard constraint handling | needs a *smooth* model + cost — chokes on discrete/non-convex traversability; needs a good initial guess; solver can fail to converge in real time |
| **Tube / robust MPC** | nominal NMPC + an invariant error tube | formal disturbance bounds | conservative; tube computation assumes a bounded, often linear, error model |
| **Geometric tracker** (Pure Pursuit / Stanley + speed law) | none — pure feedback | trivial, robust, cheap | no dynamic feasibility reasoning; only adequate for mild terrain / low speed |

MPPI and NMPC are **peers**, not a ranking — MPPI when the cost is nasty and a GPU
is available, NMPC when the model is smooth and constraints are hard.

---

## 4. The cross-cutting axis: learned traversability

Orthogonal to *which* planner: the **costmap feeding all of the above is
increasingly learned** (self-supervised from driving experience, or from
imagery/lidar semantics) rather than hand-engineered slope/roughness thresholds.
In the DARPA RACER era this is arguably a bigger determinant of off-road
performance than the choice of global search. A classical planner on a good
learned costmap beats a fancy planner on a bad hand-tuned one.

---

## 5. Decomposition variants (it's not always "kinematic global + MPPI")

| Variant | Global | Mid | Local |
|---|---|---|---|
| **Classic two-layer** | lattice / Hybrid A\* | — | MPPI or NMPC |
| **Skip the kinematic global** | D\* Lite on 2D costmap → cost-to-go | — | **long-horizon** MPPI does kinematics *and* dynamics |
| **Optimization mid-layer** | any global search (seed) | CHOMP / TrajOpt / collocation smoother | short-horizon tracker |
| **End-to-end / RL** | learned policy (imitation or RL) — replaces global+local | | |

The "skip the kinematic global" variant is common when compute allows a long MPPI
horizon — the global layer degrades to a cheap 2D cost-to-go and MPPI owns
feasibility. End-to-end learned driving exists in research (BADGR, imitation
drivers) but is rare in fielded safety-critical stacks.

---

## 6. Where this repo sits

- **M2 car-like, off-road:** classic two-layer.
  - Global: the **state lattice** we're building (docs 24–25) — chosen over
    Hybrid A\* for reuse of doc 22's fixed-graph search and a clean benchmark
    swap; chosen over sampling for determinism.
  - Local: **MPPI** (doc 15) or **MPC** (doc 13), already present.
- **Costmap:** hand-engineered for now; the learned-traversability axis (§4) is a
  Phase-3 direction, not scheduled.
- **Terrain:** 2.5D drape design in doc 25 §4; SE(3) primitives out of scope.
- Doc 21 already ruled out **DWA** for this repo (dead-end maze topology).

---

## Interview angle

- **Q: Name the main planning methods for off-road car-like navigation and how
  they fit together.** Two-layer: a global kinematic planner (Hybrid A\*, state
  lattice, or sampling-based kinodynamic) over a traversability costmap, and a
  local dynamics-aware optimizer (MPPI or NMPC) that tracks it while handling slip
  and terrain. The costmap is increasingly learned. A lighter variant drops the
  kinematic global layer and uses D\* cost-to-go + long-horizon MPPI.
- **Q: Biggest disadvantage of a state lattice?** Its maneuver vocabulary is
  frozen at build time and it's only resolution-complete — plus offline primitive
  generation is non-trivial and must be redone if the vehicle changes. Everything
  it produces is kinematic-only; dynamics are someone else's problem.
- **Q: Biggest disadvantage of Hybrid A\*?** The one-node-per-cell duplicate
  detection can throw away the better continuation, and the final path quality
  depends on a nonconvex smoothing pass rather than the search itself — so a lot
  of the search cost is only getting "close enough" for the smoother.
- **Q: Lattice vs. Hybrid A\* — when would you pick each?** Lattice when you want
  determinism, precomputation, and clean incremental replanning (D\* on a fixed
  graph). Hybrid A\* when implementation speed matters, you need reverse motion
  cheaply, or the environment varies enough that maintaining a lattice's exact-node
  assumptions is a hassle.
- **Q: Why not sampling-based for the global layer?** Non-repeatability — the same
  query gives different paths run to run, which is bad for testing, debugging, and
  certification — plus it needs long runtime to reach good-cost solutions and
  struggles in narrow passages. It wins when the state space is genuinely
  high-dimensional (trailers, full kinodynamics) where a lattice can't be built.
- **Q: MPPI vs. NMPC for the local layer?** MPPI for non-convex / discrete costs
  and contact-rich dynamics, if you have a GPU; NMPC for smooth models with hard
  constraints and tight compute. They're peers. MPPI's weakness is sample count
  and local optima; NMPC's is convergence failure on bad cost landscapes.
- **Q: What's the single highest-leverage thing in a modern off-road stack?**
  Often not the planner at all — the quality of the (learned) traversability
  costmap. A simple planner on a good costmap beats a sophisticated one on a bad
  costmap.
