# 25 — Ackermann state lattice, step 2: control set + search

Step 1 ([doc 24](24-lattice-ackermann-dubins.md)) built `dubins.py` — the *edge*
primitive: the shortest bounded-curvature curve between two oriented poses. This
step turns a pile of possible edges into a **minimal control set** and runs the
search.

`vto_planning/lattice_ackermann.py` + `test/test_lattice_ackermann.py`. No ROS.

---

## 1. General idea / theoretical aspect

### From "an edge" to "the edge set"

A lattice search needs a **fixed, small** set of primitives to expand each node
with — doc 22's diff-drive lattice hand-picked 7 (straight, 4 arcs, 2 pivots,
[lattice.py:71-104](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L71-L104)).
For a car we can't hand-pick, because a primitive must **exactly** connect two
lattice nodes under the curvature bound (no pivot to fix a snapped endpoint —
[doc 24 §1](24-lattice-ackermann-dubins.md)). So we generate the set:

1. **Enumerate** candidate endpoints — every lattice node `(Δrow, Δcol, Δθ_bin)`
   within a small window of the origin.
2. **Connect** each to the origin pose `(0,0,0)` with a Dubins curve (step 1).
3. **Filter** out the long ones (a candidate whose Dubins curve loops halfway
   around the turning circle is not a useful unit move).
4. **Prune** the redundant ones — if a candidate's endpoint is already reachable
   by stringing together shorter accepted primitives at no extra cost, it adds
   nothing to what the search can express, so drop it.

What survives is the **control set** (a.k.a. minimal spanning control set): the
smallest primitive library whose compositions still reach every lattice node the
full library could, i.e. it spans the same reachable lattice but with the least
branching factor. This is the classic Pivtoraiko & Kelly construction.

### Why pruning matters

A\*'s cost is `O(b^d)` in the branching factor `b`. Every redundant primitive is a
duplicate successor at every single node expansion, for the whole search. Halving
`b` is worth far more than the one-time generation cost. Doc 22 §"Complexity"
already noted the diff-drive lattice keeps `b` at ~5–7; for the car we *earn* a
small `b` by pruning rather than getting it for free from a hand-picked set.

### Where it sits

Still **global generation**, same pipeline slot as A\* and the diff-drive lattice.
Output is a `nav_msgs/Path` (once the step-3 node wraps it); a controller
(Pure Pursuit / MPC / MPPI) tracks it.

---

## 2. Method selection — the design choices

### Enumerate-and-prune vs. hand-designed vs. Hybrid-A\*-style fixed inputs

- **Hand-designed control set** (what doc 22 did): fine when the model can pivot
  and snap. A car can't — a hand-picked arc almost never lands exactly on a
  `(Δrow, Δcol, Δθ_bin)` node, so the primitives wouldn't compose. Rejected.
- **Hybrid-A\*-style fixed control inputs** (integrate `{left, straight, right}`
  for a fixed time, endpoints float): avoids the exact-connection problem, but the
  search state goes continuous and doc 22's exact-node machinery
  ([lattice.py:107-217](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L107-L217))
  no longer applies — see [doc 24 §2](24-lattice-ackermann-dubins.md). Rejected for
  *this* step to keep the reuse; a genuine option for a separate build.
- **Enumerate every lattice endpoint + connect with Dubins + prune** (chosen):
  every primitive lands exactly on a node by construction (Dubins connects any two
  poses exactly), so doc 22's `build_primitive_table` / `lattice()` run **unchanged**
  on the result. The generation cost is a few hundred closed-form Dubins calls at
  startup — trivial.

### Pruning: full path-decomposition vs. our bounded version

Pivtoraiko & Kelly prune a primitive iff its *entire swept path* is reproducible
by a sequence of shorter primitives (a full path-decomposition test, recursively).

We do a **bounded** version: drop a primitive iff its *endpoint node* is reachable
by composing at most `max_combo_depth` (default 3) shorter accepted primitives for
total cost `≤` this primitive's cost `× (1 + tol)`. This is weaker — it can keep a
primitive whose endpoint needs 4+ pieces, and it matches on endpoint not on swept
shape — but it catches the dominant redundancy (near-collinear straights, an arc
equal to two half-arcs) and it's simple and fast. Same spirit as doc 22's honest
"radii are chosen, not solved for" simplification. Documented, not hidden.

### Reuse `lattice()` unchanged — the primitive contract

`make_ackermann_primitive_library()` emits exactly the shape
`make_primitive_library()` does — `list[dict(dh, cost, samples)]`, `samples` a list
of `(Δrow, Δcol, Δθ)` offsets from the origin at heading 0
([lattice.py:88-104](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L88-L104)).
Then `build_primitive_table()` rotates it to every heading and `lattice()` searches
it, byte-for-byte the same code path as the diff-drive lattice. **Benefit:** the
step-3 node, `densify_path`, `_primitive_is_clear`, and the whole `vto_bench`
harness need zero changes — only the primitive library differs. This is the same
"swap one component, compare pipelines" property doc 22 §3 built for.

### Heuristic caveat (noted, deferred)

`lattice()`'s heuristic is position-only octile
([lattice.py:178-182](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L178-L182)).
For a curvature-bounded robot the *true* cost-to-go is the **Dubins distance**
(≥ Euclidean, accounts for turning radius + final heading) — this is exactly
Hybrid A\*'s "non-holonomic-without-obstacles" heuristic. Using it here (max'd with
octile) would both tighten the search and fix octile's mild inadmissibility on
off-axis goals. `dubins_path_length()` already exists for it. Deferred to keep this
step a pure drop-in; flagged as the obvious next optimization.

### Sub-cell drift at primitive joins (known cost of node reuse)

A primitive's true Dubins endpoint isn't exactly at an integer cell; `lattice()`
composes the next primitive from the *rounded* cell
([lattice.py:203](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L203)). So a
few-primitive path accumulates sub-cell heading/position drift. Bounded (< 0.5 cell
per join), acceptable for a *reference* path that MPC/MPPI re-optimizes, and the
price of reusing an integer-node search instead of a continuous one. A full lattice
(Pivtoraiko) picks heading bins so Dubins connects integer-to-integer exactly; we
don't. Documented.

---

## 3. Coding — design of `lattice_ackermann.py`

### `make_ackermann_primitive_library(turning_radius_cells, max_reach_cells, ...)`

Returns `list[dict(dh, cost, samples)]`.

- **Candidate enumeration** — triple loop over `Δrow, Δcol ∈ [-R_reach, R_reach]`,
  `Δθ_bin ∈ range(N_HEADINGS)`; skip the origin. Each → a `dubins_shortest_path`
  from `(0,0,0)` to `(Δrow, Δcol, index_to_heading(Δθ_bin))`, in cell units,
  `radius = turning_radius_cells`. Data structure: a plain `list` of
  `(length, dr, dc, dh, path)` tuples — built once, sorted once, discarded after.
- **Length filter** — drop candidates with `path.length > max_primitive_len_cells`
  (default `1.1 · π · R` — allows a U-turn primitive, excludes full loops).
- **Greedy prune** — sort candidates by length ascending; for each, run
  `_reachable_by_combo` against the already-accepted set; accept iff not reachable
  within budget. Shortest-first ordering means the primitives that *do* the
  reaching are accepted before the ones they'd make redundant.
- **Sampling** — accepted primitive's `path` → `sample_dubins(path, step)` with
  `step = 1 / samples_per_cell` (default ~2.5 samples/cell — denser than doc 22's
  fixed 6 because primitives vary in length, and `_primitive_is_clear` rounds each
  sample to a cell). Store as `(Δrow, Δcol, Δθ)` offsets.
- **Cost** — `path.length` (execution-time proxy; for a car, ~proportional to
  length at constant speed, unlike the diff-drive case where pivots broke that).

### `_reachable_by_combo(accepted, tgt, budget, max_depth)`

A small **Dijkstra over compositions** of the accepted primitives:

- Nodes = integer lattice states `(dr, dc, dh)` reachable from `(0,0,0)`.
- Edge = apply an accepted primitive, rotating its `(dr, dc)` by the current
  heading (same rotation `primitives_at_heading` uses,
  [lattice.py:113-121](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L113-L121)),
  rounding to integers.
- `heapq` frontier keyed by cumulative cost; prune branches once cost `> budget`
  or depth `≥ max_depth`; hard cap on expansions (return `False` = keep the
  primitive, the conservative choice) so generation can't blow up.
- Returns `True` iff `tgt` is reached for `≤ budget`.

Why Dijkstra and not plain BFS: the test is about *cost*, not reachability —
"is there a cheaper-or-equal way already".

### `ackermann_primitive_table(**kwargs)` and `plan_ackermann(grid, start, goal, ...)`

Thin ergonomics:
`ackermann_primitive_table = build_primitive_table(make_ackermann_primitive_library(...))`
and `plan_ackermann` = build-table-if-needed + call `lattice()` with a
`(row, col, heading_index)` start and `(row, col)` goal. Keeps the step-3 node and
the tests a one-liner.

### Complexity

Generation: `|candidates| = (2R_reach+1)² · N_HEADINGS` Dubins calls (O(1) each) +
`|survivors|` bounded combo-Dijkstras — all at startup, ≪ 1 s. Search: unchanged
from doc 22 — standard A\* over `(row, col, heading)` with branching factor =
`|control set|`.

**Actual counts** (as generated): `R = 4` cells, `reach = 7` → **15** canonical
primitives (1 straight, ±22.5/45/67.5/90° turns, plus a pair of Δθ=0 S-curve
"lane-change" primitives the pruner keeps because integer rounding makes them
non-composable from the single arcs). `build_primitive_table` expands that to
`16 headings × 15 = 240` table entries; branching factor per node expansion is
**15**. Test config (`R = 3`, `reach = 5`) → **9**.

---

## 4. Terrain and the 2.5D extension (design, not built yet)

Everything above assumes motion in a flat plane. On real off-road terrain
(start and goal at different elevation, slopes, ridges) the lattice **stays 2D** —
`(row, col, heading)` — but terrain enters as **cost and feasibility at search
time**. The homogeneity that lets one primitive table be reused everywhere
(rotate + translate, [lattice.py:107-122](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L107-L122))
is only exactly true on a plane; the standard workaround is **2.5D** — a 2D
lattice plus an elevation layer the primitives are "draped" onto.

### What stays, what changes

- **`prim["cost"]` stays the terrain-free base cost** — pure geometry (Dubins
  length + any comfort penalty), precomputed once, still reusable at every
  heading and map location.
- **A new precomputed field `prim["cells"]`** — the primitive's swept-cell list
  as deduplicated integer `(Δrow, Δcol)` offsets with the arc length `ds`
  traversed in each, rasterized once inside `build_primitive_table` from the
  `samples` trace. This is what makes per-expansion terrain cost cheap.
- **The search adds a terrain term per expansion** (the same place
  `lattice()` already adds a per-cell `penalty(nr, nc)`,
  [lattice.py:211](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L211)):

  ```
  edge_cost = prim["cost"]
            + Σ over prim["cells"] of  T[r+Δr][c+Δc] · ds        # traversability
            + w_climb · max(0, h[end] − h[start])                # net climb (asymmetric)
  ```

- **Feasibility gate folded into the same cell loop** — reject the primitive
  (early-exit, like `_primitive_is_clear`,
  [lattice.py:135-142](../ros2_ws/src/vto_planning/vto_planning/lattice.py#L135-L142))
  if any swept cell exceeds max grade, or the side-slope roll estimate exceeds
  the rollover margin, or a descent grade exceeds the braking limit.

### Why this is efficient

- **Terrain-derived layers are computed once per costmap update** (1–10 Hz in the
  node, numpy-vectorized), *not* per expansion:
  elevation `h`, slope magnitude `|∇h|` (finite differences), side-slope roll,
  roughness, and the fused per-cell traversability cost `T`. The search only ever
  *reads* `T` and `h`.
- **Per expansion the terrain cost is O(cells per primitive) ≈ 10–25 array reads**
  — the same order as the collision check that already runs, and it shares the
  loop (touch each cell once for both feasibility and cost).
- **Climb is an O(1) endpoint lookup** `h[end] − h[start]`, not an integral along
  the path — the net height change is all that matters for work done, and
  `max(0, ·)` makes climbing cost more than descending refunds.
- No per-state caching needed: gathering ~20 cells is already cheaper than a
  cache lookup + invalidation on costmap update.

### When the 2.5D drape is no longer enough

Draping a flat primitive onto steep terrain (> ~15–20°) mis-estimates its arc
length and effective curvature (the turn really happens in the terrain's tilted
tangent plane), and ignores wheel slip. Past that, either generate true SE(3)
primitives (heading + pitch, primitive count multiplies — heavy) or hand the
dynamics to the terrain-aware local layer (MPPI with a 3D vehicle model). For a
ground vehicle you still never put `z` in the search state — it is pinned by
`(row, col)` via the elevation map, a dependent lookup, not a searched DOF.

---

## Interview angle

- **Q: What is a "minimal spanning control set" and why bother?** The smallest
  primitive library whose compositions still reach every lattice node the full
  candidate set could. Bother because A\*'s cost is exponential in branching
  factor — every redundant primitive is a duplicate successor at every expansion
  for the entire search; the one-time pruning pays for itself many times over.
- **Q: How do you generate primitives that land exactly on lattice nodes for a
  car?** Enumerate the target nodes, then solve the exact boundary-value problem
  to each with a Dubins (or Reeds–Shepp) curve — those connect any two poses
  exactly under the curvature bound. Contrast the diff-drive lattice, which picks
  arcs freely and relies on an in-place pivot to absorb the endpoint mismatch.
- **Q: Your pruning matches on endpoint, not swept path — what's the risk?** It can
  keep a primitive that's geometrically redundant (its curve equals two others'
  concatenation) as long as the *cheaper composition reaches a different node*, and
  it can miss redundancy that needs >`max_depth` pieces. Trade-off: full
  path-decomposition pruning (Pivtoraiko) is stronger but much heavier; the bounded
  endpoint test catches the common cases cheaply.
- **Q: Why can you reuse the diff-drive lattice's search verbatim?** Because the
  only thing that changed is the primitive library; the search state
  `(row, col, heading)`, the rotate-to-every-heading table, collision-checking, and
  path reconstruction are all primitive-agnostic. That's a deliberate interface
  contract, and it's what lets the benchmark swap one planner for another with no
  other change.
- **Q: What heuristic should a car-like lattice use, and why isn't Euclidean
  enough?** The Dubins (or Reeds–Shepp) distance to goal — it accounts for the
  turning radius and the required final heading, which Euclidean ignores, so it's a
  much tighter admissible estimate near the goal. It's identical to Hybrid A\*'s
  non-holonomic-without-obstacles heuristic.
- **Q: On off-road terrain, does the primitive library or the search hold the
  terrain cost?** The library holds only the terrain-free base cost (geometry,
  reusable at every heading and location); the terrain increment is added per node
  expansion, because the same primitive costs differently at different places on
  the map. Same hook `lattice()` already uses for its per-cell clearance penalty.
- **Q: How do you compute a curved primitive's terrain cost without killing the
  search rate?** Precompute the rasterized swept-cell list once per
  (primitive, heading); precompute the traversability + elevation layers once per
  costmap update; at expansion, gather the layer over ~10–25 cells in one pass
  shared with the collision check. Get climb from the endpoint elevation
  difference, O(1), not by integrating elevation along the path.
- **Q: Why not add elevation to the search state off-road?** For a ground vehicle
  `z` is not a free DOF — it's `terrain(x, y)`, a dependent lookup. Adding it would
  multiply the state space to represent zero extra choice. (A UAV/AUV is different:
  there `z` is independent and the lattice genuinely becomes 3D / SE(3).)

---

## Check-in

After you've read this and the code:

1. Default `turning_radius_cells` — I've left it a **parameter** (the step-3 node
   converts `R_min` metres ÷ map resolution). For host tests I use `R = 4` cells.
   Reasonable, or do you want a specific `R_min` in metres pinned now?
2. Pruning: depth-4 combo + endpoint-within-1-cell + 15% cost slack — accept the
   simplification (documented in §2), or do you want the full Pivtoraiko
   swept-path-decomposition prune?
3. Terrain / 2.5D (§4) — design only for now; build it as part of step 3 (the node
   owns the costmap), or leave it until after the flat-ground pipeline runs?
4. Next: **step 3** (`lattice_ackermann_planner_node.py`, the ROS drop-in), or
   pause to eyeball the generated control set first?
