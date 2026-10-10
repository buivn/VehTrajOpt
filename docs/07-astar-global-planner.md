# 07 — A\* Global Planner (design)

**Status:** design only — no code yet. This doc is the "explain before we build" step.
**Goal:** a global planner that takes the maze + a goal and produces a collision-free
`nav_msgs/Path` on `/plan`, which the existing Pure Pursuit follower already consumes.

---

## 0. Where this sits in the pipeline

```
        ┌──────────────┐   nav_msgs/Path    ┌───────────────┐  TwistStamped   ┌────────┐
 map +  │  A* PLANNER  │  ───/plan────────► │ PURE PURSUIT  │ ──/cmd_vel────► │ ROBOT  │
 goal ► │  (this doc)  │                    │  (Phase 1 ✅) │                 │ (Gazebo)│
        └──────────────┘                    └───────────────┘                 └────────┘
        global: "which way?"                local: "what wheel commands?"
```

We are building the **global planner** box. The follower already exists and speaks
`nav_msgs/Path`, so the planner's *only* job is: search the map, output a Path in the
same frame. Clean seam — nothing downstream changes.

---

## 1. General idea / theory

### 1.1 Configuration space → occupancy grid
The planner searches the robot's **configuration space**. For a first planner we use the
simplest useful C-space: **2D position (x, y)**, ignoring heading. We discretize it into an
**occupancy grid** — a 2D array where each cell is `free` or `occupied`. Our maze is *already*
a grid (recursive-backtracker, 1.5 m corridors), so this representation loses almost nothing.

Two coordinate systems, and converting between them is a recurring source of bugs:
- **World frame** — meters, continuous, origin at maze center (`--origin center`).
- **Grid frame** — integer cell indices `(row, col)`. `world = origin + (col, row) · resolution`.

### 1.2 Graph search: Dijkstra → A\*
Treat each free cell as a **graph node**; edges connect neighbors (4-connected = N/S/E/W, or
8-connected = add diagonals). We want the least-cost path from start to goal.

- **Dijkstra** explores outward by cheapest-cost-so-far `g(n)`. Correct but *uninformed* — it
  expands in all directions equally, wasting work.
- **A\*** = Dijkstra + a **heuristic** `h(n)` estimating remaining cost to goal. It expands by
  `f(n) = g(n) + h(n)`, so it's *pulled toward* the goal and explores far fewer cells.

`g(n)` = actual cost from start to `n`. `h(n)` = estimated cost from `n` to goal.

**Admissibility** (the key theoretical property): if `h(n)` **never overestimates** the true
remaining cost, A\* is guaranteed to return the **optimal** path. For a grid:
- 4-connected → **Manhattan** distance is admissible.
- 8-connected → **Euclidean** or **octile** distance is admissible (Manhattan would overestimate).

### 1.3 Clearance-aware cost (your advisor's idea)
Plain A\* hugs walls — the shortest path clips corners, which is dangerous for a real robot in
a 1.5 m corridor. We add a **clearance penalty** to the step cost so the search *stays away
from walls* (confirmed direction):

```
step_cost(a → b) = geometric_dist(a, b) + w · clearance_penalty(b)
clearance_penalty(cell) = large when close to an obstacle, ~0 when far
```

`clearance_penalty` comes from a **distance transform**: for every free cell, precompute the
distance to the nearest obstacle, then map "small distance → big penalty." This is the same
concept as a **costmap inflation layer** in Nav2. The weight `w` trades path length against
safety margin. Result: paths ride the **center of corridors** instead of scraping walls.

> Note: this keeps A\* admissible only if we're careful — the penalty raises `g`, and the
> heuristic must still not exceed the *true* (penalized) cost-to-go. In practice we keep `h`
> as pure geometric distance (a lower bound), which stays admissible.

#### Tuning: weight (strength) vs. inflation_radius (reach) — the non-obvious part
`penalty = w · max(0, R − dist_to_wall)` is a **truncated linear cost well** of width
`R` beside each wall. The two knobs do *different* jobs:
- **`w` (clearance_weight)** — how *deep* the well is (how hard the path is pushed out).
- **`R` (inflation_radius)** — how *far* the well reaches.

Beyond `R` cells from a wall the penalty is **flat zero** — no gradient. So if `R` is
smaller than half the corridor, the middle is a zero-cost plateau and raising `w`
does **nothing** to centre the path (it just pins it to the edge of the band). To
centre, the wells from *both* walls must overlap → **`R ≥ half the corridor width`**.
Measured on our maze (1.5 m = 15 cells @ 0.1 m, start→far-corner), min distance kept
from any wall:

| params | min wall dist | avg |
|--------|--------------|-----|
| w=3, R=3 | 0.30 m | 0.40 m |
| w=**20**, R=3 | 0.30 m | 0.40 m *(weight alone: no change!)* |
| w=5, **R=7** | **0.70 m** | 0.73 m *(centred; ideal ~0.75)* |

Chosen defaults: **`w=5`, `R=7`**. Cost: the centred path is ~10 % longer (it stops
cutting corners) — the intended trade. This is exactly a **costmap inflation layer**.

---

## 2. Method selection (recap)

Decided in the last session: **grid A\* first**, because the maze *is* a grid, A\* is optimal +
resolution-complete here, it's a low-risk self-contained node, and it's the highest-value
interview topic. The advisor's sampling-based kinodynamic planner comes **after** as a Phase-1.5
comparison (grid search vs. sampling on the same maze). Not either/or — sequenced.

---

## 3. Coding design

### 3.1 New package: `vto_planning`
A new `ament_python` package mirroring `vto_control`, so planners live together as we add more
(A\* now; the C++ bridge and, later, Hybrid A\* / D\* Lite). Layout:

```
ros2_ws/src/vto_planning/
├── package.xml
├── setup.py            # entry_points: astar_planner = vto_planning.astar_planner_node:main
├── setup.cfg           # install_scripts=$base/lib/vto_planning   ← the Phase-1 gotcha!
├── resource/vto_planning
└── vto_planning/
    ├── __init__.py
    ├── astar.py                # PURE algorithm — no ROS imports
    └── astar_planner_node.py   # ROS wrapper (Node)
```

**Key design decision — separate the algorithm from ROS.** `astar.py` has *zero* ROS
dependencies: it takes a grid + start/goal cells and returns a list of cells. That means we can
**unit-test it on the host** (no rebuild, no container, no Gazebo) and reuse it anywhere. The
ROS node is a thin adapter: subscribe → convert to cells → call `astar()` → convert back →
publish. This "keep the algorithm framework-free" split is a design point interviewers probe.

### 3.2 The core data structures (in `astar.py`)

| Structure | Type | Why this one |
|-----------|------|--------------|
| **Occupancy grid** | 2D `numpy` array, `0`=free `100`=occupied | O(1) neighbor lookup; matches `nav_msgs/OccupancyGrid` layout |
| **Clearance grid** | 2D `numpy` float array (distance transform) | Precomputed once → O(1) penalty per cell during search |
| **Open set** | **min-heap** via `heapq`, entries `(f, tie_count, cell)` | Always pop the lowest-`f` node in O(log n). `tie_count` (a monotonic counter) breaks `f`-ties so we never compare raw cells |
| **`came_from`** | `dict[cell → parent_cell]` | Reconstruct the path by walking parents backward from goal |
| **`g_score`** | `dict[cell → float]` | Best-known cost to each cell; lets us *relax* edges (update if we find a cheaper route) |
| **`closed`** | `set[cell]` | Cells already finalized — never reopen, prevents reprocessing |

`cell` = a `(row, col)` **tuple** (hashable → usable as dict/set keys; that's why tuples, not lists).

### 3.3 The algorithm (pseudocode)
```
def astar(grid, clearance, start, goal, w):
    open   = [(h(start,goal), next(counter), start)]   # heap
    g      = {start: 0}
    came   = {}
    closed = set()
    while open:
        _, _, cur = heappop(open)
        if cur == goal: return reconstruct(came, cur)
        if cur in closed: continue          # stale heap entry
        closed.add(cur)
        for nb in neighbors(cur, grid):     # skips occupied + out-of-bounds
            tentative = g[cur] + step_cost(cur, nb, clearance, w)
            if tentative < g.get(nb, INF):  # relax
                g[nb]    = tentative
                came[nb] = cur
                heappush(open, (tentative + h(nb, goal), next(counter), nb))
    return None                             # no path (goal unreachable)
```
Note we don't decrease-key the heap (Python `heapq` can't); we push a duplicate and skip stale
pops via the `closed` check. Standard, simple, correct.

### 3.4 The ROS node (`astar_planner_node.py`, class `AStarPlanner(Node)`)
Responsibilities (the adapter layer):
- **Map in.** *Design choice to confirm* — see §3.5.
- **Goal in.** Subscribe `/goal_pose` (`geometry_msgs/PoseStamped`) — this is exactly what
  RViz's **"2D Nav Goal"** button publishes, so you click a goal in the maze interactively.
- **Start.** The robot's current pose, from TF (`map`→`base_link`) or `/odom`.
- **Convert** world start/goal → grid cells (`world_to_grid`), run `astar()`, convert the cell
  list → `nav_msgs/Path` (`grid_to_world`), stamp with the map frame, publish on `/plan`.
- **(Optional) smoothing** — collinear-point pruning or a shortcut pass, so Pure Pursuit gets a
  cleaner path. Can defer to a follow-up.

### 3.5 Map source — one decision to make
A\* needs an occupancy grid. Options, in order of ROS-idiomatic-ness:

- **(A) `map_server` publishes `/map`** (`nav_msgs/OccupancyGrid`); node subscribes. The
  standard Nav2 way. Needs us to emit a `map.pgm` + `map.yaml` from the maze — cleanest
  long-term, tiny bit of plumbing now.
- **(B) Node loads a grid file directly** via a parameter (e.g. a `.npy`/`.pgm`). Simplest;
  fewer moving parts; less "real ROS."
- **(C) Extend `generate_maze.py`** to *also* emit the occupancy grid alongside `maze.sdf`, so
  the SDF the robot drives in and the grid the planner searches are provably the same maze.

**Recommendation: (C) + (A)** — have the generator emit both `maze.sdf` and `map.pgm/.yaml`
(single source of truth, no drift), then serve it with `map_server`. It's the honest ROS
pipeline and sets up Nav2 later. I'll confirm before building.

---

## 4. Build order (next steps, one per session)
1. **`astar.py`** — pure algorithm + a host unit test on a tiny hand-made grid. *(smallest, testable, no ROS)*
2. **Map emission** — extend `generate_maze.py` to output `map.pgm/.yaml`; eyeball it.
3. **`astar_planner_node.py`** — the ROS wrapper + package scaffolding.
4. **Integrate** — launch maze + planner + Pure Pursuit; click a goal in RViz; watch it solve.
5. **Clearance term + smoothing** — tune `w`, confirm paths center in corridors.

---

## Interview angle

- **Q: Dijkstra vs. A\*?** A\* adds an admissible heuristic `h`; it expands by `f=g+h` instead
  of `g`, so it's goal-directed and expands far fewer nodes. With `h=0`, A\* *is* Dijkstra.
- **Q: What makes a heuristic admissible, and why care?** It never overestimates true
  cost-to-go → A\* returns the optimal path. Overestimating can be faster but loses optimality
  (that's Weighted A\*). *Consistency* (triangle inequality) is stronger and lets you skip
  reopening closed nodes.
- **Q: 4- vs 8-connectivity, and the matching heuristic?** 4-conn → Manhattan; 8-conn →
  octile/Euclidean. Using Manhattan with diagonals overestimates → breaks optimality.
- **Q: Why a heap for the open set?** Need repeated extract-min; heap gives O(log n) push/pop
  vs O(n) for a scan. The tie-breaker counter avoids comparing unorderable cell tuples.
- **Q: Why the `closed` set / how handle stale heap entries?** Python `heapq` has no
  decrease-key, so we push duplicates and skip any pop already in `closed`.
- **Q: Clearance cost — how, and does it break admissibility?** Distance-transform penalty
  added to step cost pushes paths to corridor centers (costmap inflation). Keep `h` a pure
  geometric lower bound so A\* stays admissible.
- **Q: When would you NOT use A\*?** High-dimensional / kinodynamic C-spaces where discretizing
  every dimension explodes — reach for sampling planners (RRT\*/PRM). (→ the advisor's planner.)
- **Q: A\* gives a geometric path — how do you make it drivable for a non-holonomic robot?**
  Post-smoothing + a tracking controller, or plan in (x,y,θ) with **Hybrid A\*** so motions
  respect the turning radius from the start.
