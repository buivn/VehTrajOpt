# 23 — Multi-robot coordination roadmap: from one diff-drive to a coordinated fleet

Scoping pass, no code. This is the **orchestration-track** roadmap — a different
axis from the single-robot generation/optimization track (docs 20–22). Goal: a
phased plan to grow the existing single-robot stack into an *N*-robot coordinated
system, with the method-selection trade-offs recorded up front so the build
order isn't decided ad hoc.

> **Numbering note.** Doc 20/22 tentatively earmarked doc 23 for *collocation
> optimization* on the single-robot track. That doc isn't written yet and
> multi-robot coordination is the active interview priority (Symbotic system-design
> round, Overland fleet questions), so this took the slot. Renumber to 24+ if you
> want collocation back at 23.

> **Companion KB notes** (general theory, kept out of this repo per the
> cv_resume/robotics-kb split):
> `robotics-kb/notes/90-orchestration/mapf-solvers.md` and
> `robotics-kb/notes/90-orchestration/fleet-traffic-coordination.md`. This doc is
> the *VehTrajOpt-specific build plan*; those are the reusable mechanism notes.

---

## 0. Where the stack is today (single robot)

| Capability | Where | Status |
|---|---|---|
| Diff-drive robot in Gazebo | `vto_description`, `vto_simulation` | ✅ Phase 0 |
| Global planner (A\*) | doc 07, `vto_planning` | ✅ |
| Trackers: Pure Pursuit / MPC / MPPI | docs 05, 13, 15, `vto_control` | ✅ common `Path → cmd_vel` interface |
| Localization: AMCL + EKF fusion | docs 11, 18, 19 | ✅ |
| Benchmark harness (per-leg metrics, seed sweeps) | doc 14, `vto_bench` | ✅ |
| Maze world (procedural, seeded) | doc 06 | ✅ |

Everything is **one robot, one namespace, one costmap, one goal**. The core
`src/` planner is kinodynamic sampling for car-like/trailer models; the ROS 2
layer wraps it behind `nav_msgs/Path`.

## 1. Target

*N* diff-drive robots (start N=2, design for N≈10–20 in sim) sharing one
warehouse-style world, each receiving a stream of goals, coordinated so they
never collide and don't deadlock in the maze's single-width corridors. Reuse the
existing trackers and `vto_bench` unchanged; add the coordination layers above
them.

**Non-goals:** real hardware, >~20 robots (Gazebo won't take it), heterogeneous
robot types, a production fleet-manager UI. Optimal MAPF is explicitly *not* a
goal — bounded-suboptimal / decoupled is the target (see §3).

## 2. The layer stack, mapped to this repo

Top-down, with what already exists vs. what's new:

| Layer | Job | Exists? | New work |
|---|---|---|---|
| **Task allocation** | which robot gets which goal | ❌ | `vto_fleet` pkg: greedy + Hungarian assignment over a cost matrix |
| **Coordination / MAPF** | conflict-free paths on a shared graph | ❌ | space-time A\* + prioritised planning, then PBS/windowed |
| **Traffic / conflict** | resolve real-time contention not in the plan | ❌ | zone-mutex manager; open corridors → reactive |
| **Local planning / control** | per-robot path execution + reactive safety | ✅ trackers | multi-namespace bringup; add other robots as dynamic obstacles in the local costmap; ORCA layer |
| **Fault recovery** | robot fails / path blocked / new goal | partial (BT ideas in `robotics-kb`) | 3-layer: BT recovery → scoped replan → task reassignment |
| **Comms substrate** | keep DDS discovery cost bounded | ❌ | per-robot `ROS_DOMAIN_ID` isolation; MQTT for fleet-plane traffic |

## 3. Method selection (decisions, with the "why not")

### Coordination layer — which MAPF?

**Choice: space-time A\* per robot + prioritised planning first, then PBS, then a
windowed (WHCA\*-style) re-solve.**

- **Why not CBS/ECBS first** — optimal/bounded-suboptimal, but the two-level
  constraint-tree search is a big implementation lift and its high-level
  branching blows up exactly on the single-width-corridor conflicts the maze is
  full of. Wrong first target for a learning repo.
- **Why prioritised planning first** — `O(N)` low-level searches, no high-level
  search at all: plan robot 1 freely, robot 2 avoiding robot 1's space-time
  path, etc. Smallest thing that demonstrates the space-time reservation idea
  end to end. Incomplete (a low-priority robot can get boxed in) — that's the
  *point*, it motivates the next step.
- **Why PBS second** — searches the priority order instead of fixing it, so it
  starves far less, and one branching decision resolves a whole conflict chain.
  The natural "fix prioritised planning's starvation without CBS's blow-up"
  upgrade.
- **Why windowed on top** — bounds per-cycle compute regardless of path length
  and absorbs new goals / failed robots (you re-solve every cycle anyway). This
  is what makes it a *live* system rather than a one-shot plan.
- **JAIR tie-in** — the adaptive-path-expansion / dynamic-leading work is a
  third point on this spectrum (decoupled, but the leader/topology adapts) built
  for a constraint neither CBS nor prioritised planning models
  (team-connectivity). Worth a paragraph in the eventual doc; not a build
  target.

### Traffic layer — zones or a full reservation table?

**Choice: zone-based mutual exclusion, scoped to maze chokepoints.**

- **Why not a full space-time reservation table over every cell** — hard
  path-level guarantee, but needs tight fleet time-sync (a robot running slow
  invalidates every downstream reservation) and a central arbiter in the loop
  for every move. Over-built for a sim with wide-ish corridors.
- **Why zones** — mutex only the contended regions (corridor segments,
  intersections, goal cells); leave the rest to each robot's local planner +
  ORCA. No arbiter for the common case, no time-sync. Matches the "hand-designed
  spatial decomposition" framing in the KB note.
- **The threshold to record** — zones hold *while corridors stay uncontended*.
  If the maze + fleet density makes corridors themselves contended (measure:
  mean wait time at zone boundaries), that's the trigger to add per-zone
  windowed reservations. Make `vto_bench` log that signal.

### Task allocation — Hungarian or greedy?

**Choice: greedy nearest-goal for the live loop, Hungarian as the batch
re-optimizer and the benchmark baseline.** `scipy.optimize.linear_sum_assignment`
handles the rectangular N≠M case directly. Fold battery / priority into the cost
cell (soft) or mask to `+inf` (hard). VRP/MILP territory the moment a robot gets
a multi-stop route — out of scope. (KB: `task-assignment-hungarian.md`.)

### Collision defence — three independent layers

1. Path-level: zones / reservations (planned paths don't conflict).
2. Execution-time: each robot marks the others into its local costmap + ORCA
   reciprocal avoidance (catches control error, late obstacles).
3. Onboard: a hard stop-distance check independent of the planner.

## 4. Phased build order

Each phase is one or more numbered docs, built and benchmarked before the next —
same pace rule as the rest of the repo.

- **Phase A — 2 robots, multi-namespace bringup + space-time A\* + prioritised
  planning.** New `vto_fleet` pkg. Robots spawn in separate namespaces, each
  runs its existing tracker. A fleet node builds the shared roadmap graph, runs
  space-time A\* for robot 1, then robot 2 avoiding robot 1's reserved
  `(cell, t)` cells. Demonstrate a head-on conflict in a corridor being resolved
  by one robot waiting. *Deliverable: two robots swap ends of a corridor without
  colliding.*
- **Phase B — zone manager.** Enumerate maze chokepoints as zones with a mutex.
  Robots request/hold/release; open corridors fall through to the local planner.
  Add a global zone-acquisition order for deadlock freedom. *Deliverable: 4–6
  robots, no deadlock, `vto_bench` logs zone wait times.*
- **Phase C — PBS + windowed re-solve.** Replace fixed priority with a searched
  order; re-solve on a rolling horizon every cycle so new goals and stopped
  robots are absorbed. *Deliverable: 10 robots, continuous goal stream, no
  starvation over a long run.*
- **Phase D — task allocation.** `vto_fleet` assignment node: greedy live loop +
  periodic Hungarian batch. Cost = travel time (+ priority, + battery later).
  *Deliverable: goals dispatched to the fleet, not hand-assigned.*
- **Phase E — fault recovery.** 3-layer pipeline: per-robot BT catches a local
  stall; if unresolved, scoped replan of the affected zone; if the robot can't
  finish, reassign the task. *Deliverable: kill a robot mid-task, fleet
  recovers, its goal gets picked up by another robot.*
- **Phase F (optional) — comms substrate.** Per-robot `ROS_DOMAIN_ID`
  isolation + an MQTT fleet plane, to show the DDS O(N²) discovery argument with
  a measured before/after. Mostly an infra/interview-story phase.

## 5. Metrics — extend `vto_bench`

Reuse the per-leg harness (doc 14). What's new for the *fleet* comparison:

- **Throughput** — goals completed per minute, fleet-wide.
- **Makespan / flowtime** — max and sum of per-robot completion times for a goal
  batch (the two standard MAPF team objectives).
- **Conflict count** — vertex/edge conflicts that reached the *reactive* layer
  (i.e. the coordination layer missed them). Should trend to zero as the
  coordination method improves; non-zero is the failure signal.
- **Zone wait time** — mean/max time robots spend blocked at a zone boundary.
  This is the density-threshold signal from §3.
- **Deadlock/livelock events** — count of runs where the fleet makes no progress
  for `> T` seconds.
- **Re-solve compute** — coordination-layer solve time per cycle vs. fleet size
  (the scaling curve: prioritised vs. PBS vs. windowed).
- **Starvation** — max time any single robot waits for its next goal / for a
  path (fairness).

## 6. New package layout (proposed)

- **`vto_fleet`** — new. Task allocation (greedy + Hungarian), the coordination
  solver (space-time A\* / prioritised / PBS / windowed), the zone manager, the
  shared roadmap graph builder.
- **`vto_bringup`** — extend with `fleet.launch.py` (N robots, namespaced) and a
  fleet-world variant.
- **`vto_bench`** — extend `metrics.py` with §5.
- **`vto_control` / `vto_planning` / `vto_description`** — unchanged; the fleet
  layer sits above them. Add a "robots as dynamic obstacles" costmap layer +
  ORCA to `vto_control` in Phase A.

## Interview angle

- **Q: Which MAPF solver for a 200-robot warehouse, and why not CBS?**
  CBS is optimal but its high-level constraint-tree search branches exponentially
  in conflict count and blows up on conflict-symmetric bottlenecks (corridors,
  doorways). At fleet scale you decompose spatially (per-zone solvers), bound
  suboptimality (ECBS focal search), and window the horizon (WHCA\*), or you
  drop to PBS/prioritised planning and accept incompleteness. The design move is
  choosing a point on the coupled↔decoupled spectrum, not picking "the
  algorithm".
- **Q: Do you use a reservation table to reserve every cell+time for each
  robot?**
  Not fleet-wide. A full space-time table gives a hard path-level guarantee but
  needs tight time-sync and a live central arbiter for every move. Zone-based
  mutual exclusion reserves *regions* only where robots actually contend
  (chokepoints), leaves open corridors to reactive avoidance — cheaper, no
  arbiter in the common case, no fleet time-sync. Reservation tables come back
  bounded to one zone and a short horizon inside a windowed solve.
- **Q: How do you do spatial decomposition?**
  Static geometric cuts along the physical layout, drawn through the
  lowest-traffic points (mid-corridor, never an intersection), sized so each
  sub-solver stays real-time (~10–40 robots). Boundary crossings need a handoff
  (boundary-as-zone, or an admission protocol). Intersections become their own
  tiny reservation coordinators. A coarse region-graph routes between zones on
  top. Dynamic alternative: independence detection — cluster by actual path
  conflict each cycle.
- **Q: With zone management, where is a robot–robot conflict actually resolved?**
  Split by geometry. At a chokepoint: the zone mutex — the robots are never in
  the conflict geometry together, because reactive sensing deadlocks in tight
  space. In an open corridor: the robot's own local planner + ORCA reciprocal
  avoidance. Always underneath: an onboard safety stop independent of the
  planner. The sensor/controller on the robot owns the open-corridor case and
  the safety backstop, not the chokepoint case.
- **Q: How does the fleet keep working if part of it loses the link to the
  central scheduler?** (JAIR translation)
  Connectivity constraints in the algorithm become a degrade-gracefully
  principle in the system: a disconnected robot/sub-fleet needs enough local
  autonomy — cached task queue, local zone-based conflict resolution, a
  hold-last-safe-state BT — to keep operating safely rather than freezing.
  Zone-based traffic works here *because* it doesn't need a live central arbiter
  for the common case.

---

## Check-in

Does this phasing look right — start at 2 robots + prioritised planning, defer
PBS/windowing to Phase C — or would you rather go straight to a windowed solver?
And: keep this at doc 23 (bumping collocation to 24), or renumber?
