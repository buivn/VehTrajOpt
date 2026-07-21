# 06 — The Maze Environment

A bounded **20 m × 30 m maze** with dense walls and narrow corridors, for testing
planning and navigation (dead ends, tight passages, no straight-line path to the
goal). Corridors are **1.5 m = 5 × the robot width** (0.30 m).

Files:
[`generate_maze.py`](../ros2_ws/src/vto_simulation/scripts/generate_maze.py) (the
generator),
[`worlds/maze.sdf`](../ros2_ws/src/vto_simulation/worlds/maze.sdf) (the generated
world),
[`maze.launch.py`](../ros2_ws/src/vto_bringup/launch/maze.launch.py).

---

## 1. Why a *generator*, not a hand-written world

The maze has **191 wall segments**. Hand-authoring that SDF would be unreadable
and impossible to tweak. Instead a small Python script *generates* the world from
a few parameters (size, corridor width, seed). Change a number, regenerate — the
`.sdf` is a build artifact, not something you edit.

> This is a common robotics pattern: **procedural world generation**. Test
> environments (mazes, warehouses, forests) are parameterized and generated, so
> you can sweep difficulty or randomize layouts for robust testing.

---

## 2. The layout

- **Bounds:** **centered on the world origin** → `x ∈ [-10, 10]`, `y ∈ [-15, 15]`
  (so the maze sits in the middle of the Gazebo view, not off in a corner).
  Pass `--origin corner` to instead put the bottom-left at (0,0).
- **Grid:** 12 columns × 18 rows of cells.
- **Corridors:** exactly **1.5 m** wide (5 × robot width).
- **Walls:** ~0.15 m thick, 0.6 m tall (tall enough for a LiDAR later).
- **Robot spawn:** bottom-left free cell, ≈ (-9.1, -14.1).

### Making it fit exactly

You can't have *both* an exact 20×30 outer box *and* a whole number of exact
1.5 m corridors with a fixed wall thickness — the arithmetic rarely lands on an
integer. The generator resolves this by **solving the wall thickness** so the grid
fills the dimension exactly:

```
n corridors + (n+1) walls = length
choose n = round((length − t₀) / (corridor + t₀))   # nearest cell count
solve   t = (length − n·corridor) / (n+1)            # exact wall thickness
```

So corridors stay **exactly 1.5 m** and the box is **exactly 20×30**; the wall
thickness comes out at 0.154 m (x) and 0.158 m (y) — a hair different per axis,
imperceptible in practice.

---

## 3. How the maze is carved: recursive backtracker

The walls form a **perfect maze** — exactly one path between any two cells (fully
connected, with dead ends). The algorithm (`carve()`):

1. Start every interior wall *present*.
2. DFS from cell (0,0): pick a random unvisited neighbor, **remove the wall**
   between them, move there, repeat. Backtrack at dead ends.
3. When done, every cell has been visited and the removed walls form the corridors.

"Perfect maze" properties, and why they suit testing:
- **Fully connected** → the robot can always reach any cell (a valid goal exists).
- **Narrow, winding, dead-ended** → forces real planning; a greedy
  drive-toward-goal fails. Exactly the stress test for Nav2 / your planner.

Reproducible: the generator uses a seeded PRNG, so `--seed 7` always yields the
same maze. Change the seed for a different layout.

---

## 4. From grid to SDF

Each remaining wall becomes an axis-aligned **box**. For performance they're all
**collision+visual pairs under one static link** (`<model name="maze"><link
name="walls">…`), rather than 191 separate models — one static body, many shapes,
cheap for the physics engine.

- **Perimeter:** 4 full-length boxes at the exact boundaries.
- **Interior vertical walls:** thickness `t_x` in x, spanning one cell pitch in y
  (segments overlap at junctions so there are no corner gaps).
- **Interior horizontal walls:** thickness `t_y` in y, spanning a pitch in x.

The world header (physics, `gz-sim` plugins, sun, ground plane) mirrors
[`empty.sdf`](../ros2_ws/src/vto_simulation/worlds/empty.sdf), including the
sensors plugin so a LiDAR will work here later.

---

## 5. Run it

```bash
# regenerate the world (host or container) — only if you change params
cd ros2_ws/src/vto_simulation
python3 scripts/generate_maze.py --out worlds/maze.sdf              # defaults: 20x30, 1.5 m
python3 scripts/generate_maze.py --seed 12 --corridor 1.2 --out worlds/maze.sdf

# then rebuild vto_simulation so the world is installed, and launch (in container):
colcon build --packages-select vto_simulation vto_bringup && source install/setup.bash
ros2 launch vto_bringup maze.launch.py                 # GUI: see the maze + robot
ros2 launch vto_bringup maze.launch.py headless:=true  # server only

# drive it (new shell): see docs/commands.md — teleop on /cmd_vel (TwistStamped)
```

Generator options: `--width --height --corridor --wall --wall-height --seed
--origin {center,corner} --out`.

---

## 6. What's next with it

- **Pure Pursuit** through the maze — but a fixed test path won't cut it; the maze
  needs a *planner* to find the route first. That's the motivation for **Nav2** (or
  bridging the C++ core planner) — global planning + local control.
- **Sensors + SLAM/localization** (Phase 2): drop a LiDAR on the robot, and the
  maze becomes a real mapping/navigation testbed.

---

## Interview angle

**Q: Why generate test worlds procedurally?**
Parameterized generation lets you sweep difficulty, randomize layouts (seeded for
reproducibility), and keep the source readable — a 191-wall SDF by hand is
unmaintainable. Standard for robotics testing.

**Q: What's a "perfect maze" and why use one for nav testing?**
A spanning-tree maze: exactly one path between any two cells, fully connected, with
dead ends. It guarantees a reachable goal while forcing genuine planning — greedy
"head toward the goal" gets trapped in dead ends.

**Q: How do you fit exact-width corridors into an exact-size box?**
Solve the wall thickness: `t = (length − n·corridor)/(n+1)` for the nearest integer
cell count `n`. Corridors stay exact; walls take up the slack.

**Q: Why one static link with many shapes instead of many models?**
One static rigid body with many collision/visual geometries is far cheaper for the
physics/rendering engine than hundreds of separate models, and it never moves.

---

## Where this maps in our repo

- Generator: [`generate_maze.py`](../ros2_ws/src/vto_simulation/scripts/generate_maze.py)
- World: [`worlds/maze.sdf`](../ros2_ws/src/vto_simulation/worlds/maze.sdf)
- Launch: [`maze.launch.py`](../ros2_ws/src/vto_bringup/launch/maze.launch.py)
  (`gazebo.launch.py` gained `world`, `x`, `y` args)
- **Next:** a planner (Nav2 / C++ bridge) to actually solve the maze.
