# 12 — Making localization take effect: the map-frame refactor

Wiring AMCL in (doc 11) is necessary but not sufficient. This is the change that makes
the robot *actually* benefit from it: moving the planner and follower off the odom-frame
shortcut and onto the **`map` frame via TF**.

---

## 1. Why AMCL did nothing until now

AMCL publishes its correction as the **`map→odom`** transform. But the Phase-1.5 loop
ran entirely in **odom**:
- the planner converted poses with a *static spawn offset* and published `/plan` in `odom`;
- Pure Pursuit read the robot pose straight from `/odom` and followed the odom path.

A control loop that never references the `map` frame **cannot see** a correction that
lives in `map→odom`. So the robot kept driving to the drifting odom goal — AMCL running,
but ignored. **The fix: express the start pose, the path, and the follower's pose all in
`map`.** Then, as odom drifts, AMCL adjusts `map→odom`, the robot's *map* pose stays
true, and both nodes track it correctly.

---

## 2. The two changes

### Planner (`astar_planner_node.py`) — and it got *simpler*
- **Start pose:** `tf_buffer.lookup_transform("map", "base_footprint", …)` — the
  AMCL-corrected pose — instead of `/odom` + a static spawn offset.
- **Output:** publish `/plan` in the **`map`** frame (it already planned on the map grid;
  we just stopped converting the result back to odom).
- **Deleted:** the `spawn_x/y/yaw` params and the `_world_from_odom`/`_odom_from_world`
  helpers, and the `/odom` subscription. Localization owns `map→odom` now; the kludge is gone.

### Follower (`pure_pursuit_node.py`)
- **Pose:** `lookup_transform("map", "base_footprint", …)` for `(x, y, yaw)` instead of
  reading `/odom.pose`. The `/plan` is in `map`, so the pose must be too.
- **Speed:** still `v_meas = /odom.twist.linear.x`. That's fine — it's a **body-frame
  scalar speed**, frame-independent; only the *pose* needed to move to `map`.

### Frame roles now
```
 map ─[map→odom, AMCL, corrects drift]→ odom ─[odom→base, wheel odom]→ base_footprint
        planner & follower read map→base_footprint (the whole chain) via TF
```

---

## 3. Gotchas that matter

- **`use_sim_time` everywhere.** TF lookups are time-stamped; AMCL stamps its transforms
  with sim time. A node without `use_sim_time:=true` looks up the wrong clock and TF
  fails. (We pass it to every node in the launch.)
- **Look up at `Time(0)` (= latest available)** with a small timeout, so we don't fail on
  the newest transform not being buffered yet.
- **Guard on missing TF.** Right after startup `map→base` may not exist yet (AMCL not
  active). Both nodes warn and skip a cycle rather than crash.
- **One owner of `map→odom`.** We deleted the static transform publisher — two publishers
  of the same transform make the robot jump/jitter.

---

## 4. Verified

Headless end-to-end (`ros2_ws/scripts/phase2_amcl_test.sh`): AMCL active → planner start
taken from TF `map→base` (`-9.10,-14.10`) → `/plan` in the map frame (128 waypoints) →
Pure Pursuit publishes `/cmd_vel` at 50 Hz (only possible if it resolved the map pose).
The drift-correction efficacy itself (faraway goal, no collision) is the GUI test.

---

## Interview angle

- **Q: You added AMCL but the robot still drifted — why?** The control loop ran in the
  `odom` frame; AMCL's correction is the `map→odom` transform, so a loop that never uses
  `map` ignores it. Fix: plan and control in `map`, reading `map→base` from TF.
- **Q: Why read pose from TF instead of the `/odom` topic?** `/odom` is the raw drifting
  estimate; TF `map→base_footprint` composes `map→odom` (AMCL) with `odom→base` (wheels),
  giving the drift-corrected pose. TF is the single place all frames compose.
- **Q: Why is it OK to still take speed from `/odom`?** Linear speed is a body-frame
  scalar, independent of which world frame you localize in; only pose is frame-relative.
- **Q: What breaks if two nodes publish `map→odom`?** TF has one parent per frame; two
  publishers fight, the transform flickers, and poses jump. Exactly one owner
  (localization) may publish it.
