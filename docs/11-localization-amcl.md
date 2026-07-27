# 11 — Localization with AMCL (Adaptive Monte Carlo Localization)

Why the robot drifted, what `map→odom` really is, and how AMCL corrects it. AMCL =
**Adaptive Monte Carlo Localization** — a particle filter that estimates the robot's
pose in the map by matching laser scans against the map.

---

## 1. The problem it solves

The TF chain is two links:
```
 map ──[ map→odom ]──► odom ──[ odom→base_footprint ]──► base_footprint
      localization owns this      wheel odometry owns this (drifts)
```
- `odom→base_footprint` (the `/odom` topic) is **smooth, high-rate, and drifts** — dead
  reckoning from wheel encoders (`d = wheel_rotation × radius`), which slip.
- `map→odom` is the **drift correction**. Static (fake) in Phase 1.5; **AMCL makes it
  live**. AMCL never edits `/odom`; it publishes a time-varying `map→odom` so that
  `map→base = map→odom ∘ odom→base` tracks the *true* pose while `/odom` stays smooth.

Why keep both frames? `odom` is continuous (good for control, never jumps); `map` is
drift-free but *jumps* when a correction lands. Controllers use `odom` locally;
navigation goals live in `map`.

---

## 2. Monte Carlo Localization (the particle filter)

Represent the belief about the robot's pose as a **cloud of N weighted particles**,
each a hypothesis `(x, y, θ)` in the map. Three steps run every cycle:

### (a) Predict — motion model
When odometry reports the robot moved `Δ`, **push every particle by `Δ` plus noise**
(the odometry is uncertain). The cloud drifts and spreads — uncertainty grows.

### (b) Update — measurement model
Take the real **laser scan**. For each particle, ask: *"if the robot were here, what
scan would the map produce?"* (ray-cast the map from that pose). **Weight the particle
by how well the expected scan matches the real one.** Particles whose predicted view
fits reality get high weight; those facing the wrong way / wrong corridor get ~0.
(Nav2 uses the *likelihood-field* model: score each beam endpoint by its distance to
the nearest map obstacle — fast, smooth.)

### (c) Resample
**Draw a new particle set in proportion to weight** — high-weight hypotheses spawn
copies, low-weight ones die. The cloud **contracts** around the true pose. Over a few
cycles the particles concentrate where scans consistently agree with the map.

The estimated pose = the weighted mean (or best cluster) of the particles.

### "Adaptive" = KLD-sampling
N isn't fixed. When the cloud is spread (lost / just initialized), AMCL uses **many**
particles; once converged, it **drops to few** (cheap). KLD-sampling picks N each step
so the particle set approximates the true belief within a bound — adaptive compute.

---

## 3. How AMCL produces `map→odom`

AMCL estimates `map→base_footprint` (where the robot truly is in the map). It also
*knows* `odom→base_footprint` from `/tf` (wheel odometry). So it computes and
broadcasts:
```
map→odom = map→base_footprint ∘ (odom→base_footprint)⁻¹
```
As odom drifts, this product shifts to cancel it. That's the whole trick: AMCL doesn't
fight odom, it publishes the *offset* that makes the drifting chain end at the truth.

---

## 4. Inputs / outputs / requirements

**Subscribes:** `/scan` (LaserScan), `/map` (OccupancyGrid, latched), `/tf`
(`odom→base_footprint`), and `/initialpose` (from RViz "2D Pose Estimate").
**Publishes:** `/tf` (`map→odom`), `/amcl_pose` (pose + covariance), `/particle_cloud`.
**Needs:** a **lidar** (added in this step), a **static map**, a working **odom→base
TF**, and an **initial pose** (we know the spawn, so we seed it).

Integration consequence: **remove the static `map→odom`** transform — AMCL owns it now,
and two publishers of the same transform fight.

### The three outputs (don't confuse them)
| output | type | role | who consumes |
|--------|------|------|--------------|
| **`map→odom` on `/tf`** | transform | the **authoritative** correction the whole nav stack uses | TF (everything) |
| **`/amcl_pose`** | `PoseWithCovarianceStamped` | map-frame pose **+ covariance** (uncertainty), throttled | logging, monitoring, optional EKF fusion |
| **`/particle_cloud`** | `nav2_msgs/ParticleCloud` | the particle set — **debug/visualization only** | RViz |

Navigation uses the **TF**; `/amcl_pose` and `/particle_cloud` only *report about* the
estimate. `/amcl_pose` is the same pose as `map→base` but carries covariance and comes at
update rate; `/particle_cloud` is optional and functionally inert.

---

## 5. Build plan (one verified step at a time)

1. **Lidar** ✅ — `laser` link + `gpu_lidar` sensor + `/scan` bridge. *(this step)*
2. **Verify `/scan`** in the running sim (RViz LaserScan display; scan hugs the walls).
3. **amcl node + config** — likelihood-field model, particle counts, motion noise,
   `set_initial_pose` at spawn.
4. **Launch integration** — add amcl + lifecycle-manage it; **delete** the static
   `map→odom`; RViz shows `/particle_cloud`.
5. **Test** — drive a faraway goal; confirm `map→odom` corrects and Gazebo vs RViz stay
   aligned (no wall collision).

---

## 6. Unknown environments: SLAM (no prior map)

AMCL has a hard prerequisite: a **known map** — it localizes *against* `/map` by scoring
"expected scan vs map." Remove the map and AMCL cannot run.

In an unknown environment you must estimate the **pose AND the map at the same time** —
**SLAM** (Simultaneous Localization And Mapping). Chicken-and-egg: you need a pose to
place scans into a map, and a map to correct the pose.

### 6.1 The direct extension of MCL — FastSLAM (RBPF)
SLAM extends the particle filter directly. A **Rao-Blackwellized particle filter**
factorizes `p(trajectory, map | data) = p(trajectory | data) · p(map | trajectory, data)`:
- **sample the trajectory** with a particle filter (exactly as in MCL), and
- **each particle carries its OWN map**, grown by dropping its scans into an occupancy
  grid *assuming that particle's trajectory is correct*.

So the one change from AMCL: a particle is no longer just a pose — it's **(pose + a map
built under that pose)**, and you weight it by how well the new scan fits *its own* map.
ROS's `gmapping` is this.

### 6.2 The modern approach — graph SLAM
Today's default (`slam_toolbox`, Cartographer) is **pose-graph optimization**:
- **nodes** = robot poses over time; **edges** = constraints (odometry between
  consecutive poses; scan-match constraints between nearby poses).
- **loop closure** — recognizing you've *returned to a place already seen* — adds an edge
  tying the revisit to the earlier pose, and a global least-squares solve then **spreads
  the accumulated drift out around the whole loop**. Loop closure is what makes large
  maps globally consistent; **AMCL never needs it** because the map is given.

### 6.3 What it means practically (ROS 2 / this project)
| situation | tool | estimates |
|-----------|------|-----------|
| **known map** (our maze) | **AMCL** (`nav2_amcl`) | pose only |
| **unknown map** (explore) | **SLAM** (`slam_toolbox`) | pose **and** map |
| perfect pose already | plain mapping | map only (the easy part) |

We **have** the exact maze map (we generated it), so **AMCL is correct here**. If the
robot instead had to *explore* an unknown maze, we'd run `slam_toolbox` to build the map
while driving, optionally **save** it, then switch to AMCL for repeat runs — Nav2 treats
these as a mode swap (SLAM to make the map, AMCL to reuse it).

---

## Interview angle

- **Q: What is AMCL / MCL?** A particle filter for global-ish pose tracking: particles
  are pose hypotheses, propagated by a motion model, weighted by a sensor model against
  a known map, then resampled. "Adaptive" = KLD-sampling varies the particle count.
- **Q: What exactly does AMCL output?** The `map→odom` transform (plus pose + particle
  cloud). It corrects wheel-odom drift without modifying `/odom`.
- **Q: AMCL vs an EKF (robot_localization)?** EKF fuses proprioceptive sources
  (odom+IMU) — smooths but **drifts unbounded** (no absolute reference). AMCL adds an
  **exteroceptive** fix (lidar vs map), so it's drift-free but needs a map and can fail
  in feature-poor / symmetric spaces. Real stacks use both: EKF for `odom→base`, AMCL
  for `map→odom`.
- **Q: Failure modes?** Kidnapped robot (no global recovery unless enabled),
  perceptual aliasing (symmetric corridors), too few particles, bad initial pose,
  wrong scan↔map frame.
- **Q: Why particles instead of a Kalman filter for the map fix?** The belief is often
  **multi-modal** (several corridors look alike); particles represent arbitrary
  distributions, a single Gaussian (KF) can't.
- **Q: AMCL vs SLAM — when each?** AMCL needs a *known* map and estimates pose only. In
  an *unknown* environment you run SLAM, estimating pose **and** map jointly. Known map
  → localize (AMCL); unknown → map-and-localize (SLAM); then reuse the saved map with AMCL.
- **Q: How does SLAM relate to the AMCL particle filter?** FastSLAM is a
  Rao-Blackwellized particle filter — same MCL loop, but each particle also carries its
  own map. Modern graph SLAM (`slam_toolbox`) instead optimizes a pose graph.
- **Q: What is loop closure and why does it matter?** Detecting that the robot has
  returned to a previously visited place; it adds a constraint that lets global
  optimization correct accumulated drift across the whole trajectory. Only SLAM needs it —
  AMCL's map is fixed, so there's nothing to close.
