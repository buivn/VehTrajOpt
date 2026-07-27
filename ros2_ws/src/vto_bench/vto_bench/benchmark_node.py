"""Controller benchmark orchestrator (see docs/14).

Runs a deterministic GOAL TOUR and logs the 7 metrics per leg, so Pure Pursuit /
MPC / MPPI / … are all measured identically on the same map, goals, and starts.

Flow:
  on /map      -> build grid, sample `num_legs` reachable well-clear goal cells
                  (seeded -> identical every run), verified solvable by A*.
  tour         -> publish goal_i on /goal_pose; a timer samples per-tick data
                  (TF map->base pose, /cmd_vel, cross-track vs /plan, clearance,
                  /controller/compute_ms); on reach (or timeout) write a per-leg
                  summary row, advance. Repeat `repeats` times, then stop.
Outputs: <output_dir>/<label>_summary.csv (one row/leg) + <label>_ticks.csv.
"""
import csv
import math
import os
import random
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from vto_bench.metrics import (SUMMARY_COLUMNS, cross_track, path_length,
                               summarize)
from vto_planning.astar import astar, clearance_field, inflate_obstacles


class Benchmark(Node):
    def __init__(self):
        super().__init__("benchmark")
        self.declare_parameter("label", "controller")     # tags the output files
        self.declare_parameter("num_legs", 30)
        self.declare_parameter("repeats", 2)
        self.declare_parameter("seed", 1)
        self.declare_parameter("goal_tolerance", 0.25)     # [m]
        self.declare_parameter("leg_timeout", 120.0)       # [s] give up on a leg
        self.declare_parameter("sample_rate", 20.0)        # [Hz] per-tick logging
        self.declare_parameter("min_clear_cells", 6)       # goal must be this clear of walls
        self.declare_parameter("leg_min_dist", 10.0)       # [m] leg PATH length band (bounds
        self.declare_parameter("leg_max_dist", 25.0)       #     drive time & keeps legs diverse)
        self.declare_parameter("robot_radius", 0.18)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("spawn_x", -9.1)
        self.declare_parameter("spawn_y", -14.1)
        self.declare_parameter("output_dir", "/workspace/ros2_ws/results")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        # state
        self.grid = None
        self.res = self.map_ox = self.map_oy = None
        self.clear = None                 # clearance field (cells) on the inflated grid
        self.goals = None                 # list[(x, y)] tour goals (world/map)
        self.plan_pts = []                # current leg's /plan points
        self.plan_len = 0.0
        self.cmd = (0.0, 0.0)             # latest /cmd_vel (v, w)
        self.compute_ms = None            # latest /controller/compute_ms
        self.repeat = 0
        self.leg = 0
        self.tour_built = False
        self.leg_active = False
        self.leg_start_t = 0.0
        self.leg_start_xy = (0.0, 0.0)
        self.last_send = 0.0          # when the current goal was last (re)published
        self.records = []
        self.done = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(Path, "/plan", self._on_plan, latched)
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Float64, "/controller/compute_ms", self._on_compute, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        os.makedirs(self.get_parameter("output_dir").value, exist_ok=True)
        self._open_csv()
        self.timer = self.create_timer(
            1.0 / self.get_parameter("sample_rate").value, self._tick)
        self.get_logger().info("benchmark up: waiting for /map to build the tour")

    # ---------------- setup ---------------- #
    def _open_csv(self):
        d = self.get_parameter("output_dir").value
        label = self.get_parameter("label").value
        self.summary_f = open(os.path.join(d, f"{label}_summary.csv"), "w", newline="")
        self.summary_w = csv.DictWriter(self.summary_f, fieldnames=SUMMARY_COLUMNS)
        self.summary_w.writeheader()
        self.ticks_f = open(os.path.join(d, f"{label}_ticks.csv"), "w", newline="")
        self.ticks_w = csv.writer(self.ticks_f)
        self.ticks_w.writerow(["repeat", "leg", "t", "x", "y", "v", "w",
                               "cte", "clearance", "compute_ms"])

    def _cell_from_world(self, x, y):
        return (int((y - self.map_oy) / self.res), int((x - self.map_ox) / self.res))

    def _world_from_cell(self, r, c):
        return (self.map_ox + (c + 0.5) * self.res, self.map_oy + (r + 0.5) * self.res)

    def _on_map(self, msg: OccupancyGrid):
        if self.goals is not None:
            return
        w, h = msg.info.width, msg.info.height
        self.res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        raw = [list(msg.data[r * w:(r + 1) * w]) for r in range(h)]
        rr = self.get_parameter("robot_radius").value
        sm = self.get_parameter("safety_margin").value
        infl = max(0, int((rr + sm) / self.res + 0.999))
        self.grid = inflate_obstacles(raw, infl)
        self.clear = clearance_field(self.grid)
        self._build_tour(h, w)
        self.tour_built = True          # the tour timer starts leg 0 once TF is up
        self.get_logger().info(
            f"tour ready: {len(self.goals)} legs x {self.get_parameter('repeats').value} "
            f"repeats (label={self.get_parameter('label').value}); waiting for TF to start")

    def _path_len_m(self, path):
        return self.res * sum(
            math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
            for k in range(len(path) - 1))

    def _build_tour(self, rows, cols):
        """Chain goals so each leg's A* PATH length is in [leg_min_dist, leg_max_dist]
        — bounds drive time and keeps legs short + diverse. Deterministic (seed +
        spawn), so both controllers get the identical tour."""
        n = self.get_parameter("num_legs").value
        mc = self.get_parameter("min_clear_cells").value
        dmin = self.get_parameter("leg_min_dist").value
        dmax = self.get_parameter("leg_max_dist").value
        rng = random.Random(self.get_parameter("seed").value)
        candidates = [(r, c) for r in range(rows) for c in range(cols)
                      if self.clear[r][c] >= mc]
        cur_xy = (self.get_parameter("spawn_x").value,
                  self.get_parameter("spawn_y").value)
        cur_cell = self._cell_from_world(*cur_xy)
        goals, attempts = [], 0
        while len(goals) < n and attempts < 20000:
            attempts += 1
            cell = rng.choice(candidates)
            path = astar(self.grid, cur_cell, cell, clearance=self.clear,
                         clearance_weight=0.0)
            if path is None or not (dmin <= self._path_len_m(path) <= dmax):
                continue
            cur_xy, cur_cell = self._world_from_cell(*cell), cell
            goals.append(cur_xy)
        self.goals = goals
        self._write_tour()

    def _write_tour(self):
        path = os.path.join(self.get_parameter("output_dir").value, "tour.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["leg", "goal_x", "goal_y"])
            for i, (x, y) in enumerate(self.goals):
                w.writerow([i, round(x, 3), round(y, 3)])

    # ---------------- data in ---------------- #
    def _on_plan(self, msg: Path):
        self.plan_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.plan_len = path_length(self.plan_pts)

    def _on_cmd(self, msg: TwistStamped):
        self.cmd = (msg.twist.linear.x, msg.twist.angular.z)

    def _on_compute(self, msg: Float64):
        self.compute_ms = msg.data

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter("global_frame").value,
                self.get_parameter("base_frame").value,
                rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    # ---------------- tour control ---------------- #
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _send_goal(self):
        gx, gy = self.goals[self.leg]
        g = PoseStamped()
        g.header.frame_id = self.get_parameter("global_frame").value
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y, g.pose.orientation.w = gx, gy, 1.0
        self.goal_pub.publish(g)
        self.last_send = self._now()

    def _start_leg(self):
        # called from _tick only when TF is available -> leg_start_xy is real.
        self.leg_start_xy = self._robot_xy() or (0.0, 0.0)
        self.plan_pts, self.plan_len = [], 0.0       # wait for the new leg's plan
        self.records = []
        self.leg_start_t = self._now()
        self.leg_active = True
        self._send_goal()
        gx, gy = self.goals[self.leg]
        self.get_logger().info(
            f"[r{self.repeat} leg {self.leg + 1}/{len(self.goals)}] goal "
            f"({gx:.1f},{gy:.1f}) from ({self.leg_start_xy[0]:.1f},{self.leg_start_xy[1]:.1f})")

    def _tick(self):
        if self.done or self.goals is None or not self.tour_built:
            return
        xy = self._robot_xy()
        if xy is None:
            return                       # wait for AMCL/TF before starting/logging
        if not self.leg_active:
            self._start_leg()            # begin the next leg now that TF is up
            return
        x, y = xy
        v, w = self.cmd
        clr_m = 0.0
        r, c = self._cell_from_world(x, y)
        if 0 <= r < len(self.clear) and 0 <= c < len(self.clear[0]):
            clr_m = self.clear[r][c] * self.res
        self.records.append({"t": self._now(), "x": x, "y": y, "v": v, "w": w,
                             "cte": cross_track(x, y, self.plan_pts),
                             "clearance": clr_m, "compute_ms": self.compute_ms})

        # a goal sent before the planner's TF was ready gets dropped -> re-send it
        if not self.plan_pts and (self._now() - self.last_send) > 3.0:
            self._send_goal()

        gx, gy = self.goals[self.leg]
        reached = ((gx - x) ** 2 + (gy - y) ** 2) ** 0.5 < \
            self.get_parameter("goal_tolerance").value
        timeout = (self._now() - self.leg_start_t) > \
            self.get_parameter("leg_timeout").value
        if reached or timeout:
            self._finish_leg(reached)

    def _finish_leg(self, reached):
        self.leg_active = False
        s = summarize(self.records, self.plan_len, reached)
        row = {"controller": self.get_parameter("label").value,
               "repeat": self.repeat, "leg": self.leg,
               "start_x": round(self.leg_start_xy[0], 3),
               "start_y": round(self.leg_start_xy[1], 3),
               "goal_x": round(self.goals[self.leg][0], 3),
               "goal_y": round(self.goals[self.leg][1], 3), **s}
        self.summary_w.writerow(row)
        self.summary_f.flush()
        for rec in self.records:
            self.ticks_w.writerow([self.repeat, self.leg, round(rec["t"], 3),
                                   round(rec["x"], 3), round(rec["y"], 3),
                                   round(rec["v"], 3), round(rec["w"], 3),
                                   round(rec["cte"], 4), round(rec["clearance"], 3),
                                   rec["compute_ms"]])
        self.ticks_f.flush()
        self.get_logger().info(
            f"[r{self.repeat} leg {self.leg + 1}] {'REACHED' if reached else 'TIMEOUT'} "
            f"t={s['time_to_goal']:.1f}s cte_max={s['cte_max']:.3f} "
            f"dw_rms={s['dw_rms']:.3f} clr={s['min_clearance']:.2f}")
        self._advance()

    def _advance(self):
        self.leg += 1
        if self.leg >= len(self.goals):
            self.leg = 0
            self.repeat += 1
            if self.repeat >= self.get_parameter("repeats").value:
                self.done = True
                self.summary_f.close()
                self.ticks_f.close()
                # sentinel so a batch script knows this controller's sweep finished
                open(os.path.join(self.get_parameter("output_dir").value,
                                  f"{self.get_parameter('label').value}.done"), "w").close()
                self.get_logger().info("BENCHMARK DONE — results written.")
                return
        self._start_leg()


def main():
    rclpy.init()
    node = Benchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
