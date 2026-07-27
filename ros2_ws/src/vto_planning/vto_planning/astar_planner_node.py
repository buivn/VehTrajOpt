"""A* global planner node — map-frame / TF-aware (Phase 2, with AMCL).

Pipeline seam (see docs/07, docs/12):
    /map (OccupancyGrid, latched)   ─┐
    TF map->base_footprint (AMCL)   ─┼─► A* ─► /plan (nav_msgs/Path, MAP frame)
    /goal_pose (PoseStamped, RViz)  ─┘

Frames (the Phase-2 change): the robot's start pose comes from **TF map->base**,
which AMCL keeps corrected against odom drift. We plan and publish entirely in the
`map` frame, so the follower (also map-frame now) benefits from the correction.
The old static spawn offset (`_world_from_odom` etc.) is gone — localization owns
map->odom, and we just read the corrected pose from tf.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from vto_planning.astar import (astar, clearance_field, inflate_obstacles,
                                prune_collinear)


class AStarPlanner(Node):
    def __init__(self):
        super().__init__("astar_planner")

        # --- parameters -------------------------------------------------
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("connectivity", 8)          # 4 or 8
        # clearance cost (soft centering): penalty = w * max(0, R - dist_to_wall).
        self.declare_parameter("clearance_weight", 6.0)
        self.declare_parameter("inflation_radius", 8)      # cells (~half corridor)
        self.declare_parameter("occ_thresh", 50)
        # C-space inflation (hard body clearance) by robot_radius + margin.
        self.declare_parameter("robot_radius", 0.18)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("prune", True)

        # --- state ------------------------------------------------------
        self.grid = None            # inflated occupancy grid (row 0 = bottom = min y)
        self.clearance = None       # precomputed distance transform
        self.res = None
        self.map_ox = None
        self.map_oy = None

        # --- TF: robot start pose in the map frame (AMCL-corrected) -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- QoS --------------------------------------------------------
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        plan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # --- I/O --------------------------------------------------------
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.plan_pub = self.create_publisher(Path, "/plan", plan_qos)

        self.get_logger().info(
            "astar_planner up (map-frame): waiting for /map, /goal_pose, and TF")

    # ================= world(map) <-> grid ============================ #
    def _cell_from_world(self, x, y):
        c = int((x - self.map_ox) / self.res)
        r = int((y - self.map_oy) / self.res)      # row 0 = bottom (min y)
        return (r, c)

    def _world_from_cell(self, r, c):
        return (self.map_ox + (c + 0.5) * self.res,
                self.map_oy + (r + 0.5) * self.res)

    def _robot_xy_in_map(self):
        """Robot (x, y) in the map frame from TF (AMCL-corrected). None if unavailable."""
        gf = self.get_parameter("global_frame").value
        bf = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                gf, bf, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(f"TF {gf}->{bf} unavailable: {e}")
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    # ================= callbacks ===================================== #
    def _on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        self.res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        occ = self.get_parameter("occ_thresh").value
        d = msg.data
        raw = [list(d[r * w:(r + 1) * w]) for r in range(h)]
        rr = self.get_parameter("robot_radius").value
        sm = self.get_parameter("safety_margin").value
        inflate_cells = max(0, math.ceil((rr + sm) / self.res))
        self.grid = inflate_obstacles(raw, inflate_cells, occ)
        self.clearance = clearance_field(self.grid, occ)
        self.get_logger().info(
            f"map: {w}x{h} @ {self.res} m, origin=({self.map_ox:.2f},{self.map_oy:.2f}); "
            f"inflated walls by {inflate_cells} cells; clearance precomputed")

    def _on_goal(self, msg: PoseStamped):
        if self.grid is None:
            self.get_logger().warn("goal received but no /map yet — ignoring")
            return
        start_xy = self._robot_xy_in_map()
        if start_xy is None:
            self.get_logger().warn("goal received but robot TF pose unavailable — ignoring")
            return
        sx, sy = start_xy

        gf = self.get_parameter("global_frame").value
        if msg.header.frame_id and msg.header.frame_id != gf:
            self.get_logger().warn(
                f"goal in frame '{msg.header.frame_id}', expected '{gf}'; using coords as-is")
        gx, gy = msg.pose.position.x, msg.pose.position.y

        start = self._cell_from_world(sx, sy)
        goal = self._cell_from_world(gx, gy)
        self.get_logger().info(
            f"planning: start map({sx:.2f},{sy:.2f})=cell{start} -> "
            f"goal map({gx:.2f},{gy:.2f})=cell{goal}")

        cells = astar(
            self.grid, start, goal,
            connectivity=self.get_parameter("connectivity").value,
            clearance_weight=self.get_parameter("clearance_weight").value,
            inflation_radius=self.get_parameter("inflation_radius").value,
            occ_thresh=self.get_parameter("occ_thresh").value,
            clearance=self.clearance,
        )
        if cells is None:
            self.get_logger().warn("A*: no path found (goal blocked/unreachable)")
            return
        if self.get_parameter("prune").value:
            cells = prune_collinear(cells)

        self._publish_path(cells)
        self.get_logger().info(f"published /plan: {len(cells)} waypoints (map frame)")

    # ================= output ======================================== #
    def _publish_path(self, cells):
        gf = self.get_parameter("global_frame").value
        path = Path()
        path.header.frame_id = gf
        path.header.stamp = self.get_clock().now().to_msg()
        for (r, c) in cells:
            wx, wy = self._world_from_cell(r, c)     # cell -> world(map); published as-is
            ps = PoseStamped()
            ps.header.frame_id = gf
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.plan_pub.publish(path)


def main():
    rclpy.init()
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
