"""State-lattice global planner node for the diff-drive robot (M1) — same
pipeline seam as astar_planner_node.py, drop-in alternative selected via the
`planner:=lattice` launch gate (docs/22, vto_bringup/maze_astar.launch.py).

    /map (OccupancyGrid, latched)   ─┐
    TF map->base_footprint (AMCL)   ─┼─► lattice ─► /plan (nav_msgs/Path, MAP frame)
    /goal_pose (PoseStamped, RViz)  ─┘

Unlike A*, a lattice search state carries a HEADING (row, col, heading_index),
not just position, so the start state needs the robot's current yaw -- not just
its (x, y) -- from TF. See docs/22 for why this is expected to help M1's path
*quality* (time_to_goal, driven length) rather than feasibility (A* was always
drivable here via turn-in-place).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from vto_planning.astar import clearance_field, inflate_obstacles
from vto_planning.lattice import (build_primitive_table, densify_path,
                                   heading_index_from_yaw, lattice)


class LatticePlanner(Node):
    def __init__(self):
        super().__init__("lattice_planner")

        # --- parameters -------------------------------------------------
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("clearance_weight", 6.0)
        self.declare_parameter("inflation_radius", 8)      # cells (~half corridor)
        self.declare_parameter("occ_thresh", 50)
        self.declare_parameter("robot_radius", 0.18)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("goal_tolerance_cells", 1.5)
        # motion-primitive shape (docs/22) -- built once, reused for every plan.
        self.declare_parameter("step_cells", 3.0)
        self.declare_parameter("wide_radius_cells", 6.0)
        self.declare_parameter("tight_radius_cells", 3.0)

        # --- state ------------------------------------------------------
        self.grid = None
        self.clearance = None
        self.res = None
        self.map_ox = None
        self.map_oy = None
        self.primitive_table = None   # built once on first /map (docs/22: reused
                                       # everywhere via rotate+translate, not per-plan)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        plan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.plan_pub = self.create_publisher(Path, "/plan", plan_qos)

        self.get_logger().info(
            "lattice_planner up (map-frame): waiting for /map, /goal_pose, and TF")

    # ================= world(map) <-> grid ============================ #
    def _cell_from_world(self, x, y):
        c = int((x - self.map_ox) / self.res)
        r = int((y - self.map_oy) / self.res)
        return (r, c)

    def _world_from_cell(self, r, c):
        return (self.map_ox + (c + 0.5) * self.res,
                self.map_oy + (r + 0.5) * self.res)

    def _robot_pose_in_map(self):
        """Robot (x, y, yaw) in the map frame from TF (AMCL-corrected). None if
        unavailable. Yaw is needed here (unlike astar_planner_node.py) because a
        lattice start state includes a heading bin."""
        gf = self.get_parameter("global_frame").value
        bf = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                gf, bf, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(f"TF {gf}->{bf} unavailable: {e}")
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

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
        self.primitive_table = build_primitive_table(
            step_cells=self.get_parameter("step_cells").value,
            wide_radius_cells=self.get_parameter("wide_radius_cells").value,
            tight_radius_cells=self.get_parameter("tight_radius_cells").value,
        )
        self.get_logger().info(
            f"map: {w}x{h} @ {self.res} m, origin=({self.map_ox:.2f},{self.map_oy:.2f}); "
            f"inflated walls by {inflate_cells} cells; clearance + primitive table built")

    def _on_goal(self, msg: PoseStamped):
        if self.grid is None:
            self.get_logger().warn("goal received but no /map yet — ignoring")
            return
        pose = self._robot_pose_in_map()
        if pose is None:
            self.get_logger().warn("goal received but robot TF pose unavailable — ignoring")
            return
        sx, sy, syaw = pose

        gf = self.get_parameter("global_frame").value
        if msg.header.frame_id and msg.header.frame_id != gf:
            self.get_logger().warn(
                f"goal in frame '{msg.header.frame_id}', expected '{gf}'; using coords as-is")
        gx, gy = msg.pose.position.x, msg.pose.position.y

        sr, sc = self._cell_from_world(sx, sy)
        # heading_index_from_yaw, not heading_to_index directly -- the two
        # angle conventions differ by a fixed pi/2 twist (row/col vs x/y
        # axes); harmless here since free in-place pivots absorb any
        # mislabeling, but fixed anyway now that docs/39 found + fixed it
        # for the Ackermann planner, which has no such pivot to hide it.
        start = (sr, sc, heading_index_from_yaw(syaw))
        goal = self._cell_from_world(gx, gy)
        self.get_logger().info(
            f"planning: start map({sx:.2f},{sy:.2f},yaw={syaw:.2f})=cell{start} -> "
            f"goal map({gx:.2f},{gy:.2f})=cell{goal}")

        states = lattice(
            self.grid, start, goal,
            primitive_table=self.primitive_table,
            goal_tolerance=self.get_parameter("goal_tolerance_cells").value,
            occ_thresh=self.get_parameter("occ_thresh").value,
            clearance_weight=self.get_parameter("clearance_weight").value,
            inflation_radius=self.get_parameter("inflation_radius").value,
            clearance=self.clearance,
        )
        if states is None:
            self.get_logger().warn("lattice: no path found (goal blocked/unreachable)")
            return

        # densify_path deliberately keeps every sampled point along each curved
        # primitive (docs/22) -- unlike astar_planner_node.py, this is NOT run
        # through prune_collinear: that pruning targets straight-line redundancy
        # on a grid path and would flatten the arcs this planner exists to keep.
        cells = densify_path(states, self.primitive_table)

        self._publish_path(cells, exact_goal=(gx, gy))
        self.get_logger().info(f"published /plan: {len(cells)} waypoints (map frame)")

    # ================= output ======================================== #
    def _publish_path(self, cells, exact_goal=None):
        """cells -> nav_msgs/Path in the map frame.

        The search only guarantees landing within `goal_tolerance_cells` of
        the requested goal (module docstring, lattice.py) -- no primitive
        combination generally lands exactly on an arbitrary continuous goal.
        `exact_goal` (the raw (x, y) from /goal_pose, meters) is appended as
        one final straight docking waypoint so the path's own last point is
        the literal goal, not a near-goal lattice state -- see docs/30. A*
        needs no equivalent because it searches the goal CELL directly, so
        its last waypoint is already within half a cell of the true goal.
        """
        gf = self.get_parameter("global_frame").value
        path = Path()
        path.header.frame_id = gf
        path.header.stamp = self.get_clock().now().to_msg()
        for (r, c) in cells:
            wx, wy = self._world_from_cell(r, c)
            ps = PoseStamped()
            ps.header.frame_id = gf
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        if exact_goal is not None and path.poses:
            gx, gy = exact_goal
            last = path.poses[-1].pose.position
            if math.hypot(gx - last.x, gy - last.y) > 1e-3:
                ps = PoseStamped()
                ps.header.frame_id = gf
                ps.pose.position.x = gx
                ps.pose.position.y = gy
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
        self.plan_pub.publish(path)


def main():
    rclpy.init()
    node = LatticePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
