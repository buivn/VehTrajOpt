"""State-lattice global planner node for the Ackermann (car-like) robot (M2)
-- same pipeline seam as lattice_planner_node.py (diff-drive), swapped onto
lattice_ackermann.py's Dubins control set (docs/24-25). docs/31 flagged A*
as kinematically naive for this robot (ignores heading/turning radius
entirely) -- true, and still the reason this node exists -- but docs/46
adds astar_planner (already fully robot-agnostic, no changes needed) as an
explicit, deliberately-naive FAST baseline via planner:=astar, to measure
the actual planning-time/path-quality trade-off rather than assume it.

    /map (OccupancyGrid, latched)          ─┐
    TF map->base_footprint (AMCL, docs/38) ─┼─► lattice_ackermann ─► /plan (nav_msgs/Path, MAP frame)
    /goal_pose (PoseStamped, RViz)          ─┘

turning_radius_m is not a free tuning knob: it is the bicycle-model minimum
turning radius R_min = wheelbase / tan(max_steer_angle), a physical limit of
ackermannbot.urdf.xacro (wheelbase=0.29 m, max_steer_angle=0.5 rad ->
R_min ~= 0.53 m, see that file's own comment). A tighter default would let
the search plan curves the steering geometry cannot execute; a looser one
would just discard reachable primitives. Only this one physically-anchored
parameter (plus the search-window size, max_reach_cells) is exposed here --
the remaining make_ackermann_primitive_library() knobs (heading-change cap,
sample density, prune tolerances) are library-construction internals, not
mission-level tuning, and stay at their validated defaults (docs/25).
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from vto_planning.astar import clearance_field, inflate_obstacles
from vto_planning.lattice import densify_path, heading_index_from_yaw, lattice
from vto_planning.lattice_ackermann import ackermann_primitive_table


class LatticeAckermannPlanner(Node):
    def __init__(self):
        super().__init__("lattice_ackermann_planner")

        # --- parameters -------------------------------------------------
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("clearance_weight", 6.0)
        self.declare_parameter("inflation_radius", 8)      # cells (~half corridor)
        self.declare_parameter("occ_thresh", 50)
        # same as diffbot's default (lattice_planner_node.py): both robots
        # share base_wid=0.30 m, and corridor clearance is width-driven, not
        # length-driven, since the vehicle travels along its length axis.
        self.declare_parameter("robot_radius", 0.18)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("goal_tolerance_cells", 1.5)
        # bicycle-model turning radius (docs/31, ackermannbot.urdf.xacro) --
        # module docstring explains why this default, not a rounder number.
        self.declare_parameter("turning_radius_m", 0.29 / math.tan(0.5))
        self.declare_parameter("max_reach_cells", 7)
        # Safety net, not a routine limit (docs/45): this node's lattice()
        # call previously had NO cap, on the assumption "a bad interactive
        # goal is rare, one slow reject beats a wrong path" (docs/40). That
        # assumption broke once ordinary long-haul replans could
        # legitimately need 300k+ expansions (~30s) after docs/44's
        # clearance-penalty fix -- a slow-but-VALID search, not a reject,
        # which then triggered a retry storm via the benchmark's own resend
        # logic (fixed separately, docs/45). This cap exists purely so a
        # genuinely pathological query still returns in bounded time rather
        # than hanging the node's single-threaded executor indefinitely;
        # 800000 has real margin above the 310k measured for a real,
        # successful cross-map replan, so it should not reject legitimate
        # long routes in practice.
        self.declare_parameter("max_expansions", 800000)

        # --- state ------------------------------------------------------
        self.grid = None
        self.clearance = None
        self.res = None
        self.map_ox = None
        self.map_oy = None
        self.primitive_table = None   # built once on first /map, in CELL units
                                       # derived from turning_radius_m and this
                                       # map's resolution (docs/22 pattern)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        plan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.plan_pub = self.create_publisher(Path, "/plan", plan_qos)
        # planner runtime metric (docs/46) -- same topic/shape as
        # astar_planner_node.py's, so the benchmark can compare them.
        self.planner_compute_pub = self.create_publisher(Float64, "planner/compute_ms", 10)

        self.get_logger().info(
            "lattice_ackermann_planner up (map-frame): waiting for /map, "
            "/goal_pose, and TF")

    # ================= world(map) <-> grid ============================ #
    def _cell_from_world(self, x, y):
        c = int((x - self.map_ox) / self.res)
        r = int((y - self.map_oy) / self.res)
        return (r, c)

    def _world_from_cell(self, r, c):
        return (self.map_ox + (c + 0.5) * self.res,
                self.map_oy + (r + 0.5) * self.res)

    def _robot_pose_in_map(self):
        """Robot (x, y, yaw) in the map frame from TF (AMCL-corrected, docs/38).
        None if unavailable. Both ekf_node and amcl publish to the shared /tf
        (ekf_ackermann.yaml publish_tf, amcl_ackermann.yaml tf_broadcast) --
        unlike pure_pursuit_ackermann_node.py's pre-docs/37 caveat about
        ackermann_steering_controller's OWN tf_odometry topic, this lookup is
        map->base_footprint via AMCL+EKF's chain, not that native one."""
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

        turning_radius_cells = self.get_parameter("turning_radius_m").value / self.res
        self.primitive_table = ackermann_primitive_table(
            turning_radius_cells=turning_radius_cells,
            max_reach_cells=self.get_parameter("max_reach_cells").value,
        )
        self.get_logger().info(
            f"map: {w}x{h} @ {self.res} m, origin=({self.map_ox:.2f},{self.map_oy:.2f}); "
            f"inflated walls by {inflate_cells} cells; turning_radius="
            f"{turning_radius_cells:.2f} cells; clearance + primitive table built")

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
        # heading_index_from_yaw, NOT heading_to_index directly -- see
        # lattice.py's docstring: this module's heading-index convention and
        # a real-world TF yaw differ by a fixed pi/2 twist (row/col vs x/y
        # axes). A forward-only vehicle has no free way to correct a
        # mislabeled start heading the way diff-drive's in-place pivots do,
        # so getting this right matters here specifically (docs/39).
        start = (sr, sc, heading_index_from_yaw(syaw))
        goal = self._cell_from_world(gx, gy)
        self.get_logger().info(
            f"planning: start map({sx:.2f},{sy:.2f},yaw={syaw:.2f})=cell{start} -> "
            f"goal map({gx:.2f},{gy:.2f})=cell{goal}")

        t0 = time.perf_counter()
        states = lattice(
            self.grid, start, goal,
            primitive_table=self.primitive_table,
            goal_tolerance=self.get_parameter("goal_tolerance_cells").value,
            occ_thresh=self.get_parameter("occ_thresh").value,
            clearance_weight=self.get_parameter("clearance_weight").value,
            inflation_radius=self.get_parameter("inflation_radius").value,
            clearance=self.clearance,
            max_expansions=self.get_parameter("max_expansions").value,
        )
        self.planner_compute_pub.publish(Float64(data=(time.perf_counter() - t0) * 1000.0))
        if states is None:
            self.get_logger().warn("lattice_ackermann: no path found (goal blocked/unreachable)")
            return

        # densify_path deliberately keeps every sampled point along each
        # curved primitive (docs/22) -- the arcs are the whole point here.
        cells = densify_path(states, self.primitive_table)

        self._publish_path(cells, exact_goal=(gx, gy))
        self.get_logger().info(f"published /plan: {len(cells)} waypoints (map frame)")

    # ================= output ======================================== #
    def _publish_path(self, cells, exact_goal=None):
        """cells -> nav_msgs/Path in the map frame. See lattice_planner_node.py's
        docstring for why exact_goal is appended as a final docking waypoint
        (docs/30) -- identical reasoning, unchanged here."""
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
    node = LatticeAckermannPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
