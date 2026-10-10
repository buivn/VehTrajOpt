"""Follower planner node for the Ackermann robot (docs/47-48) -- same
pipeline seam as astar_planner_node.py / lattice_ackermann_planner_node.py,
wrapping vto_planning/follower.py's plan_follower() (A* guidance + PID +
dynamic-bicycle-model trajectory) instead of a grid search or a lattice
search:

    /map (OccupancyGrid, latched)          ─┐
    TF map->base_footprint (AMCL, docs/38) ─┼─► plan_follower ─► /plan (nav_msgs/Path, MAP frame)
    /goal_pose (PoseStamped, RViz)          ─┘

Publishing to the SAME /plan topic the other two planners use is the point:
pure_pursuit_ackermann / mpc_controller_ackermann need no changes at all to
consume this planner's output -- "feed the result trajectory into the
controller we already have," literally.

Unit handling (the one real subtlety here, see docs/48): follower.py's
BicycleModel/follow_path are unit-agnostic, but is_free() inside them needs
INTEGER CELL indices, so all physical limits (wheelbase, v_max, accel
bounds -- anything with a length dimension) must be converted meters ->
cells by dividing by map resolution before being handed to plan_follower,
the same move lattice_ackermann_planner_node.py already makes for
turning_radius_cells. steer_vel/max_steer_angle stay in radians (angular,
nothing to convert). theta needs NO conversion -- cell axes (row||y,
col||x) share the same orientation as world axes, just scaled, so atan2
gives the same angle in either space; this is a materially different (and
simpler) situation than lattice.py's heading-BIN convention, which has a
real pi/2 offset baked into its discretization (docs/39).

On failure (collision mid-route, or a waypoint chase timing out): unlike
astar_planner_node/lattice_ackermann_planner_node (which publish nothing on
failure), this node still publishes whatever PREFIX of the trajectory was
collision-free. The robot then visibly drives up to, and stops exactly at,
the failure point -- follow_path's states list already contains this
information up to the failure; the ROS wrapper's job is to not throw it
away. A* itself finding no path at all is the one case with truly nothing
to publish, same as the other two planners.
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

from vto_planning.astar import astar, clearance_field, inflate_obstacles, prune_collinear
from vto_planning.follower import plan_follower


class FollowerPlanner(Node):
    def __init__(self):
        super().__init__("follower_planner")

        # --- parameters: A* layer (same names/defaults as astar_planner_node) ---
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("connectivity", 8)
        self.declare_parameter("clearance_weight", 6.0)
        self.declare_parameter("inflation_radius", 8)
        self.declare_parameter("occ_thresh", 50)
        self.declare_parameter("robot_radius", 0.18)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("prune", True)

        # --- parameters: bicycle-model / PID layer (docs/48) --------------
        # wheelbase/max_steer_angle match ackermannbot.urdf.xacro (docs/31),
        # same defaults as pure_pursuit_ackermann_node.py / mpc_controller_
        # ackermann_node.py, for a fair apples-to-apples comparison between
        # the three planner options.
        self.declare_parameter("wheelbase", 0.29)          # [m]
        self.declare_parameter("max_steer_angle", 0.5)     # [rad]
        self.declare_parameter("v_min", 0.0)               # [m/s]
        self.declare_parameter("v_max", 0.5)               # [m/s], matches the controllers' default
        # accel/steer_vel limits: NOT measured from the real robot (no such
        # spec exists yet) -- picked fresh at this robot's scale, same as
        # follower.py's own module-level defaults (docs/47/48).
        self.declare_parameter("accel_min", -0.3)          # [m/s^2]
        self.declare_parameter("accel_max", 0.3)           # [m/s^2]
        self.declare_parameter("steer_vel_min", -1.0)      # [rad/s]
        self.declare_parameter("steer_vel_max", 1.0)       # [rad/s]
        # PID gains: the advisor's own Utils/PIDController.hpp defaults,
        # kept as starting points (docs/47) -- not yet tuned for this robot.
        self.declare_parameter("kp_steer", 10.0)
        self.declare_parameter("ki_steer", 0.1)
        self.declare_parameter("kd_steer", 0.1)
        self.declare_parameter("kp_vel", 10.0)
        self.declare_parameter("ki_vel", 0.1)
        self.declare_parameter("kd_vel", 0.1)
        self.declare_parameter("dt", 0.05)                 # [s] internal simulation step
        # waypoint-reach tolerance: intermediate A* corners are guidance,
        # not the final goal, so this is looser than pure_pursuit_ackermann's
        # own goal_tolerance (0.25 m) -- 0.4 m default.
        self.declare_parameter("reach_tolerance_m", 0.4)
        self.declare_parameter("max_steps_per_waypoint", 20000)

        # --- state ----------------------------------------------------------
        self.grid = None
        self.clearance = None
        self.res = None
        self.map_ox = None
        self.map_oy = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        plan_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.plan_pub = self.create_publisher(Path, "/plan", plan_qos)
        # same topic/shape as the other two planners' (docs/46) so the
        # benchmark can compare planning time across all three.
        self.planner_compute_pub = self.create_publisher(Float64, "planner/compute_ms", 10)

        self.get_logger().info(
            "follower_planner up (map-frame): waiting for /map, /goal_pose, and TF")

    # ================= world(map) <-> grid ============================ #
    def _cell_from_world(self, x, y):
        c = int((x - self.map_ox) / self.res)
        r = int((y - self.map_oy) / self.res)
        return (r, c)

    def _world_from_cell(self, r, c):
        return (self.map_ox + (c + 0.5) * self.res,
                self.map_oy + (r + 0.5) * self.res)

    def _robot_pose_in_map(self):
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
        self.get_logger().info(
            f"map: {w}x{h} @ {self.res} m, origin=({self.map_ox:.2f},{self.map_oy:.2f}); "
            f"inflated walls by {inflate_cells} cells; clearance precomputed")

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
        goal = self._cell_from_world(gx, gy)
        # start state: (row, col, theta, v, steer) -- theta = raw yaw (no
        # conversion, see module docstring); v/steer start at rest.
        start = (sr, sc, syaw, 0.0, 0.0)
        self.get_logger().info(
            f"planning: start map({sx:.2f},{sy:.2f},yaw={syaw:.2f})=cell{start[:2]} -> "
            f"goal map({gx:.2f},{gy:.2f})=cell{goal}")

        gp = self.get_parameter
        connectivity = gp("connectivity").value
        clearance_weight = gp("clearance_weight").value
        inflation_radius = gp("inflation_radius").value
        occ_thresh = gp("occ_thresh").value
        prune = gp("prune").value

        def astar_fn(grid, s, g):
            cells = astar(grid, s, g, connectivity=connectivity,
                          clearance_weight=clearance_weight,
                          inflation_radius=inflation_radius,
                          occ_thresh=occ_thresh, clearance=self.clearance)
            if cells is None:
                return None
            return prune_collinear(cells) if prune else cells

        # meters -> cells for anything with a length dimension (module
        # docstring); angular quantities (max_steer_angle, steer_vel) pass
        # through unchanged.
        res = self.res
        t0 = time.perf_counter()
        states, reached = plan_follower(
            self.grid, start, goal, astar_fn,
            wheelbase=gp("wheelbase").value / res,
            max_steer_angle=gp("max_steer_angle").value,
            v_min=gp("v_min").value / res,
            v_max=gp("v_max").value / res,
            accel_min=gp("accel_min").value / res,
            accel_max=gp("accel_max").value / res,
            steer_vel_min=gp("steer_vel_min").value,
            steer_vel_max=gp("steer_vel_max").value,
            reach_tolerance=gp("reach_tolerance_m").value / res,
            dt=gp("dt").value,
            max_steps_per_waypoint=gp("max_steps_per_waypoint").value,
            occ_thresh=occ_thresh,
            kp_steer=gp("kp_steer").value, ki_steer=gp("ki_steer").value,
            kd_steer=gp("kd_steer").value,
            kp_vel=gp("kp_vel").value, ki_vel=gp("ki_vel").value,
            kd_vel=gp("kd_vel").value,
        )
        self.planner_compute_pub.publish(Float64(data=(time.perf_counter() - t0) * 1000.0))

        if states is None:
            self.get_logger().warn("follower: A* found no path (goal blocked/unreachable)")
            return
        if not reached:
            self.get_logger().warn(
                f"follower: did NOT reach the goal (collision or waypoint timeout) — "
                f"publishing the {len(states)}-state prefix that WAS collision-free, "
                "so the robot visibly stops at the failure point")

        self._publish_path(states, exact_goal=(gx, gy) if reached else None)
        self.get_logger().info(
            f"published /plan: {len(states)} states (map frame), reached={reached}")

    # ================= output ======================================== #
    def _publish_path(self, states, exact_goal=None):
        """states -> nav_msgs/Path in the map frame. Orientation is set from
        the SIMULATED theta (not identity, unlike astar_planner_node.py /
        lattice_ackermann_planner_node.py) -- unlike A*/lattice, this
        planner's states genuinely carry a real heading estimate at every
        point, so RViz's Path arrows can show it directly. Not consumed by
        the downstream controllers (pursuit_core.py only reads positions),
        so this is a visualization improvement, not a functional one."""
        gf = self.get_parameter("global_frame").value
        path = Path()
        path.header.frame_id = gf
        path.header.stamp = self.get_clock().now().to_msg()
        for (r, c, theta, _v, _steer) in states:
            wx, wy = self._world_from_cell(r, c)
            ps = PoseStamped()
            ps.header.frame_id = gf
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.z = math.sin(theta / 2.0)
            ps.pose.orientation.w = math.cos(theta / 2.0)
            path.poses.append(ps)
        if exact_goal is not None and path.poses:
            gx, gy = exact_goal
            last = path.poses[-1].pose.position
            if math.hypot(gx - last.x, gy - last.y) > 1e-3:
                ps = PoseStamped()
                ps.header.frame_id = gf
                ps.pose.position.x = gx
                ps.pose.position.y = gy
                ps.pose.orientation = path.poses[-1].pose.orientation
                path.poses.append(ps)
        self.plan_pub.publish(path)


def main():
    rclpy.init()
    node = FollowerPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
