"""Pure Pursuit path tracker for the Ackermann (car-like) robot — docs/35,
pose source revised docs/39.

Same geometric algorithm as pure_pursuit_node.py (diff-drive), but the
command law respects the bicycle-model constraint that curvature -- not
angular velocity -- is what's physically bounded (by the steering angle
limit), and there is no turn-in-place recovery (a car can't rotate without
translating). See docs/31 (theory) and docs/35 (this rework) for why.

Output is geometry_msgs/TwistStamped on
/ackermann_steering_controller/reference (native topic name, no /cmd_vel
remap exists for this robot yet -- docs/34). ackermann_steering_controller
internally derives the required steering angle from (v, w); this node's
job is to never ask for one beyond max_steer_angle.

Pose comes from TF map->base_footprint (AMCL+EKF, docs/37-38), NOT the
controller's own Odometry message anymore. docs/35's original reasoning --
ackermann_steering_controller's `enable_odom_tf` publishes to its own
namespaced `<name>/tf_odometry` topic, not the shared /tf a
TransformListener reads -- is still true, but is no longer the deciding
fact: this robot now has AMCL+EKF (docs/37-38), and BOTH of those publish
to the shared /tf (ekf_ackermann.yaml publish_tf, amcl_ackermann.yaml
tf_broadcast), so a map->base_footprint lookup resolves fine via that
chain. It must be map-frame, not odom-frame, because lattice_ackermann_
planner_node.py (docs/39) publishes /plan in the map frame -- tracking
against an odom-frame pose while the path is in map frame would silently
drift apart from AMCL's map->odom correction. Odometry is still subscribed
for its velocity (v_meas has no TF equivalent).
"""
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from vto_control.pursuit_core import (adaptive_lookahead, lookahead_point,
                                      velocity_command_ackermann)


class PurePursuitAckermann(Node):
    def __init__(self):
        super().__init__("pure_pursuit_ackermann")
        # --- tunables ---------------------------------------------------
        self.declare_parameter("max_linear", 0.5)       # [m/s]
        self.declare_parameter("min_linear", 0.12)      # [m/s] anti-stall floor while driving
        self.declare_parameter("goal_tolerance", 0.25)  # [m] stop within this of the last point
        self.declare_parameter("lookahead_min", 0.5)    # [m]
        self.declare_parameter("lookahead_max", 1.0)    # [m]
        self.declare_parameter("lookahead_gain", 1.0)   # [s]
        self.declare_parameter("slow_gain", 0.8)        # speed drop vs curvature
        # bicycle-model limits (docs/31/35) -- defaults match
        # ackermannbot.urdf.xacro's wheelbase/max_steer_angle.
        self.declare_parameter("wheelbase", 0.29)          # [m]
        self.declare_parameter("max_steer_angle", 0.5)     # [rad]
        self.max_curvature = (math.tan(self.get_parameter("max_steer_angle").value)
                              / self.get_parameter("wheelbase").value)
        # pose source (docs/39): /plan is in map frame, so pose must be too.
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        self.path_pts: list[tuple[float, float]] = []
        self.reached = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(
            Odometry, "/ackermann_steering_controller/odometry", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(
            TwistStamped, "/ackermann_steering_controller/reference", 10)
        self.compute_pub = self.create_publisher(Float64, "controller/compute_ms", 10)
        self.get_logger().info(
            f"pure_pursuit_ackermann up (max_curvature={self.max_curvature:.3f} 1/m): "
            "waiting for /plan, odometry, and TF map->base_footprint")

    # ------------------------------------------------------------------ #
    def _on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.reached = False
        self.get_logger().info(f"received path with {len(self.path_pts)} points")

    def _pose_in_map(self):
        """Robot (x, y, yaw) in the map frame from TF (AMCL+EKF, docs/38-39).
        None if unavailable -- same lookup lattice_ackermann_planner_node.py
        uses, kept independent here rather than shared: these are two
        separate nodes with no ROS-native way to share a helper without a
        new library package, and the lookup is 6 lines."""
        gf = self.get_parameter("global_frame").value
        bf = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                gf, bf, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(f"TF {gf}->{bf} unavailable: {e}", throttle_duration_sec=2.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def _on_odom(self, msg: Odometry):
        if not self.path_pts or self.reached:
            return

        pose = self._pose_in_map()
        if pose is None:
            return
        v_meas = msg.twist.twist.linear.x
        px, py, yaw = pose

        gx, gy = self.path_pts[-1]
        if math.hypot(gx - px, gy - py) < self.get_parameter("goal_tolerance").value:
            self.reached = True
            self._publish(0.0, 0.0)
            self.get_logger().info("goal reached — stopping")
            return

        t0 = time.perf_counter()
        ld = adaptive_lookahead(v_meas,
                                self.get_parameter("lookahead_gain").value,
                                self.get_parameter("lookahead_min").value,
                                self.get_parameter("lookahead_max").value)
        target = lookahead_point(self.path_pts, px, py, ld) or (gx, gy)

        v, w, _ = velocity_command_ackermann(
            target, px, py, yaw,
            max_linear=self.get_parameter("max_linear").value,
            min_linear=self.get_parameter("min_linear").value,
            max_curvature=self.max_curvature,
            slow_gain=self.get_parameter("slow_gain").value,
        )
        self.compute_pub.publish(Float64(data=(time.perf_counter() - t0) * 1000.0))
        self._publish(v, w)

    # ------------------------------------------------------------------ #
    def _publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = PurePursuitAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
