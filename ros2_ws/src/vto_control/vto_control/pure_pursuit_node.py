"""Pure Pursuit path tracker (Phase 1).

The classic geometric path follower. Given a map-frame path (nav_msgs/Path) and the
robot's pose from TF (map->base, AMCL-corrected; /odom twist used only for speed),
it repeatedly:
  1. picks a "lookahead" point on the path a fixed distance ahead of the robot,
  2. computes the curvature of the arc that connects the robot to that point,
  3. commands a forward speed + the matching angular speed.

Output is geometry_msgs/TwistStamped on /cmd_vel (what Jazzy's
diff_drive_controller expects). This is the Phase-1 *baseline*: MPC and MPPI will
implement the same interface (Path + Odometry in -> /cmd_vel out) so all three
are directly comparable in vto_bench.
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
                                      velocity_command)


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        # --- tunables ---------------------------------------------------
        self.declare_parameter("max_linear", 0.5)       # [m/s]
        self.declare_parameter("min_linear", 0.12)      # [m/s] anti-stall floor while driving
        self.declare_parameter("max_angular", 1.5)      # [rad/s]
        self.declare_parameter("goal_tolerance", 0.25)  # [m] stop within this of the last point
        # adaptive lookahead: Ld = clamp(gain*|v|, min, max). fast->long, slow->short.
        # ld_min must not be tiny: a small carrot spikes curvature and stalls v.
        self.declare_parameter("lookahead_min", 0.5)    # [m] tight tracking in turns
        self.declare_parameter("lookahead_max", 1.0)    # [m] smooth on straights
        self.declare_parameter("lookahead_gain", 1.0)   # [s] Ld per unit speed
        # pivot in place when the carrot is more than this off-heading.
        self.declare_parameter("turn_in_place_angle", 0.7)   # [rad] ~40 deg
        self.declare_parameter("turn_in_place_gain", 2.0)    # P gain for the pivot
        self.declare_parameter("slow_gain", 0.8)             # speed drop vs curvature
        # frames: /plan is in the map frame, so the robot pose must be too — read it
        # from TF map->base (AMCL-corrected), not straight from /odom (which drifts).
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        self.path_pts: list[tuple[float, float]] = []
        self.reached = False

        # TF: robot pose in the map frame (the whole point of Phase 2).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.compute_pub = self.create_publisher(Float64, "controller/compute_ms", 10)
        self.get_logger().info("pure_pursuit up: waiting for /plan, /odom, and TF map->base")

    # ------------------------------------------------------------------ #
    def _on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.reached = False
        self.get_logger().info(f"received path with {len(self.path_pts)} points")

    def _robot_pose_in_map(self):
        """(x, y, yaw) of the robot in the map frame from TF. None if unavailable."""
        gf = self.get_parameter("global_frame").value
        bf = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                gf, bf, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def _on_odom(self, msg: Odometry):
        if not self.path_pts or self.reached:
            return

        # measured forward speed (body frame) still comes from odom twist
        v_meas = msg.twist.twist.linear.x
        # robot pose in the MAP frame from TF (AMCL-corrected) — this is what makes
        # localization take effect: /plan is in map, so the pose must be too.
        pose = self._robot_pose_in_map()
        if pose is None:
            return
        px, py, yaw = pose

        # stop when close enough to the final path point
        gx, gy = self.path_pts[-1]
        if math.hypot(gx - px, gy - py) < self.get_parameter("goal_tolerance").value:
            self.reached = True
            self._publish(0.0, 0.0)
            self.get_logger().info("goal reached — stopping")
            return

        t0 = time.perf_counter()
        # (1) adaptive lookahead from the measured speed, then pick the carrot forward.
        ld = adaptive_lookahead(v_meas,
                                self.get_parameter("lookahead_gain").value,
                                self.get_parameter("lookahead_min").value,
                                self.get_parameter("lookahead_max").value)
        target = lookahead_point(self.path_pts, px, py, ld) or (gx, gy)

        # (2)+(3) command: pivot if far off-heading, else drive with curvature slowdown.
        v, w, _ = velocity_command(
            target, px, py, yaw,
            max_linear=self.get_parameter("max_linear").value,
            min_linear=self.get_parameter("min_linear").value,
            max_angular=self.get_parameter("max_angular").value,
            turn_in_place_angle=self.get_parameter("turn_in_place_angle").value,
            turn_in_place_gain=self.get_parameter("turn_in_place_gain").value,
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
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
