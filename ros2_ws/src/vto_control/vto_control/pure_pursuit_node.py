"""Pure Pursuit path tracker (Phase 1).

The classic geometric path follower. Given a path (nav_msgs/Path) and the robot's
pose (nav_msgs/Odometry), it repeatedly:
  1. picks a "lookahead" point on the path a fixed distance ahead of the robot,
  2. computes the curvature of the arc that connects the robot to that point,
  3. commands a forward speed + the matching angular speed.

Output is geometry_msgs/TwistStamped on /cmd_vel (what Jazzy's
diff_drive_controller expects). This is the Phase-1 *baseline*: MPC and MPPI will
implement the same interface (Path + Odometry in -> /cmd_vel out) so all three
are directly comparable in vto_bench.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        # --- tunables ---------------------------------------------------
        self.declare_parameter("lookahead", 0.6)        # [m] carrot distance
        self.declare_parameter("max_linear", 0.5)       # [m/s]
        self.declare_parameter("max_angular", 1.5)      # [rad/s]
        self.declare_parameter("goal_tolerance", 0.25)  # [m] stop within this of the last point

        self.path_pts: list[tuple[float, float]] = []
        self.reached = False

        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.get_logger().info("pure_pursuit up: waiting for /plan and /odom")

    # ------------------------------------------------------------------ #
    def _on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.reached = False
        self.get_logger().info(f"received path with {len(self.path_pts)} points")

    def _on_odom(self, msg: Odometry):
        if not self.path_pts or self.reached:
            return

        # robot pose in the odom frame
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # stop when close enough to the final path point
        gx, gy = self.path_pts[-1]
        if math.hypot(gx - px, gy - py) < self.get_parameter("goal_tolerance").value:
            self.reached = True
            self._publish(0.0, 0.0)
            self.get_logger().info("goal reached — stopping")
            return

        ld = self.get_parameter("lookahead").value
        target = self._lookahead_point(px, py, ld)
        if target is None:
            target = (gx, gy)  # near the end: aim straight at the goal

        # transform the target into the robot frame
        dx, dy = target[0] - px, target[1] - py
        x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        y_r = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        # pure-pursuit curvature: kappa = 2*y_r / Ld^2
        dist = max(math.hypot(x_r, y_r), 1e-6)
        curvature = 2.0 * y_r / (dist * dist)

        v = self.get_parameter("max_linear").value
        if x_r < 0.0:                       # target behind us: turn in place
            v = 0.0
        w = max(-self.get_parameter("max_angular").value,
                min(self.get_parameter("max_angular").value, v * curvature if v > 0 else curvature))
        self._publish(v, w)

    # ------------------------------------------------------------------ #
    def _closest_index(self, x, y):
        """Index of the path point nearest the robot (tracks progress)."""
        best_i, best_d = 0, float("inf")
        for i, (tx, ty) in enumerate(self.path_pts):
            d = (tx - x) ** 2 + (ty - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _lookahead_point(self, x, y, ld):
        """First path point >= `ld` away, searching *forward* from the closest
        point so the carrot is always ahead on the path, never behind us."""
        start = self._closest_index(x, y)
        for tx, ty in self.path_pts[start:]:
            if math.hypot(tx - x, ty - y) >= ld:
                return tx, ty
        return None  # ran off the end of the path → near the goal

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
