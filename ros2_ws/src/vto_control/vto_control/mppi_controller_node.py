"""MPPI path-tracking controller — drop-in alternative to Pure Pursuit / MPC.

Same interface (so all three A/B on the same benchmark tour):
    /plan (map frame) + TF map->base pose + /odom twist  ->  /cmd_vel

Each tick: build an N-point reference window from the path, then MPPICore samples K
rollouts and returns the cost-weighted control. Runs on a fixed-rate timer. See docs/15.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from vto_control.mppi_core import MPPICore
from vto_control.pursuit_core import sample_reference


class MPPIController(Node):
    def __init__(self):
        super().__init__("mppi_controller")
        # --- horizon / sampling ----------------------------------------
        self.declare_parameter("N", 20)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("K", 1000)          # rollouts per tick
        self.declare_parameter("lambda", 1.0)      # path-integral temperature
        self.declare_parameter("sigma_v", 0.25)    # sampling noise (linear)
        self.declare_parameter("sigma_w", 0.6)     # sampling noise (angular)
        # --- limits (forward-only: v_min=0, see docs/14) ---------------
        self.declare_parameter("v_min", 0.0)
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("w_max", 1.5)
        # --- cost weights ----------------------------------------------
        self.declare_parameter("w_pos", 12.0)
        self.declare_parameter("w_theta", 0.5)
        self.declare_parameter("w_ctrl", 0.02)
        # --- behavior --------------------------------------------------
        self.declare_parameter("ref_speed", 0.5)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        gp = self.get_parameter
        self.N = gp("N").value
        self.dt = gp("dt").value
        self.mppi = MPPICore(
            N=self.N, dt=self.dt, K=gp("K").value, lambda_=gp("lambda").value,
            sigma_v=gp("sigma_v").value, sigma_w=gp("sigma_w").value,
            v_min=gp("v_min").value, v_max=gp("v_max").value, w_max=gp("w_max").value,
            w_pos=gp("w_pos").value, w_theta=gp("w_theta").value,
            w_ctrl=gp("w_ctrl").value)

        self.path_pts: list[tuple[float, float]] = []
        self.reached = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.compute_pub = self.create_publisher(Float64, "controller/compute_ms", 10)
        self.timer = self.create_timer(1.0 / gp("control_rate").value, self._control)
        self.get_logger().info("mppi_controller up: waiting for /plan, /odom, and TF map->base")

    # ------------------------------------------------------------------ #
    def _on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.reached = False
        self.mppi.reset()                       # don't carry a stale nominal into a new goal
        self.get_logger().info(f"received path with {len(self.path_pts)} points")

    def _on_odom(self, msg: Odometry):
        pass                                    # MPPI needs no u_prev anchor

    def _robot_pose_in_map(self):
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

    # ------------------------------------------------------------------ #
    def _control(self):
        if not self.path_pts or self.reached:
            return
        pose = self._robot_pose_in_map()
        if pose is None:
            return
        px, py, yaw = pose

        gx, gy = self.path_pts[-1]
        if math.hypot(gx - px, gy - py) < self.get_parameter("goal_tolerance").value:
            self.reached = True
            self._publish(0.0, 0.0)
            self.get_logger().info("goal reached — stopping")
            return

        ds = self.get_parameter("ref_speed").value * self.dt
        ref = np.array(sample_reference(self.path_pts, px, py, self.N + 1, ds))

        t0 = time.perf_counter()
        v, w = self.mppi.control(np.array([px, py, yaw]), ref)
        self.compute_pub.publish(Float64(data=(time.perf_counter() - t0) * 1000.0))
        self._publish(v, w)

    def _publish(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = MPPIController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
