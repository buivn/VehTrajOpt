"""MPC path-tracking controller — drop-in alternative to Pure Pursuit.

Same interface (so we can A/B them on identical goals for the report):
    /plan (map frame) + TF map->base pose + /odom twist  ->  /cmd_vel

Each control tick it turns the global path into an N-point reference window
(pursuit_core.sample_reference), then solves the receding-horizon OCP
(mpc_core.MPCCore) and publishes the first control. See docs/13.

The solve (~10 ms) runs on a fixed-rate TIMER (not the 100 Hz odom callback) so we
control the compute budget. Pose comes from TF (AMCL-corrected); /odom twist is the
current (v, ω) used as the accel/comfort anchor.
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

from vto_control.mpc_core import MPCCore
from vto_control.pursuit_core import sample_reference


class MPCController(Node):
    def __init__(self):
        super().__init__("mpc_controller")
        # --- horizon / limits ------------------------------------------
        self.declare_parameter("N", 20)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("w_max", 1.5)
        self.declare_parameter("dv_max", 0.15)
        self.declare_parameter("dw_max", 0.30)
        # --- cost weights (the controller's "personality") -------------
        self.declare_parameter("q_pos", 10.0)
        self.declare_parameter("q_theta", 0.5)
        self.declare_parameter("r_v", 0.1)
        self.declare_parameter("r_w", 0.05)
        self.declare_parameter("s_v", 1.0)
        self.declare_parameter("s_w", 0.5)
        self.declare_parameter("qf_pos", 50.0)
        self.declare_parameter("qf_theta", 2.0)
        # --- behavior --------------------------------------------------
        self.declare_parameter("ref_speed", 0.5)       # [m/s] ref-window spacing = ref_speed*dt
        self.declare_parameter("goal_tolerance", 0.25)  # [m]
        self.declare_parameter("control_rate", 20.0)   # [Hz] MPC solve rate
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        gp = self.get_parameter
        self.N = gp("N").value
        self.dt = gp("dt").value
        self.mpc = MPCCore(
            N=self.N, dt=self.dt,
            v_max=gp("v_max").value, w_max=gp("w_max").value,
            dv_max=gp("dv_max").value, dw_max=gp("dw_max").value,
            q_pos=gp("q_pos").value, q_theta=gp("q_theta").value,
            r_v=gp("r_v").value, r_w=gp("r_w").value,
            s_v=gp("s_v").value, s_w=gp("s_w").value,
            qf_pos=gp("qf_pos").value, qf_theta=gp("qf_theta").value,
        )

        self.path_pts: list[tuple[float, float]] = []
        self.reached = False
        self.u_prev = (0.0, 0.0)        # current (v, ω) from odom, anchors accel/comfort

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.compute_pub = self.create_publisher(Float64, "controller/compute_ms", 10)
        self.timer = self.create_timer(1.0 / gp("control_rate").value, self._control)
        self.get_logger().info("mpc_controller up: waiting for /plan, /odom, and TF map->base")

    # ------------------------------------------------------------------ #
    def _on_path(self, msg: Path):
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.reached = False
        self.get_logger().info(f"received path with {len(self.path_pts)} points")

    def _on_odom(self, msg: Odometry):
        # measured (v, ω): anchors the accel/comfort limits to the robot's real speed
        self.u_prev = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

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

        # global path -> N+1 reference states over the horizon (a moving window)
        ds = self.get_parameter("ref_speed").value * self.dt
        ref = sample_reference(self.path_pts, px, py, self.N + 1, ds)
        ref_arr = np.array(ref).T                                  # (3, N+1)

        t0 = time.perf_counter()
        v, w = self.mpc.solve(np.array([px, py, yaw]),
                              np.array(self.u_prev), ref_arr)
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
    node = MPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
