"""Publish a test path (nav_msgs/Path) on /plan for the Pure Pursuit follower.

Phase-1 stand-in for a real planner (the C++ core planner arrives in Phase 1.5).
Emits a smooth quarter-circle arc from the origin, curving left to ~(2, 2), in the
`odom` frame. Re-published on a timer so late subscribers still receive it.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("radius", 2.0)
        self.declare_parameter("num_points", 25)

        self.pub = self.create_publisher(Path, "/plan", 10)
        self.timer = self.create_timer(1.0, self._publish)

    def _publish(self):
        frame = self.get_parameter("frame_id").value
        r = self.get_parameter("radius").value
        n = self.get_parameter("num_points").value

        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()

        # arc centered at (0, r): angle a from -pi/2 -> 0 gives (0,0) -> (r, r)
        for i in range(n):
            a = -math.pi / 2 + (math.pi / 2) * i / (n - 1)
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose.position.x = r * math.cos(a)
            ps.pose.position.y = r + r * math.sin(a)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub.publish(path)


def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
