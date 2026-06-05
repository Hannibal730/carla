"""
Generic odometry → Path accumulator for RViz visualization.
Topics and frame_id are configurable via ROS parameters:
  odom_topic  (default: /odometry/local)
  path_topic  (default: /path/local)
  frame_id    (default: odom)
  max_poses   (default: 2000)  — oldest poses are dropped beyond this limit.
                                 Prevents unbounded memory growth and DDS bloat
                                 during long drives.
"""
import collections
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('path_topic', '/path/local')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('max_poses', 1000)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        frame_id   = self.get_parameter('frame_id').get_parameter_value().string_value
        max_poses  = self.get_parameter('max_poses').get_parameter_value().integer_value

        self._deque: collections.deque[PoseStamped] = collections.deque(maxlen=max_poses)

        self._path = Path()
        self._path.header.frame_id = frame_id

        self.create_subscription(Odometry, odom_topic, self._cb, 10)
        self._pub = self.create_publisher(Path, path_topic, 10)
        self.create_timer(0.1, self._publish)  # 10 Hz
        self.get_logger().info(
            f'path_visualizer: {odom_topic} → {path_topic}  max_poses={max_poses}')

    def _cb(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._deque.append(pose)
        self._path.header.stamp = msg.header.stamp
        self._path.poses = list(self._deque)

    def _publish(self) -> None:
        if self._deque:
            self._pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
