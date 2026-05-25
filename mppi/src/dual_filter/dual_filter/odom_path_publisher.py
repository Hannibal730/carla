"""
Generic odometry → Path accumulator for RViz visualization.
Topics and frame_id are configurable via ROS parameters:
  odom_topic  (default: /odometry/local)
  path_topic  (default: /path/local)
  frame_id    (default: odom)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('path_topic', '/path/local')
        self.declare_parameter('frame_id', 'odom')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        frame_id   = self.get_parameter('frame_id').get_parameter_value().string_value

        self._path = Path()
        self._path.header.frame_id = frame_id

        self.create_subscription(Odometry, odom_topic, self._cb, 10)
        self._pub = self.create_publisher(Path, path_topic, 10)
        self.get_logger().info(f'odom_path_publisher: {odom_topic} → {path_topic}')

    def _cb(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self._path.header.stamp = msg.header.stamp
        self._path.poses.append(pose)
        self._pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
