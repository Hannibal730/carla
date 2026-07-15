"""Placeholder node for the final automatic-parking controller."""

import rclpy
from rclpy.node import Node


class Parking(Node):
    """Empty node skeleton reserved for parking-control logic."""

    def __init__(self) -> None:
        super().__init__('parking')
        self.get_logger().info(
            'parking node started (placeholder implementation).')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Parking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
