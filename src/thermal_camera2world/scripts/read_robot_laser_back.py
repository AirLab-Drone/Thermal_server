#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from builtin_interfaces.msg import Time


class TfEchoNode(Node):
    def __init__(self):
        super().__init__('echo_robot_laser_back')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.target_frame = 'laser_back'
        self.source_frame = 'map'

        # 每 0.5 秒查詢一次
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            # 使用最新時間 (Time())
            transform = self.buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                Time()
            )
            self.get_logger().info(
                f"From {self.source_frame} to {self.target_frame}:\n"
                f"Translation: x={transform.transform.translation.x:.3f}, "
                f"y={transform.transform.translation.y:.3f}, "
                f"z={transform.transform.translation.z:.3f}\n"
                f"Rotation: x={transform.transform.rotation.x:.3f}, "
                f"y={transform.transform.rotation.y:.3f}, "
                f"z={transform.transform.rotation.z:.3f}, "
                f"w={transform.transform.rotation.w:.3f}"
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform not available: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TfEchoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
