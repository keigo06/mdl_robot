#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, Pose


class SimpleCubePublisher(Node):

    def __init__(self):
        super().__init__('simple_cube_publisher')
        self.publisher_ = self.create_publisher(
            Pose,
            'cube_pose',
            10
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.position.x = 1.0
        msg.position.y = 1.0
        msg.position.z = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(
            'Publishing cube pose: "[%f, %f, %f]"' % (
                msg.position.x, msg.position.y, msg.position.z))
        self.get_logger().info(
            'Publishing cube orientation: "[%f, %f, %f, %f]"' % (
                msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    simple_cube_publisher = SimpleCubePublisher()

    rclpy.spin(simple_cube_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_cube_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
