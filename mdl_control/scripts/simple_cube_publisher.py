#!/usr/bin/env python3

""" simple_cube_publisher.py - A ROS2 node to publish the pose of a cube.
This node publishes the pose of a cube as a PoseStamped message.
This node manages simple action of cubes.
There are 6 cubes in the assembly.
In timer_callback, the poses of the all cubes are published.
But, only one cube will be moved in the assembly now.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

import numpy as np


class SimpleCubePublisher(Node):

    def __init__(self):
        super().__init__('simple_cube_publisher')
        self.publisher_ = self.create_publisher(
            PoseStamped,
            'cube_pose',
            10
        )

        self.example_actions = [
            {'cube_name': 'mdl_cube_02_body_link',
             'pos': [0.24, 0.00, 0.06], 'attitude': [0.0, 0.0, 0.0, 1.0]},
            {'cube_name': 'mdl_cube_01_body_link',
             'pos': [0.24, 0.00, 0.18], 'attitude': [0.0, 0.0, 0.0, 1.0]},
            {'cube_name': 'mdl_cube_00_body_link',
             'pos': [0.24, 0.00, 0.30], 'attitude': [0.0, 0.0, 0.0, 1.0]},
            {'cube_name': 'mdl_cube_05_body_link',
             'pos': [0.24, 0.00, 0.42], 'attitude': [0.0, 0.0, 0.0, 1.0]},
            {'cube_name': 'mdl_cube_04_body_link',
             'pos': [0.24, 0.00, 0.54], 'attitude': [0.0, 0.0, 0.0, 1.0]},
            {'cube_name': 'mdl_cube_03_body_link',
             'pos': [0.24, 0.00, 0.66], 'attitude': [0.0, 0.0, 0.0, 1.0]},
        ]
        self.current_action_index = 0

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.current_action_index < len(self.example_actions):
            action = self.example_actions[self.current_action_index]
            msg = PoseStamped()
            msg.header.frame_id = action['cube_name']
            msg.pose.position.x = action['pos'][0]
            msg.pose.position.y = action['pos'][1]
            msg.pose.position.z = action['pos'][2]
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            self.publisher_.publish(msg)
            self.get_logger().info(
                'Publishing cube name: "%s"' % msg.header.frame_id)
            self.get_logger().info(
                'Publishing cube pose: "[%f, %f, %f]"' % (
                    msg.pose.position.x, msg.pose.position.y,
                    msg.pose.position.z))
            self.get_logger().info(
                'Publishing cube orientation: "[%f, %f, %f, %f]"' % (
                    msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w))

            self.current_action_index += 1
        else:
            self.get_logger().info('All actions have been published.')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    simple_cube_publisher = SimpleCubePublisher()
    rclpy.spin(simple_cube_publisher)
    simple_cube_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
