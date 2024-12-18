#!/usr/bin/env python3

""" cube_node.py - A ROS2 node to visualize the assembly of cubes in RViz.
This node manages the assembly of cubes and broadcasts their poses as TFs.
This node use the tf of the body_link of cube.
This node does not use the tfs of the faces of cubes.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster


class CubeNode(Node):
    def __init__(self):
        super().__init__('cube_node')
        self.mdl_cube_pose_sub = self.create_subscription(
            PoseStamped,
            'cube_pose',
            self.update_cube_pose,
            10
        )

        # initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def update_cube_pose(self, msg):
        self.get_logger().info(
            'Received cube name: "%s"' % msg.header.frame_id)
        self.get_logger().info(
            'Received cube pose: "[%f, %f, %f]"' % (
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        self.get_logger().info(
            'Received cube orientation: "[%f, %f, %f, %f]"' % (
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w))

        # get the cube pose from the message
        cube_pose = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        cube_attitude = np.array(
            [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w])

        # broadcast the transform
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = msg.header.frame_id

        t.transform.translation.x = cube_pose[0]
        t.transform.translation.y = cube_pose[1]
        t.transform.translation.z = cube_pose[2]

        t.transform.rotation.x = cube_attitude[0]
        t.transform.rotation.y = cube_attitude[1]
        t.transform.rotation.z = cube_attitude[2]
        t.transform.rotation.w = cube_attitude[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    cube_node = CubeNode()
    rclpy.spin(cube_node)
    cube_node.destroy_node

    rclpy.shutdown()


if __name__ == '__main__':
    main()
