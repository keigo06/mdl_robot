""" cube_node.py - A ROS2 node to visualize the assembly of cubes in RViz.
This node manages the assembly of cubes and broadcasts their poses as TFs.
This node use the tfs of the body_link of cubes.
This node does not use the tfs of the faces of cubes.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


class CubeNode(Node):
    def __init__(self):
        super().__init__('cube_node')

        self.cube_name = self.declare_parameter(
            'cube_name', 'cube_01').get_parameter_value().string_value

        # initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def update_cube_pose(self, msg):
        # get the cube pose from assembly.py
        # TODO: Change Pose date from demo pose to the info from assembly.py

        cube_01_pose = np.array([1.0, 1.0, 1.0])
        cube_01_attitude = np.array([0.0, 0.0, 0.0, 1.0])

        # broadcast the transform
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.cube_name + '_body_link'

        t.transform.translation.x = cube_01_pose[0]
        t.transform.translation.y = cube_01_pose[1]
        t.transform.translation.z = cube_01_pose[2]

        t.transform.rotation.x = cube_01_attitude[0]
        t.transform.rotation.y = cube_01_attitude[1]
        t.transform.rotation.z = cube_01_attitude[2]
        t.transform.rotation.w = cube_01_attitude[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init()
    node = CubeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
