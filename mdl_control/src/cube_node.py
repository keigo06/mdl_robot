""" cube_node.py - A ROS2 node to visualize the assembly of cubes in RViz.
This node manages the assembly of cubes and broadcasts their poses as TFs.
This node use the tfs of the body_link of cubes.
This node does not use the tfs of the faces of cubes.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


class CubeNode(Node):
    def __init__(self):
        super().__init__('cube_node')
        self.tf_broadcaster = TransformBroadcaster(self)


def main(args=None):
    rclpy.init(args=args)
    node = CubeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
