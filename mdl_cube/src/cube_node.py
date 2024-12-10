#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from transforms3d.euler import euler2quat

class MultiCubePublisher(Node):
    def __init__(self, num_cubes=3):
        super().__init__('multi_cube_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'cubes', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.num_cubes = num_cubes
        self.cube_list = [{'id': i, 'position': [0.0, 0.0, 0.5 * (i + 1)], 'euler_angles': [0.0, 0.0, 0.0]} for i in range(self.num_cubes)]
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.broadcast_timer_callback)
        self.colors = [
            {'r': 1.0, 'g': 0.0, 'b': 0.0},
            {'r': 0.0, 'g': 1.0, 'b': 0.0},
            {'r': 0.0, 'g': 0.0, 'b': 1.0},
            {'r': 1.0, 'g': 1.0, 'b': 0.0},
            {'r': 1.0, 'g': 0.0, 'b': 1.0}
        ]

    def broadcast_timer_callback(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        current_time = seconds + nanoseconds * 1e-9

        marker_array = MarkerArray()

        for cube in self.cube_list:
            cube_id = cube['id']
            # Calculate positions for each cube
            position_x = 5.0 * math.cos(current_time + cube_id)
            position_y = 5.0 * math.sin(current_time + cube_id)
            position_z = cube['position'][2]

            # Update Euler angles to ensure x-axis points towards origin
            position_x = 5.0 * math.cos(current_time + cube_id)
            position_y = 5.0 * math.sin(current_time + cube_id)
            position_z = cube['position'][2]
            if cube_id % 2 == 0:
                yaw = math.atan2(-position_y, -position_x)  # Point x-axis towards origin
            else:
                yaw = 0.0
            roll, pitch = 0.0, 0.0
            cube['euler_angles'] = [roll, pitch, yaw]

            # Convert Euler angles to quaternion using transforms3d
            qw, qx, qy, qz = euler2quat(roll, pitch, yaw)

            # Create Marker message
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'world'
            marker.ns = 'cube'
            marker.id = cube_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = position_x
            marker.pose.position.y = position_y
            marker.pose.position.z = position_z
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            # Set color based on cube ID
            color = self.colors[cube_id % len(self.colors)]
            marker.color.r = color['r']
            marker.color.g = color['g']
            marker.color.b = color['b']
            marker.color.a = 0.5

            marker_array.markers.append(marker)

            self.get_logger().info('Publishing marker (ID: %d) at position (%.2f, %.2f, %.2f)' %
                                   (cube_id, marker.pose.position.x, marker.pose.position.y, marker.pose.position.z))

            # Create and send TransformStamped
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = f'cube_frame_{cube_id}'
            t.transform.translation.x = position_x
            t.transform.translation.y = position_y
            t.transform.translation.z = position_z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)

        # Publish MarkerArray
        self.publisher_.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    multi_cube_publisher = MultiCubePublisher(num_cubes=5)  # Adjust the number of cubes as needed

    try:
        rclpy.spin(multi_cube_publisher)
    except KeyboardInterrupt:
        pass
    multi_cube_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()