#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from transforms3d.euler import euler2quat
from cube import Cube
import matplotlib.pyplot as plt


class CubeNode(Node):
    def __init__(self, num_cubes=5):
        super().__init__('cube_node')

        # ROS関連の設定
        self.publisher_ = self.create_publisher(MarkerArray, 'cubes', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Cubeオブジェクトのリストを作成
        self.cubes = [Cube(i, [0.0, 0.0, 0.12 * (i + 1)], [0.0, 0.0, 0.0])
                      for i in range(num_cubes)]

        # キューブの色の設定
        cmap = plt.get_cmap('tab20')
        self.colors = [cmap(i % 20) for i in range(num_cubes)]

    def publish_and_broadcast(self):
        """
        キューブの状態をパブリッシュ・ブロードキャストするメソッド。
        任意のタイミングでも呼び出し可能。
        """
        marker_array = MarkerArray()

        for i, cube in enumerate(self.cubes):
            state = cube.get_state()
            position = state['position']
            euler_angles = state['euler_angles']
            qw, qx, qy, qz = euler2quat(*euler_angles)

            # Markerメッセージの作成
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'world'
            marker.ns = 'cube'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12

            # 各キューブに色を割り当て
            color = self.colors[i]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8

            marker_array.markers.append(marker)

            # TransformStampedの作成とブロードキャスト
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = f'cube_frame_{i}'
            t.transform.translation.x = position[0]
            t.transform.translation.y = position[1]
            t.transform.translation.z = position[2]
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            # 座標変換のブロードキャスト
            self.tf_broadcaster.sendTransform(t)

        # MarkerArrayのパブリッシュ
        self.publisher_.publish(marker_array)
