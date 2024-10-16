#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
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
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Cubeオブジェクトのリストを作成
        self.cubes = [Cube(i, [0.0, 0.0, 0.12 * (i + 1)], [0.0, 0.0, 0.0])
                      for i in range(num_cubes)]

        # キューブの色の設定
        cmap = plt.get_cmap('tab20')
        self.colors = [cmap(i % 20) for i in range(num_cubes)]

        # コネクタのオフセット（キューブの中心から面までの距離）
        self.face_offset = 0.06

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

            # CubeのMarkerメッセージの作成
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

            # CubeのTransformStampedの作成とブロードキャスト
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

            # コネクタのブロードキャスト
            self.broadcast_connector_transform(cube, i)

        # MarkerArrayのパブリッシュ
        self.publisher_.publish(marker_array)

    def broadcast_connector_transform(self, cube, cube_id):
        """
        各面の結合面（connector）の位置を静的にbroadcastする
        :param cube: 対象のキューブ
        :param cube_id: キューブのID
        """
        face_offsets = [
            (self.face_offset, 0.0, 0.0),   # +X
            (-self.face_offset, 0.0, 0.0),  # -X
            (0.0, self.face_offset, 0.0),   # +Y
            (0.0, -self.face_offset, 0.0),  # -Y
            (0.0, 0.0, self.face_offset),   # +Z
            (0.0, 0.0, -self.face_offset)   # -Z
        ]

        for face_id, offset in enumerate(face_offsets):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f'cube_frame_{cube_id}'
            t.child_frame_id = f'cube_{cube_id}_face_{face_id}_connector'
            t.transform.translation.x = offset[0]
            t.transform.translation.y = offset[1]
            t.transform.translation.z = offset[2]

            # 面の姿勢 (ここでは回転はなし)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # 静的な座標変換のブロードキャスト
            self.static_tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CubeNode(num_cubes=5)  # 5個のキューブを作成
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
