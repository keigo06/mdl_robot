#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mdl_robot.mdl_control.scripts.cube_visualizer import CubeNode


class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.cube_node = CubeNode()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.update_asm)
        self.color_publisher = self.create_publisher(String, 'cube_color', 10)
        self.start_seconds = self.get_clock().now().seconds_nanoseconds()[0]
        self.start_nanoseconds = self.get_clock(
        ).now().seconds_nanoseconds()[1]
        self.start_time = self.start_seconds + self.start_nanoseconds * 1e-9

    def update_asm(self):
        """
        キューブの位置を更新し、手動でパブリッシュとブロードキャストを行う。
        """
        # キューブの状態を手動でパブリッシュ・ブロードキャスト
        self.cube_node.broadcast_transforms()

    def change_cube_colors(self, colors):
        msg = String()
        msg.data = ' '.join(colors)
        self.color_publisher.publish(msg)

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()
    server = Server()
    server.run()


if __name__ == '__main__':
    main()
