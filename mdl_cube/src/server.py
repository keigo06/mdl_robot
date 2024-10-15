#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cube_node import CubeNode
import math


class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.cube_node = CubeNode(num_cubes=5)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.update_cubes)
        self.offsets = [i * 0.5 for i in range(5)]  # 各キューブのオフセット時間
        self.start_seconds = self.get_clock().now().seconds_nanoseconds()[0]
        self.start_nanoseconds = self.get_clock(
        ).now().seconds_nanoseconds()[1]
        self.start_time = self.start_seconds + self.start_nanoseconds * 1e-9

    def update_cubes(self):
        current_seconds = self.get_clock().now().seconds_nanoseconds()[0]
        current_nanoseconds = self.get_clock().now().seconds_nanoseconds()[1]
        current_time = current_seconds + current_nanoseconds * 1e-9
        pass_time = current_time - self.start_time
        for i, cube in enumerate(self.cube_node.cubes):
            new_x = 2.0 + math.cos(pass_time + self.offsets[i])  # X方向にコサインで移動
            new_y = 2.0 + math.sin(pass_time + self.offsets[i])  # Y方向にサインで移動
            cube.update_position([new_x, new_y, cube.position[2]])  # Zは固定
        self.cube_node.timer_callback()  # cube_nodeのtimer_callbackで更新を反映

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
