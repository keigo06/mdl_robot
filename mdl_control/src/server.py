#!/usr/bin/env python3

from cube_node import CubeNode

import rclpy
from rclpy.node import Node

from heapq import heappush, heappop
import numpy as np
import math


class Server(Node):
    def __init__(self):
        super().__init__('server')
        self.cube_node = CubeNode(num_cubes=6)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.update_asm)
        self.start_seconds = self.get_clock().now().seconds_nanoseconds()[0]
        self.start_nanoseconds = self.get_clock(
        ).now().seconds_nanoseconds()[1]
        self.start_time = self.start_seconds + self.start_nanoseconds * 1e-9
        self.change_assembly_time = [10, 20]  # 10秒後にtower、20秒後にgridに変更
        self.current_assembly = 'line'
        self.timer_asm_change = self.create_timer(
            self.timer_period, self.cube_demo_3asm_change)

    def update_asm(self):
        """
        キューブの位置を更新し、手動でパブリッシュとブロードキャストを行う。
        """
        # current_seconds = self.get_clock().now().seconds_nanoseconds()[0]
        # current_nanoseconds = self.get_clock().now().seconds_nanoseconds()[1]
        # current_time = current_seconds + current_nanoseconds * 1e-9
        # pass_time = current_time - self.start_time

        # 各キューブの位置を時間に基づいて更新
        # self.cube_demo_movement(pass_time)

        # キューブの状態を手動でパブリッシュ・ブロードキャスト
        self.cube_node.publish_and_broadcast()

    def cube_demo_3asm_change(self):
        """
        時間経過に応じてアセンブリをlineからtower、gridに変更するデモ用の関数
        current_time: seconds + nanoseconds * 1e-9 [s]
        """
        current_seconds = self.get_clock().now().seconds_nanoseconds()[0]
        current_nanoseconds = self.get_clock().now().seconds_nanoseconds()[1]
        current_time = current_seconds + current_nanoseconds * 1e-9
        pass_time = current_time - self.start_time

        # 時間経過に応じてアセンブリを変更
        if pass_time > self.change_assembly_time[1] and self.current_assembly != 'grid':
            self.cube_node.assembly.create_grid_assembly()
            self.current_assembly = 'grid'
        elif pass_time > self.change_assembly_time[0] and self.current_assembly != 'tower':
            self.cube_node.assembly.create_tower_assembly()
            self.current_assembly = 'tower'

    def cube_demo_movement(self, pass_time):
        """
        デモ用の関数: 時間経過に応じてキューブの位置を更新する
        """
        for i, cube in enumerate(self.cube_node.assembly):
            new_x = 2.0 + math.cos(pass_time + self.offsets[i])  # X方向にコサインで移動
            new_y = 2.0 + math.sin(pass_time + self.offsets[i])  # Y方向にサインで移動
            cube.update_position([new_x, new_y, cube.position[2]])  # Zは固定

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
