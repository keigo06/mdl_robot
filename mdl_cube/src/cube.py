#!/usr/bin/env python3

class Cube:
    def __init__(self, cube_id, position, euler_angles):
        """
        キューブの初期化
        :param cube_id: キューブのID
        :param position: キューブの初期位置 [x, y, z]
        :param euler_angles: キューブの初期姿勢 [roll, pitch, yaw]
        """
        self.cube_id = cube_id
        self.position = position  # [x, y, z]
        self.euler_angles = euler_angles  # [roll, pitch, yaw]

    def update_position(self, new_position):
        """
        キューブの位置を更新する
        :param new_position: 新しい位置 [x, y, z]
        """
        self.position = new_position

    def update_orientation(self, new_euler_angles):
        """
        キューブの姿勢を更新する
        :param new_euler_angles: 新しい姿勢 [roll, pitch, yaw]
        """
        self.euler_angles = new_euler_angles

    def get_state(self):
        """
        キューブの現在の状態を取得する
        :return: キューブの状態（位置と姿勢）
        """
        return {
            'id': self.cube_id,
            'position': self.position,
            'euler_angles': self.euler_angles
        }
