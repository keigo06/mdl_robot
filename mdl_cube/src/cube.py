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

        # 各面のconnectorを初期化 (0-5の各面が持つconnector)
        # 初期状態は全てactiveのmaleモードに設定
        self.connectors = [{'type': 'active', 'mode': 'male'}
                           for _ in range(6)]

    def set_connector(self, face, connector_type, mode=None):
        """
        各面の結合面を設定する
        :param face: どの面か (0-5)
        :param connector_type: 'active' または 'passive'
        :param mode: 'male' または 'female' (activeの場合)
        """
        self.connectors[face]['type'] = connector_type
        if connector_type == 'active':
            self.connectors[face]['mode'] = mode

    def get_connector(self, face):
        """
        指定された面の結合面情報を取得する
        :param face: 面 (0-5)
        :return: connector情報
        """
        return self.connectors[face]

    def update_position(self, new_position):
        """
        キューブの位置を更新する
        """
        self.position = new_position

    def update_orientation(self, new_euler_angles):
        """
        キューブの姿勢を更新する
        """
        self.euler_angles = new_euler_angles

    def get_state(self):
        """
        キューブの現在の状態（位置と姿勢）を返す
        :return: 現在の位置と姿勢を辞書形式で返す
        """
        return {
            'position': self.position,
            'euler_angles': self.euler_angles
        }
