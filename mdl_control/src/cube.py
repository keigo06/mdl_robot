#!/usr/bin/env python3

class Cube:
    def __init__(self, cube_id, pos, euler_angles):
        """
        キューブの初期化
        :param cube_id: キューブのID
        :param pos: キューブの初期位置 [x, y, z]
        :param euler_angles: キューブの初期姿勢 [roll, pitch, yaw]
        """
        self.cube_id = cube_id
        self.pos = pos  # [x, y, z]
        self.euler_angles = euler_angles  # [roll, pitch, yaw]

        # 各面のconnectorを初期化 (0-5の各面が持つconnector)
        # 初期状態は全てactiveのfemaleモードに設定
        self.connectors = [{'type': 'active', 'mode': 'female'}
                           for _ in range(6)]

    def set_connector(self, face, connector_type, mode=None):
        """
        setting connector type and mode
        :param face: どの面か (0-5)
        :param connector_type: 'active' または 'passive' または 'cannot_connect'
        :param mode: 'male' または 'female' (activeの場合)
        """
        self.connectors[face]['type'] = connector_type
        if connector_type == 'active':
            self.connectors[face]['mode'] = mode

    def set_module_type(self, module_type):
        self.module_type = module_type

        if module_type == 'active_6_passive_0':
            # 6つの面がactive, 0つの面がpassive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(6)]

        elif module_type == 'active_5_passive_1':
            # 5つの面がactive, 1つの面がpassive
            self.connectors =\
                [{'type': 'passive', 'mode': None}]
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'} for _ in range(5)])

        elif module_type == 'active_4_passive_2_near':
            # 4つの面がactive, 2つの面(near)がpassive
            self.connectors =\
                [{'type': 'passive', 'mode': None} for _ in range(2)]
            self.connectors.extend(
                [{'type': 'active', 'mode': 'male'} for _ in range(4)])

        elif module_type == 'active_4_passive_2_opposite':
            # 4つの面がactive, 2つの面(opposite)がpassive
            self.connectors =\
                [{'type': 'passive', 'mode': None}]
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'} for _ in range(2)])
            self.connectors.extend(
                [{'type': 'passive', 'mode': None}])
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'} for _ in range(2)])

        elif module_type == 'active_3_passive_3_near':
            # 3つの面(near)がactive, 3つの面がpassive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(3)]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(3)])

        elif module_type == 'active_3_passive_3_konoji':
            # 3つの面(konoji)がactive, 3つの面がpassive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(2)]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(2)])
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'}])
            self.connectors.extend(
                [{'type': 'passive', 'mode': None}])

        elif module_type == 'active_2_passive_4_near':
            # 2つの面(near)がactive, 4つの面がpassive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(2)]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(4)])

        elif module_type == 'active_2_passive_4_opposite':
            # 2つの面(opposite)がactive, 4つの面がpassive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'}]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(2)])
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'}])
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(2)])

        elif module_type == 'active_1_passive_5':
            # 1つの面がactive, 5つの面がpassive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'}]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(5)])

        elif module_type == 'active_0_passive_6':
            # 0つの面がactive, 6つの面がpassive
            self.connectors =\
                [{'type': 'passive', 'mode': None} for _ in range(6)]

        else:
            raise ValueError(f"Invalid module type: {module_type}")

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
        self.pos = new_position

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
            'pos': self.pos,
            'euler_angles': self.euler_angles
        }
