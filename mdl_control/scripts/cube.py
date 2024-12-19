#!/usr/bin/env python3

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R


class Cube:
    def __init__(self, cube_id: int, pos: npt.NDArray, attitude: npt.NDArray):
        """
        :param cube_id:         cube ID
        :param pos:             cube position np.array([x, y, z])
        :param attitude:        cube attitude quaternion np.array([x, y, z, w])
        """
        self.m_size = 0.12
        self.cube_id: int = self.validate_id(cube_id)
        self.pos: npt.NDArray = self.validate_position(pos)
        self.attitude: npt.NDArray = self.validate_attitude(attitude)
        self.cube_type: str = 'active_6_passive_0'
        self.connectors: list[dict] = []
        self.set_cube_type(self.cube_type)

    @staticmethod
    def validate_id(cube_id: int) -> int:
        if cube_id is None or cube_id < 0:
            raise ValueError('invalid cube_id')
        return cube_id

    @staticmethod
    def validate_position(pos: npt.NDArray) -> npt.NDArray:
        if pos is None or len(pos) != 3:
            raise ValueError('invalid pos')
        return np.array(pos)

    @staticmethod
    def validate_attitude(attitude: npt.NDArray) -> npt.NDArray:
        if attitude is None or len(attitude) != 4:
            raise ValueError('invalid attitude')
        return np.array(attitude)

    def set_cube_type(self, cube_type: str):
        self.cube_type: str = cube_type

        if cube_type == 'active_6_passive_0':
            # 6 faces: active / 0 faces: passive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(6)]

        elif cube_type == 'active_5_passive_1':
            # 5 faces: active / 1 faces: passive
            self.connectors =\
                [{'type': 'passive', 'mode': None}]
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'} for _ in range(5)])

        elif cube_type == 'active_4_passive_2_near':
            # 4 faces: active / 2 faces(near): passive
            self.connectors =\
                [{'type': 'passive', 'mode': None} for _ in range(2)]
            self.connectors.extend(
                [{'type': 'active', 'mode': 'male'} for _ in range(4)])

        elif cube_type == 'active_4_passive_2_opposite':
            # 4 faces: active / 2 faces(opposite): passive
            self.connectors =\
                [{'type': 'passive', 'mode': None}]
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'} for _ in range(2)])
            self.connectors.extend(
                [{'type': 'passive', 'mode': None}])
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'} for _ in range(2)])

        elif cube_type == 'active_3_passive_3_near':
            # 3 faces(near): active / 3 faces: passive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(3)]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(3)])

        elif cube_type == 'active_3_passive_3_konoji':
            # 3 faces(konoji): active / 3 faces: passive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(2)]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(2)])
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'}])
            self.connectors.extend(
                [{'type': 'passive', 'mode': None}])

        elif cube_type == 'active_2_passive_4_near':
            # 2 faces(near): active / 4 faces: passive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'} for _ in range(2)]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(4)])

        elif cube_type == 'active_2_passive_4_opposite':
            # 2 faces(opposite): active / 4 faces: passive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'}]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(2)])
            self.connectors.extend(
                [{'type': 'active', 'mode': 'female'}])
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(2)])

        elif cube_type == 'active_1_passive_5':
            # 1 face: active / 5 faces: passive
            self.connectors =\
                [{'type': 'active', 'mode': 'female'}]
            self.connectors.extend(
                [{'type': 'passive', 'mode': None} for _ in range(5)])

        elif cube_type == 'active_0_passive_6':
            # 0 faces: active / 6 faces: passive
            self.connectors =\
                [{'type': 'passive', 'mode': None} for _ in range(6)]

        else:
            raise ValueError(f"Invalid cube type: {cube_type}")

    def set_connector(self, face: int, connector_type: str, mode: str = None):
        """
        setting connector type and mode
        :param face:            0-5
        :param connector_type:  'active' or 'passive' or 'cannot_connect'
        :param mode:            'male'or'female' if connector_type is 'active'
        """
        self.connectors[face]['type'] = connector_type
        if connector_type == 'active':
            self.connectors[face]['mode'] = mode

    def change_active_connector_mode(self, face: int, mode: str):
        """
        change active connector mode
        :param face:    0-5
        :param mode:    'male' or 'female'
        """
        if mode not in ['male', 'female']:
            raise ValueError(f"Invalid mode: {mode}")
        elif self.connectors[face]['type'] == 'active':
            self.connectors[face]['mode'] = mode

    def get_connector(self, face: int) -> dict:
        """
        指定された面の結合面情報を取得する
        :param face: 面 (0-5)
        :return: connector情報
        """
        return self.connectors[face]

    def get_state(self):
        """
        キューブの現在の状態（位置と姿勢）を返す
        :return: 現在の位置と姿勢を辞書形式で返す
        """
        return {
            'pos': self.pos,
            'attitude': self.attitude
        }

    def rotate(self, rotation_matrix: npt.NDArray):
        """
        任意の回転行列を使用してキューブの姿勢を変化させる
        :param rotation_matrix: 3x3の回転行列
        """
        r = R.from_matrix(rotation_matrix)
        new_attitude = r * R.from_quat(self.attitude)
        self.attitude = new_attitude.as_quat()

    def rotate_x_90(self):
        """
        x軸に対して90度回転させる
        """
        rotation_matrix = R.from_euler('x', 90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_x_minus_90(self):
        """
        x軸に対して-90度回転させる
        """
        rotation_matrix = R.from_euler('x', -90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_y_90(self):
        """
        y軸に対して90度回転させる
        """
        rotation_matrix = R.from_euler('y', 90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_y_minus_90(self):
        """
        y軸に対して-90度回転させる
        """
        rotation_matrix = R.from_euler('y', -90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_z_180(self):
        """
        z軸に対して180度回転させる
        """
        rotation_matrix = R.from_euler('z', 180, degrees=True).as_matrix()
        self.rotate(rotation_matrix)
