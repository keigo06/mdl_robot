#!/usr/bin/env python3

import numpy as np
import numpy.typing as npt
from typing import Optional, Any
from scipy.spatial.transform import Rotation as R


class Cube:
    def __init__(self, cube_id: int, pos: npt.NDArray, attitude: npt.NDArray):
        """Cube class

        Args:
            cube_id (int): cube id
            pos (npt.NDArray): position (x, y, z)
            attitude (npt.NDArray): quaternion (x, y, z, w)

        Attributes:
            m_size (float):         size of the cube
            cube_id (int):          cube id
            pos (npt.NDArray):      position (x, y, z)
            attitude (npt.NDArray): quaternion (x, y, z, w)
            cube_type (str):        cube type

            connectors (list[dict[str, str]]):
            connectors[face_number]
            = { 'type': 'active' or 'passive' or 'cannot_connect',
                'mode': 'male' or 'female' or None
                'pos_abs': npt.NDArray[],
                'dir_abs': npt.NDArray[]
            }

            active_face_list (list[int]):   face numbers of active connectors
            passive_face_list (list[int]):  face numbers of passive connectors

            unit_vectors (list[npt.NDArray]):   unit vectors (not match with face number)

        Examples:
            cube.Cube(cube_id=0, pos=[0, 0, 0], attitude=[0, 0, 0, 1])
            face_id = 0
            cube.connectors [face_id]['type']           = 'active'
            cube.connectors [face_id]['mode']           = 'male'
            cube.connectors [face_id]['pos_abs']        = npt.NDArray[0.12, 0.0, 0.0]
            cube.connectors [face_id]['dir_abs']        = npt.NDArray[1.0, 0.0, 0.0]
            active_face_list = [0, 1, 2, 3, 4, 5]
            passive_face_list = []
        """
        self.m_size = 0.12
        self.cube_id: int = self.validate_id(cube_id)
        self.pos: npt.NDArray = self.validate_position(pos)
        self.attitude: npt.NDArray = self.validate_attitude(attitude)
        self.cube_type: str = 'active_6_passive_0'
        self.connectors: list[dict[str, Any]] = []
        self.set_cube_type(self.cube_type)
        self.unit_vectors: list[npt.NDArray] = [
            *np.eye(3),
            *-np.eye(3)
        ]
        self.update_connectors_pos_abs()
        self.update_connectors_dir_abs()

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
        self.cube_type = cube_type

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

    def set_connector(self, face: int, connector_type: str, mode: Optional[str] = None):
        """ Set connector type and mode
        Args:
            face (int): 0-5
            connector_type (str): 'active' or 'passive' or 'cannot_connect'
            mode (str): 'male' or 'female' or None
        """
        self.connectors[face]['type'] = connector_type
        if connector_type == 'active':
            self.connectors[face]['mode'] = mode

    def set_active_face_list(self):
        """ Set active face list
        """
        self.active_face_list = self.get_faces_by_connector_type('active')

    def update_connectors_pos_abs(self):
        """ Update faces position in the absolute coordinate
        """
        rotation_matrix = R.from_quat(self.attitude).as_matrix()
        unit_vectors_scaled = np.array(self.unit_vectors) * self.m_size/2
        self.faces_pos \
            = self.pos + np.dot(unit_vectors_scaled, rotation_matrix.T)
        for face_id in range(6):
            self.connectors[face_id]['pos_abs'] = self.faces_pos[face_id]

    def update_connectors_dir_abs(self):
        """ Update faces unit vectors in the absolute coordinate
        """
        for face_id in range(6):
            self.connectors[face_id]['dir_abs'] \
                = self.get_vector_abs_coordinate(self.unit_vectors[face_id])

    def change_active_connector_mode(self, face: int, mode: str):
        """ Change active connector mode
        Args:
            face (int): 0-5
            mode: 'male' or 'female'
        """
        if mode not in ['male', 'female']:
            raise ValueError(f"Invalid mode: {mode}")
        elif self.connectors[face]['type'] == 'active':
            self.connectors[face]['mode'] = mode

    def get_faces_by_connector_type(self, connector_type: str) -> list[int]:
        """ Get faces of specified connector type

        Args:
            connector_type (str): 'active' or 'passive' or 'cannot_connect'

        Returns:
            list[int]: A list of face numbers of specified connector type
        """
        return [face_id for face_id, connector in enumerate(self.connectors) if connector['type'] == connector_type]

    def get_vector_abs_coordinate(self, vector: npt.NDArray) -> npt.NDArray:
        """Change the relative coordinate to the absolute coordinate.

        Args:
            vector (npt.NDArray): The vector in the relative coordinate.

        Returns:
            npt.NDArray: The vector in the absolute coordinate.
        """

        return np.dot(R.from_quat(self.attitude).as_matrix(), vector)

    def get_pos_list_abs_of_faces_by_connector_type(self, connector_type: str) -> list[npt.NDArray]:
        """ Get positions of faces of specified connector type in the absolute coordinate

        Args:
            connector_type (str): 'active' or 'passive' or 'cannot_connect'

        Returns:
            list[npt.NDArray]: A list of positions of faces of specified connector type in the absolute coordinate
        """

        self.update_connectors_pos_abs()
        return [self.connectors[face_id]['pos_abs']
                for face_id in self.get_faces_by_connector_type(connector_type)]

    def get_state(self):
        return {
            'pos': self.pos,
            'attitude': self.attitude
        }

    def rotate(self, rotation_matrix: npt.NDArray):
        """ Rotate the cube
        Args:
            rotation_matrix (npt.NDArray): 3x3 rotation
        """
        r = R.from_matrix(rotation_matrix)
        new_attitude = r * R.from_quat(self.attitude)
        self.attitude = new_attitude.as_quat()

    def rotate_x_90(self):
        """ Rotate the cube 90 degrees around the x-axis
        """
        rotation_matrix = R.from_euler('x', 90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_x_minus_90(self):
        """ Rotate the cube -90 degrees around the x-axis
        """
        rotation_matrix = R.from_euler('x', -90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_y_90(self):
        """ Rotate the cube 90 degrees around the y-axis
        """
        rotation_matrix = R.from_euler('y', 90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_y_minus_90(self):
        """ Rotate the cube -90 degrees around the y-axis
        """
        rotation_matrix = R.from_euler('y', -90, degrees=True).as_matrix()
        self.rotate(rotation_matrix)

    def rotate_z_180(self):
        """ Rotate the cube 180 degrees around the z-axis
        """
        rotation_matrix = R.from_euler('z', 180, degrees=True).as_matrix()
        self.rotate(rotation_matrix)


if __name__ == "__main__":
    cube = Cube(cube_id=0, pos=np.array(
        [0, 0, 0]), attitude=np.array([0, 0, 0, 1]))
