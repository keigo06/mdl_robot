#!/usr/bin/env python3

from scripts.cube import Cube
import pytest

import numpy as np
from scipy.spatial.transform import Rotation as R

m_size = 0.12


def test_cube_initialization():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    assert cube.cube_id == 1
    assert np.array_equal(cube.pos, np.array([0, 0, 0]))
    assert np.array_equal(cube.attitude, np.array([0, 0, 0, 1]))
    assert len(cube.connectors) == 6
    assert cube.cube_type == 'active_6_passive_0'


def test_validate_id():
    with pytest.raises(ValueError):
        Cube.validate_id(None)
    with pytest.raises(ValueError):
        Cube.validate_id(-1)
    assert Cube.validate_id(1) == 1


def test_validate_position():
    with pytest.raises(ValueError):
        Cube.validate_position(None)
    with pytest.raises(ValueError):
        Cube.validate_position(np.array([0, 0]))
    assert np.array_equal(Cube.validate_position(
        np.array([0, 0, 0])), np.array([0, 0, 0]))


def test_validate_attitude():
    with pytest.raises(ValueError):
        Cube.validate_attitude(None)
    with pytest.raises(ValueError):
        Cube.validate_attitude(np.array([0, 0]))
    assert np.array_equal(Cube.validate_attitude(
        np.array([0, 0, 0, 1])), np.array([0, 0, 0, 1]))


def test_set_cube_type_active_6_passive_0():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_6_passive_0')
    assert all(connector['type'] == 'active' for connector in cube.connectors)
    assert all(connector['mode'] == 'female' for connector in cube.connectors)


def test_set_cube_type_active_5_passive_1():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_5_passive_1')
    assert cube.connectors[0]['type'] == 'passive'
    assert cube.connectors[0]['mode'] is None
    assert all(connector['type'] ==
               'active' for connector in cube.connectors[1:])
    assert all(connector['mode'] ==
               'female' for connector in cube.connectors[1:])


def test_set_cube_type_active_4_passive_2_near():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_4_passive_2_near')
    assert all(connector['type'] ==
               'passive' for connector in cube.connectors[:2])
    assert all(connector['mode'] is None for connector in cube.connectors[:2])
    assert all(connector['type'] ==
               'active' for connector in cube.connectors[2:])
    assert all(connector['mode'] ==
               'male' for connector in cube.connectors[2:])


def test_set_cube_type_active_4_passive_2_opposite():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_4_passive_2_opposite')
    assert cube.connectors[0]['type'] == 'passive'
    assert cube.connectors[0]['mode'] is None
    assert cube.connectors[1]['type'] == 'active'
    assert cube.connectors[1]['mode'] == 'female'
    assert cube.connectors[2]['type'] == 'active'
    assert cube.connectors[2]['mode'] == 'female'
    assert cube.connectors[3]['type'] == 'passive'
    assert cube.connectors[3]['mode'] is None
    assert cube.connectors[4]['type'] == 'active'
    assert cube.connectors[4]['mode'] == 'female'
    assert cube.connectors[5]['type'] == 'active'
    assert cube.connectors[5]['mode'] == 'female'


def test_set_cube_type_active_3_passive_3_near():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_3_passive_3_near')
    assert all(connector['type'] ==
               'active' for connector in cube.connectors[:3])
    assert all(connector['mode'] ==
               'female' for connector in cube.connectors[:3])
    assert all(connector['type'] ==
               'passive' for connector in cube.connectors[3:])
    assert all(connector['mode'] is None for connector in cube.connectors[3:])


def test_set_cube_type_active_3_passive_3_konoji():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_3_passive_3_konoji')
    assert cube.connectors[0]['type'] == 'active'
    assert cube.connectors[0]['mode'] == 'female'
    assert cube.connectors[1]['type'] == 'active'
    assert cube.connectors[1]['mode'] == 'female'
    assert cube.connectors[2]['type'] == 'passive'
    assert cube.connectors[2]['mode'] is None
    assert cube.connectors[3]['type'] == 'passive'
    assert cube.connectors[3]['mode'] is None
    assert cube.connectors[4]['type'] == 'active'
    assert cube.connectors[4]['mode'] == 'female'
    assert cube.connectors[5]['type'] == 'passive'
    assert cube.connectors[5]['mode'] is None


def test_set_cube_type_active_2_passive_4_near():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_2_passive_4_near')
    assert all(connector['type'] ==
               'active' for connector in cube.connectors[:2])
    assert all(connector['mode'] ==
               'female' for connector in cube.connectors[:2])
    assert all(connector['type'] ==
               'passive' for connector in cube.connectors[2:])
    assert all(connector['mode'] is None for connector in cube.connectors[2:])


def test_set_cube_type_active_2_passive_4_opposite():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_2_passive_4_opposite')
    assert cube.connectors[0]['type'] == 'active'
    assert cube.connectors[0]['mode'] == 'female'
    assert cube.connectors[1]['type'] == 'passive'
    assert cube.connectors[1]['mode'] is None
    assert cube.connectors[2]['type'] == 'passive'
    assert cube.connectors[2]['mode'] is None
    assert cube.connectors[3]['type'] == 'active'
    assert cube.connectors[3]['mode'] == 'female'
    assert cube.connectors[4]['type'] == 'passive'
    assert cube.connectors[4]['mode'] is None
    assert cube.connectors[5]['type'] == 'passive'
    assert cube.connectors[5]['mode'] is None


def test_set_cube_type_active_1_passive_5():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_1_passive_5')
    assert cube.connectors[0]['type'] == 'active'
    assert cube.connectors[0]['mode'] == 'female'
    assert all(connector['type'] ==
               'passive' for connector in cube.connectors[1:])
    assert all(connector['mode'] is None for connector in cube.connectors[1:])


def test_set_cube_type_active_0_passive_6():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_0_passive_6')
    assert all(connector['type'] == 'passive' for connector in cube.connectors)
    assert all(connector['mode'] is None for connector in cube.connectors)


def test_get_pos_list_abs_of_faces_by_connector_type():
    cube = Cube(1, np.array([m_size/2, m_size, m_size]),
                np.array([0, 0, 0, 1]))
    cube.set_cube_type('active_3_passive_3_konoji')
    face_list = cube.get_faces_by_connector_type('active')
    assert face_list == [0, 1, 4]
    pos_list_abs_of_faces_active \
        = cube.get_pos_list_abs_of_faces_by_connector_type('active')
    pos_list_abs_of_faces_active_expected = [
        np.array([m_size/2, m_size, m_size]) + np.array([cube.m_size/2, 0, 0]),
        np.array([m_size/2, m_size, m_size]) + np.array([0, cube.m_size/2, 0]),
        np.array([m_size/2, m_size, m_size]) +
        np.array([0, -cube.m_size/2, 0]),]
    print(pos_list_abs_of_faces_active)
    print(pos_list_abs_of_faces_active_expected)
    assert np.allclose(
        pos_list_abs_of_faces_active, pos_list_abs_of_faces_active_expected)


def test_rotate_x_90():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.rotate_x_90()
    expected_attitude = R.from_euler('x', 90, degrees=True).as_quat()
    assert np.allclose(cube.attitude, expected_attitude)


def test_rotate_x_minus_90():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.rotate_x_minus_90()
    expected_attitude = R.from_euler('x', -90, degrees=True).as_quat()
    assert np.allclose(cube.attitude, expected_attitude)


def test_rotate_y_90():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.rotate_y_90()
    expected_attitude = R.from_euler('y', 90, degrees=True).as_quat()
    assert np.allclose(cube.attitude, expected_attitude)


def test_rotate_y_minus_90():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.rotate_y_minus_90()
    expected_attitude = R.from_euler('y', -90, degrees=True).as_quat()
    assert np.allclose(cube.attitude, expected_attitude)


def test_rotate_z_180():
    cube = Cube(1, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
    cube.rotate_z_180()
    expected_attitude = R.from_euler('z', 180, degrees=True).as_quat()
    assert np.allclose(cube.attitude, expected_attitude)


if __name__ == "__main__":
    pytest.main()
