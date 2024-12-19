#!/usr/bin/env python3

from scripts.assembly import Assembly
import pytest

import numpy as np


def test_create_cubic_assembly():
    assembly = Assembly(num_cubes=27)
    assembly.create_cubic_assembly()
    assert len(assembly.cubes) == 27
    positions = [tuple(cube.pos) for cube in assembly.cubes.values()]
    expected_positions = [(
        x*0.12, y*0.12, z * 0.12) for x in range(3) for y in range(3) for z in range(3)]
    assert set(positions) == set(expected_positions)
    ids = [cube.cube_id for cube in assembly.cubes.values()]
    assert set(ids) == set(range(27))


def test_update_graph():
    """ 
    """
    assembly = Assembly(num_cubes=27)
    assembly.create_cubic_assembly()
    assembly.update_graph()

# def test_get_outermost_cube_ids_cubic_assembly():
    # assembly = Assembly(num_cubes=27)
    # assembly.create_cubic_assembly()
    # outermost_ids = assembly.get_outermost_cube_ids()
    # expected_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    #                 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26]
    # assert set(outermost_ids) == set(expected_ids)


if __name__ == "__main__":
    pytest.main()
