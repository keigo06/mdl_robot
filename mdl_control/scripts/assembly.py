#!/usr/bin/env python3

""" Assembly class
This cube provides a class for creating and manipulating assemblies of cubes.
The Assembly class includes methods for creating some kinds of assemblies.
It also provides methods for managing network which show connection of cubes.
"""

from cube import Cube

import numpy as np
import numpy.typing as npt
import networkx as nx

import logging as log


logger = log.getLogger(__name__)
log.basicConfig(
    level=log.DEBUG,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[log.FileHandler(filename="logger.log")]
)


class Assembly:
    def __init__(self, num_cubes: int = 27):
        self.num_cubes = num_cubes
        self.cubes: dict[int, Cube] = {}
        self.asm_graph: nx.MultiDiGraph = nx.MultiDiGraph()

        self.actions = []
        self.unit_vectors: list[npt.NDArray] = [
            *np.eye(3),
            *-np.eye(3)
        ]
        cube = Cube(0, np.array([0.0, 0.0, 0.0]),
                    np.array([0.0, 0.0, 0.0, 1.0]))
        self.m_size: float = cube.m_size
        logger.debug("m_size: %f", self.m_size)

        self.robot_base_pos: npt.NDArray = np.array([0.0, 0.0, 0.0])
        # TODO: 今は一時的にactionを行った場所のpos

    def create_line_assembly(self):
        """Create an assembly of cubes arranged in a straight line."""
        for id in range(self.num_cubes):
            self.cubes[id] = Cube(
                id, np.array([id * 0.12, 0.0, 0.06]), np.array([0.0, 0.0, 0.0, 1.0]))
            self.cubes[id].set_cube_type('active_6_passive_0')

    def create_tower_assembly(self):
        """Create an assembly of cubes arranged in a tower."""
        for id in range(self.num_cubes):
            self.cubes[id] = Cube(id, np.array(
                [0.0, 0.0, 0.06 + id*self.m_size]), np.array([0.0, 0.0, 0.0, 1.0]))
            self.cubes[id].set_cube_type('active_6_passive_0')

    def create_grid_assembly(self):
        """Create an assembly of cubes arranged in a grid."""
        factors: list[int] = self.find_factors(self.num_cubes)
        grid_x, grid_y = min(factors, key=lambda f: abs(f[0] - f[1]))
        for i in range(grid_y):
            for j in range(grid_x):
                self.cubes[i * grid_x + j] = Cube(
                    cube_id=i * grid_x + j,
                    pos=np.array([i * 0.12, j * 0.12, 0.06]),
                    attitude=np.array([0.0, 0.0, 0.0, 1.0])
                )
                self.cubes[i * grid_x + j].set_cube_type(
                    'active_6_passive_0')

    def find_factors(self, n: int) -> list:
        """Return a list of factors of n.

        Args:
            n (int): The number to find factors of.

        Returns:
            list: A list of tuples, each containing a pair of factors.
        """
        factors: list[int] = []
        for i in range(1, int(np.sqrt(n)) + 1):
            if n % i == 0:
                factors.append((i, n // i))
        return factors

    def create_cubic_assembly(self):
        """Create an assembly of cubes arranged in a cube.
        for example, if num_cubes = 27, it creates a 3x3x3 assembly.
        """
        side_num_cubes: int = int(round(self.num_cubes ** (1/3)))
        for x in range(side_num_cubes):
            for y in range(side_num_cubes):
                for z in range(side_num_cubes):
                    cube_id: int = x * side_num_cubes * side_num_cubes + y * side_num_cubes + z
                    self.cubes[cube_id] = Cube(
                        cube_id=cube_id,
                        pos=np.array([
                            x * self.m_size, y * self.m_size, z * self.m_size]),
                        attitude=np.array([0.0, 0.0, 0.0, 1.0])
                    )
                    self.cubes[cube_id].set_cube_type('active_6_passive_0')

    def change_cubes_to_network(self):
        """create networkx graph from cubes"""
        pass

        self.asm_graph.clear()
        for cube in self.cubes.values():
            self.asm_graph.add_node(cube.cube_id, pos=cube.pos)

    def get_outermost_cube_ids(self, cube_ids: list[int] = None) -> list[int]:
        """Get the IDs of the outermost cubes in the assembly.

        Args:
            cube_ids (list[int], optional): List of cube IDs to check. Defaults to None.

        Returns:
            list: A list of IDs of the outermost cubes.
        """
        outermost_cube_ids: list[int] = []
        cube_positions: set[tuple] = set(tuple(cube.pos)
                                         for cube in self.cubes.values())
        if cube_ids is None:
            cube_ids = list(self.cubes.keys())
        for cube_id in cube_ids:
            cube = self.cubes[cube_id]
            is_outermost: bool = False
            for direction in self.unit_vectors:
                pos_near_asm_possible = tuple(
                    cube.pos + direction * self.m_size)
                if pos_near_asm_possible not in cube_positions:
                    is_outermost = True
                    break
            if is_outermost:
                outermost_cube_ids.append(cube_id)
        return outermost_cube_ids

    def get_reachable_module_ids(self) -> list[int]:
        """Get the IDs of the cubes that can be reached from the robot base.

        Returns:
            list: A list of IDs of the reachable cubes.
        """
        reachable_cube_ids: list[int] = []
        for cube in self.cubes.values():
            if np.linalg.norm(cube.pos - self.robot_base_pos) < 0.4:
                reachable_cube_ids.append(cube.cube_id)
        return reachable_cube_ids

    def get_near_asm_pos(self) -> list[npt.NDArray]:
        """Get positions near the assembly that are free.

        Returns:
            list: A list of positions near the assembly that are free.
        """
        pos_list_near_asm: list[npt.NDArray] = []

        for cube in self.cubes.values():
            for direction in self.unit_vectors:
                pos_near_asm_possible: npt.NDArray\
                    = cube.pos + direction*self.m_size
                if self.is_pos_free(pos_near_asm_possible):
                    pos_list_near_asm.append(pos_near_asm_possible)

        return pos_list_near_asm

    # def get_able_connect_dir(self, id, pos):
    #     """Get the directions in which a cube can connect at a given position.

    #     Args:
    #         id (int): The ID of the cube.
    #         pos (np.array): The position to check for possible connections.

    #     Returns:
    #         list: A list of directions in which the cube can connect.
    #     """
    #     dir_list = []

    #     for direction in self.unit_vector:
    #         near_pos = pos + direction
    #         near_cube = self.get_cube_by_pos(near_pos)

    #         if near_cube is not None:
    #             near_target_pos = near_cube.get_male_targets_pos_abs()

    #             for pos_possible_self in near_target_pos:
    #                 if np.array_equal(pos_possible_self, pos):
    #                     dir_list.append(direction)

    #     return dir_list

    def is_pos_free(self, pos: npt.NDArray) -> bool:
        """Check if a given position is free (not occupied by any cube).

        Args:
            pos (np.array): The position to check.

        Returns:
            bool: True if the position is free, False otherwise.
        """
        for cube in self.cubes.values():
            if np.array_equal(cube.pos, pos):
                return False
        return True

    def get_cube_id_by_pos(self, pos: npt.NDArray) -> int:
        """Get the cube id at a given position, if any.

        Args:
            pos (np.array): The position to check.

        Returns:
            Cube: The cube at the given position, or None if no cube is found.
        """
        for cube in self.cubes.values():
            if np.array_equal(cube.pos, pos):
                return cube.cube_id
        return None


if __name__ == '__main__':
    assembly = Assembly(num_cubes=27).create_cubic_assembly()
    print("Assembly main")
