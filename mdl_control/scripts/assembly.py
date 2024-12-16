#!/usr/bin/env python3

""" Assembly class
This module provides a class for creating and manipulating assemblies of cubes.
The Assembly class includes methods for creating some kinds of assemblies.
It also provides methods for managing network which show the connection of modules.
"""

from cube import Cube

import numpy as np


class Assembly:
    def __init__(self, num_cubes=6):
        self.num_cubes = num_cubes
        self.modules = {}
        self.actions = []
        self.unit_vector = np.array([
            [1, 0, 0],
            [-1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
            [0, 0, 1],
            [0, 0, -1]
        ])

    def create_line_assembly(self):
        """Create an assembly of cubes arranged in a straight line."""
        for id in range(self.num_cubes):
            self.modules[id] = Cube(id, np.array(
                [id * 0.12, 0.0, 0.06]), np.array([0.0, 0.0, 0.0, 1.0]))
            self.modules[id].set_module_type('active_6_passive_0')

    def create_tower_assembly(self):
        """Create an assembly of cubes arranged in a tower."""
        for id in range(self.num_cubes):
            self.modules[id] = Cube(id, np.array(
                [0.0, 0.0, 0.06 + id*0.12]), np.array([0.0, 0.0, 0.0, 1.0]))
            self.modules[id].set_module_type('active_6_passive_0')

    def create_grid_assembly(self):
        """Create an assembly of cubes arranged in a grid."""
        factors = self.find_factors(self.num_cubes)
        grid_x, grid_y = min(factors, key=lambda f: abs(f[0] - f[1]))
        for i in range(grid_y):
            for j in range(grid_x):
                self.modules[i * grid_x + j] = Cube(
                    cube_id=i * grid_x + j,
                    pos=np.array([i * 0.12, j * 0.12, 0.06]),
                    attitude=np.array([0.0, 0.0, 0.0, 1.0])
                )
                self.modules[i * grid_x + j].set_module_type(
                    'active_6_passive_0')

    def find_factors(selfg, n):
        """Return a list of factors of n.

        Args:
            n (int): The number to find factors of.

        Returns:
            list: A list of tuples, each containing a pair of factors.
        """
        factors = []
        for i in range(1, int(np.sqrt(n)) + 1):
            if n % i == 0:
                factors.append((i, n // i))
        return factors

    def get_outermost_module_ids(self):
        """Get the IDs of the outermost modules in the assembly.

        Returns:
            list: A list of IDs of the outermost modules.
        """
        outermost_module_ids = []
        module_positions = set(tuple(module.pos)
                               for module in self.modules.values())
        for cube_id, cube in self.modules.items():
            is_outermost = False
            for direction in self.unit_vector:
                pos_near_asm_possible = tuple(cube.pos + direction)
                if pos_near_asm_possible not in module_positions:
                    is_outermost = True
                    break
            if is_outermost:
                outermost_module_ids.append(cube_id)
        return outermost_module_ids

    def get_near_asm_pos(self):
        """Get positions near the assembly that are free.

        Returns:
            list: A list of positions near the assembly that are free.
        """
        pos_list_near_asm = []

        for cube in self.modules.values():
            for direction in self.unit_vector:
                pos_near_asm_possible = cube.pos + direction
                if self.is_pos_free(pos_near_asm_possible):
                    pos_list_near_asm.append(pos_near_asm_possible)

        return pos_list_near_asm

    # def get_able_connect_dir(self, id, pos):
    #     """Get the directions in which a module can connect at a given position.

    #     Args:
    #         id (int): The ID of the module.
    #         pos (np.array): The position to check for possible connections.

    #     Returns:
    #         list: A list of directions in which the module can connect.
    #     """
    #     dir_list = []

    #     for direction in self.unit_vector:
    #         near_pos = pos + direction
    #         near_module = self.get_module_by_pos(near_pos)

    #         if near_module is not None:
    #             near_target_pos = near_module.get_male_targets_pos_abs()

    #             for pos_possible_self in near_target_pos:
    #                 if np.array_equal(pos_possible_self, pos):
    #                     dir_list.append(direction)

    #     return dir_list

    def is_pos_free(self, pos):
        """Check if a given position is free (not occupied by any module).

        Args:
            pos (np.array): The position to check.

        Returns:
            bool: True if the position is free, False otherwise.
        """
        for module in self.modules.values():
            if np.array_equal(module.pos, pos):
                return False
        return True

    def get_module_by_pos(self, pos):
        """Get the module at a given position, if any.

        Args:
            pos (np.array): The position to check.

        Returns:
            Cube: The module at the given position, or None if no module is found.
        """
        for module in self.modules.values():
            if np.array_equal(module.pos, pos):
                return module
        return None