#!/usr/bin/env python3

from trasition_state import TransitionState
from assembly import Assembly

import numpy as np
import numpy.typing as npt

from scipy.optimize import linear_sum_assignment


class Planner:
    def __init__(self):
        self.asm_first: Assembly = Assembly(num_cubes=27)
        self.asm_first.create_cubic_assembly()

        self.asm_purpose: Assembly = Assembly(num_cubes=27)
        self.asm_purpose.create_tower_assembly()

        if len(self.asm_first.cubes) != len(self.asm_purpose.cubes):
            raise ValueError(
                "The number of cubes in the initial state and the goal state must be the same.")

        self.asm: Assembly = self.asm_first

    def get_heuristec(self, asm_current: Assembly) -> float:
        """
        Get the heuristic value of the current state.
        Calculated by matching Algorithm.
        """

        # target_positions: list[npt.NDArray] = [
        #     cube.pos for cube in self.asm_purpose.cubes.values()]

        cost_matrix: np.ndarray = np.zeros(
            (len(asm_current.cubes), len(self.asm_purpose.cubes)))

        for i, cube_current in enumerate(asm_current.cubes.values()):
            for j, cube_purpose in enumerate(self.asm_purpose.cubes.values()):
                cost_matrix[i, j] = np.linalg.norm(
                    cube_current.pos - cube_purpose.pos)
                if cost_matrix[i, j] != 0:
                    cost_matrix[i, j] += asm_current.action_start_cost

        # Hungarian Algorithm
        row, col = linear_sum_assignment(cost_matrix, maximize=False)

        total_cost = cost_matrix[row, col].sum()

        return total_cost
