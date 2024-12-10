from cube import Cube

import numpy as np


class Assembly:
    def __init__(self, num_cubes=5):
        # cubesは辞書形式で登録
        self.num_cubes = num_cubes
        self.cubes = {}
        self.actions = []
        self.unit_vector = [
            [1, 0, 0],
            [-1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
            [0, 0, 1],
            [0, 0, -1]
        ]

    def create_line_assembly(self):
        """ キューブを直線状に配置するアセンブリを作成 """
        for id in range(self.num_cubes):
            self.cubes[id] = Cube(id, [id * 0.12, 0.0, 0.06], [0.0, 0.0, 0.0])

    def create_tower_assembly(self):
        """ キューブをタワー状に配置するアセンブリを作成 """
        for id in range(self.num_cubes):
            self.cubes[id] = Cube(
                id, [0.0, 0.0, 0.06 + id*0.12], [0.0, 0.0, 0.0])

    def find_factors(n):
        """Return a list of factors of n"""
        factors = []
        for i in range(1, int(math.sqrt(n)) + 1):
            if n % i == 0:
                factors.append((i, n // i))
        return factors

    def create_grid_assembly(self):
        """ キューブをグリッド状に配置するアセンブリを作成 """
        factors = self.find_factors(self.num_cubes)
        grid_x, grid_y = min(factors, key=lambda f: abs(f[0] - f[1]))
        for i in range(grid_y):
            for j in range(grid_x):
                self.cubes[i * grid_x + j] = Cube(
                    cube_id=i * grid_x + j,
                    pos=[i * 0.12, j * 0.12, 0.06],
                    euler_angles=[0.0, 0.0, 0.0]
                )

    def get_outermost_module_ids(self):
        outermost_module_ids = []

        module_positions = set(tuple(cube.pos)
                               for cube in self.cubes.values())

        for cube_id, cube in self.cubes.items():
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
        pos_list_near_asm = []

        for cube in self.cubes.values():
            for direction in self.directions:
                pos_near_asm_possible = cube.pos + direction
                if self.is_pos_free(pos_near_asm_possible):
                    pos_list_near_asm.append(pos_near_asm_possible)

        return pos_list_near_asm
