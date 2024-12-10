from cube import Cube  # Cubeクラスをインポート

import math


class Assembly:
    def __init__(self, num_cubes=5):
        # cubesは辞書形式で登録
        self.num_cubes = num_cubes
        self.cubes = {}

    # def get_cube(self, cube_id):
    #     """
    #     指定されたIDのCubeオブジェクトを返す
    #     :param cube_id: キューブのID
    #     :return: Cubeオブジェクト
    #     """
    #     for cube in self.cubes:
    #         if cube.cube_id == cube_id:
    #             return cube
    #     return None

    # def add_cube(self, position, euler_angles):
    #     """
    #     新しいキューブをアセンブリに追加する
    #     :param position: キューブの初期位置
    #     :param euler_angles: キューブの初期姿勢
    #     """
    #     new_cube = Cube(len(self.cubes), position, euler_angles)
    #     self.cubes.append(new_cube)

    # def connect_cubes(self, cube1_id, cube2_id, face1, face2):
    #     """
    #     2つのキューブを指定された面で結合する
    #     :param cube1_id: キューブ1のID
    #     :param cube2_id: キューブ2のID
    #     :param face1: キューブ1の結合面
    #     :param face2: キューブ2の結合面
    #     """
    #     cube1 = self.get_cube(cube1_id)
    #     cube2 = self.get_cube(cube2_id)

    #     if cube1 and cube2:
    #         cube1.connect(face1, cube2, face2)
    #         cube2.connect(face2, cube1, face1)

    # def get_assembly_state(self):
    #     """
    #     アセンブリ全体の状態を返す（各キューブの状態を取得）
    #     :return: アセンブリ全体のキューブの状態
    #     """
    #     return [cube.get_state() for cube in self.cubes]

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
                    position=[i * 0.12, j * 0.12, 0.06],
                    euler_angles=[0.0, 0.0, 0.0]
                )
