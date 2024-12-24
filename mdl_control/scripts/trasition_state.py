#!/usr/bin/env python3

import copy
import numpy as np
import numpy.typing as npt
from assembly import Assembly

from logging import getLogger, basicConfig, DEBUG, INFO
# import coloredlogs
# coloredlogs.install(level='DEBUG')
logger = getLogger(('app'))
basicConfig(
    level=INFO, filename='logger.log',
    filemode='w', format='%(asctime)s-%(process)s-%(levelname)s-%(message)s'
)


class TransitionState:
    def __init__(self, asm_current: Assembly, pre_state=None, pre_action: dict = None):
        # TODO: Set pre of robot
        if asm_current is None:
            logger.info("asm is None")

        self.asm: Assembly = asm_current
        self.actions: list[dict] = []
        self.pre_state: TransitionState = pre_state
        self.pre_action: dict = pre_action

        if pre_state is None:
            self.cost: float = 0
        else:
            self.cost: float = pre_state.cost + pre_action["cost"]
        self.heuristic: float = 0
        self.score: float = self.heuristic + self.cost

    def recalculate_score(self):
        self.score = self.heuristic + self.cost

    def __hash__(self):
        """Generate hash value calculated by the position of cubes and the position of the robot base.

        Returns:
            int: hash value
        """
        # TODO: 複数種類のCubeやConnectorを扱う場合のhash計算
        # 種類ごとに並べればいいのでは？
        # attitudeはposに付随させる
        cube_pos_list = tuple(sorted(tuple(cube.pos)
                                     for cube in self.asm.cubes.values()))
        robot_base_pos = tuple(self.asm.robot_base_pos)
        return hash((cube_pos_list, robot_base_pos))

    def __le__(self, other):
        # less than or equal to, <=
        return self.score <= other.score

    def __ge__(self, other):
        # greater than or equal to, >=
        return self.score >= other.score

    def __lt__(self, other):
        # less than, <
        return self.score < other.score

    def __gt__(self, other):
        # greater than, >
        return self.score > other.score

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()

    def get_able_eliminate_modules_id(self) -> list[int]:
        id_list_reachable: list[int] = self.asm.get_reachable_module_ids_wout_cube(
        )
        id_list_outmost: list[int] = self.asm.get_outermost_cube_ids(
            id_list_reachable)
        # TODO: Check connectivity
        # TODO: Check robot IK
        return id_list_outmost

    def get_able_add_modules_pos(self, asm_eliminate: Assembly, id: int) -> list[npt.NDArray]:
        id_list_reachable: list[npt.NDArray] \
            = asm_eliminate.get_reachable_module_ids_with_cube()
        pos_list_add: list[npt.NDArray] = asm_eliminate.get_near_asm_pos(
            id_list_reachable)
        # TODO: Check connectivity
        # TODO: Check connector direction and type by using id
        # TODO: Check robot IK
        return pos_list_add

    def get_able_actions(self) -> None:
        self.actions.clear()
        id_list_eliminate: list[int] = self.get_able_eliminate_modules_id()
        for id in id_list_eliminate:
            asm_eliminate: Assembly = self.eliminate_module(id)
            pos_list_add: list[npt.NDArray] = self.get_able_add_modules_pos(
                asm_eliminate, id)
            for pos in pos_list_add:
                current_pos: npt.NDArray = self.asm.cubes[id].pos

                if np.linalg.norm(pos - current_pos) == 0:
                    continue
                asm_add: Assembly = self.add_module(asm_eliminate, id, pos)
                action: dict = {
                    "id": id,
                    "pos": pos
                }
                self.evaluate_action(asm_add, action, current_pos)

                self.actions.append(action)

    def evaluate_action(self, asm_add: Assembly, action: dict, current_pos: npt.NDArray):
        new_pos = action["pos"]
        action_pos_distance_cost = np.linalg.norm(new_pos - current_pos)

        action["cost"] = action_pos_distance_cost + asm_add.action_start_cost
        action["next_asm"] = copy.deepcopy(asm_add)

    def eliminate_module(self, id):
        asm_eliminate = copy.deepcopy(self.asm)
        asm_eliminate.cubes.pop(id)
        asm_eliminate.robot_base_pos = self.asm.cubes[id].pos
        return asm_eliminate

    def add_module(self, asm_eliminate: Assembly, id: int, pos: npt.NDArray):
        asm_add = copy.deepcopy(asm_eliminate)
        asm_add.cubes[id] = copy.deepcopy(self.asm.cubes[id])
        asm_add.cubes[id].pos = pos
        asm_add.robot_base_pos = pos
        return asm_add

    def do_action(self, action):
        asm_eliminate = self.eliminate_module(action["id"])
        asm_add = self.add_module(asm_eliminate, action["id"], action["pos"])

    def do_action_if(self, action: dict) -> tuple[Assembly, float]:

        asm_eliminate = self.eliminate_module(action["id"])
        asm_add = self.add_module(asm_eliminate, action["id"], action["pos"])
        reward = 0.0
        return asm_add, reward


if __name__ == "__main__":
    asm_first = Assembly(num_cubes=27)
    asm_first.create_cubic_assembly()
    state = TransitionState(asm_first)
    id_list_eliminate_modules = state.get_able_eliminate_modules_id()
    print(id_list_eliminate_modules)
    id_selected = id_list_eliminate_modules[0]
    asm_eliminate = state.eliminate_module(id_selected)
    print(asm_eliminate.cubes[1].pos)
    pos_list_add: list[npt.NDArray] = state.get_able_add_modules_pos(
        asm_eliminate, id_selected)
    print(pos_list_add)
    print(len(pos_list_add))
