#!/usr/bin/env python3

import copy
import numpy as np
import numpy.typing as npt
from typing import Optional, Any

from assembly import Assembly

from logging import getLogger, basicConfig, DEBUG, INFO

# logger = getLogger(('app_planner'))
logger = getLogger(('app_transition_state'))
logger.setLevel(INFO)
# basicConfig(
#     level=INFO, filename='logger.log',
#     filemode='w', format='%(asctime)s-%(process)s-%(levelname)s-%(message)s'
# )


class TransitionState:
    def __init__(self, asm_current: Assembly, pre_state: Optional[Any] = None, pre_action: Optional[dict] = None) -> None:
        # TODO: Set pre of robot
        if asm_current is None:
            logger.info("asm is None")

        self.asm: Assembly = asm_current
        self.actions: list[dict] = []
        self.pre_state: Optional[TransitionState] = pre_state
        self.pre_action: Optional[dict] = pre_action

        # score/heuristic/cost is better if it is smaller
        # because it is used in priority queue
        self.cost: float = 0
        self.cost_calculation_type: str = "manhattan"
        self.cost_round_size: int = 4
        if pre_state is None or pre_action is None:
            self.cost = 0
        else:
            self.cost = round(
                pre_state.cost + pre_action["cost"], self.cost_round_size)
        self.heuristic: float = 0
        # f = g + h
        # g: cost, h: heuristic, f: score
        self.score: float = np.round(
            self.heuristic + self.cost, self.cost_round_size)

    def recalculate_score(self):
        self.score = np.round(self.heuristic + self.cost, self.cost_round_size)

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
        logger.debug(f"id_list_reachable_wout_cube: {id_list_reachable}")
        logger.debug(f"num_of_id_list_reachable: {len(id_list_reachable)}")
        id_list_outmost: list[int] = self.asm.get_outermost_cube_ids(
            id_list_reachable)
        logger.debug(f"id_list_outmost_in_reachable: {id_list_outmost}")
        logger.debug(f"num_of_id_list_outmost: {len(id_list_outmost)}")

        id_list_outmost_copy = id_list_outmost[:]
        for cube_id in id_list_outmost_copy:
            asm_eliminate = copy.deepcopy(self.asm)
            asm_eliminate.cubes.pop(cube_id)
            asm_eliminate.update_networkx_remove_node(cube_id)
            if not asm_eliminate.check_connectivity():
                id_list_outmost.remove(cube_id)

        # TODO: Check robot IK
        return id_list_outmost

    def get_able_add_modules_pos(self, asm_eliminate: Assembly, id: int) -> list[npt.NDArray]:
        id_list_reachable: list[int] \
            = asm_eliminate.get_reachable_module_ids_with_cube()
        logger.debug(f"id_list_reachable_with_cube: {id_list_reachable}")
        logger.debug(f"num_of_id_list_reachable: {len(id_list_reachable)}")
        pos_list_add: list[npt.NDArray] = asm_eliminate.get_near_asm_pos(
            id_list_reachable)
        logger.debug(f"pos_list_add_in_reachable: {pos_list_add}")
        logger.debug(f"num_of_pos_list_add: {len(pos_list_add)}")
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

                if np.round(np.linalg.norm(pos - current_pos), self.cost_round_size) == 0:
                    continue
                asm_add: Assembly = self.add_module(asm_eliminate, id, pos)
                action: dict = {
                    "cube_id": id,
                    "pos": pos
                }
                self.evaluate_action(asm_add, action, current_pos)
                logger.debug(
                    f"cube_id: {id}, to_pos: {pos}, action_cost: {action['cost']}")

                self.actions.append(action)

    def evaluate_action(self, asm_add: Assembly, action: dict, current_pos: npt.NDArray):
        new_pos = action["pos"]
        if self.cost_calculation_type == "manhattan":
            action_pos_distance_cost = np.sum(np.abs(new_pos - current_pos))
        elif self.cost_calculation_type == "euclidean":
            action_pos_distance_cost = np.linalg.norm(new_pos - current_pos)
        else:
            logger.error("cost_calculation_type is invalid")
            raise ValueError("cost_calculation_type is invalid")
        action["cost"] = np.round(
            action_pos_distance_cost + asm_add.action_start_cost, self.cost_round_size)
        # action["next_asm"] = copy.deepcopy(asm_add)

    def eliminate_module(self, id: int) -> Assembly:
        asm_eliminate: Assembly = copy.deepcopy(self.asm)
        asm_eliminate.cubes.pop(id)
        asm_eliminate.update_networkx_remove_node(id)
        asm_eliminate.robot_base_pos = self.asm.cubes[id].pos
        return asm_eliminate

    def add_module(self, asm_eliminate: Assembly, id: int, pos: npt.NDArray):
        asm_add = copy.deepcopy(asm_eliminate)
        asm_add.cubes[id] = copy.deepcopy(self.asm.cubes[id])
        asm_add.cubes[id].pos = pos
        asm_add.update_networkx_add_node(id, pos)
        asm_add.robot_base_pos = pos
        return asm_add

    def do_action(self, action):
        asm_eliminate = self.eliminate_module(action["cube_id"])
        asm_add = self.add_module(
            asm_eliminate, action["cube_id"], action["pos"])

    def do_action_if(self, action: dict) -> tuple[Assembly, float]:

        asm_eliminate = self.eliminate_module(action["cube_id"])
        asm_add = self.add_module(
            asm_eliminate, action["cube_id"], action["pos"])
        reward = 0.0
        return asm_add, reward


if __name__ == "__main__":
    asm_first = Assembly(num_cubes=27, mode="all_connector_active")
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
