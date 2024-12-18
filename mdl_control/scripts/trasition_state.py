#!/usr/bin/env python3

import copy
import numpy as np

from logging import getLogger, basicConfig, DEBUG, INFO
# import coloredlogs
# coloredlogs.install(level='DEBUG')
logger = getLogger(('app'))
basicConfig(
    level=INFO, filename='logger.log',
    filemode='w', format='%(asctime)s-%(process)s-%(levelname)s-%(message)s'
)


class TransitionState:
    def __init__(self, asm, pre_asm=None, action=None):
        if asm is None:
            logger.info("asm is None")

        self.asm = asm
        self.actions = []
        self.pre_asm = pre_asm  # Pre-transition assembly state
        self.action = action  # The action taken during the transition

        if pre_asm is None:
            self.cost = 0
        else:
            self.cost = pre_asm.cost + action["cost"]
        self.heuristic = 0
        self.score = self.heuristic + self.cost

    def __hash__(self):
        return hash(tuple(sorted((id, cube.position)
                                 for id, cube in self.asm.cubes.items())))

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

    def get_able_eliminate_modules_id(self):
        return list(self.asm.cubes.keys())

    def get_able_add_modules_pos(self, asm_eliminate, id):
        pos_list_add = asm_eliminate.get_near_asm_pos()

        for pos in pos_list_add:
            asm_add = copy.deepcopy(asm_eliminate)
            asm_add.cubes[id] = copy.deepcopy(asm_add.cubes[id])
            asm_add.cubes[id].pos = pos

        return pos_list_add

    def get_able_actions(self):
        self.actions.clear()
        id_list_eliminate = self.get_able_eliminate_modules_id()
        for id in id_list_eliminate:
            asm_eliminate = self.eliminate_module(id)
            pos_list_add = self.get_able_add_modules_pos(asm_eliminate, id)

            for pos in pos_list_add:
                current_pos = self.asm.cubes[id].pos

                action = {
                    "id": id,
                    "pos": pos
                }

                asm_add = self.add_module(asm_eliminate, id, pos)

                self.evaluate_action(asm_add, action, current_pos)

                self.actions.append(action)

    def dvaluate_action(self, asm_add, action, current_pos):
        new_pos = action["pos"]
        action_pos_distance_cost = np.linalg.norm(new_pos - current_pos)

        action["cost"] = action_pos_distance_cost + asm_add.action_start_cost

        action["next_asm"] = copy.deepcopy(asm_add)

    def eliminate_module(self, id):
        asm_eliminate = copy.deepcopy(self.asm)

        asm_eliminate.cubes.pop(id)

        return asm_eliminate

    def add_module(self, asm_eliminate, id, pos):
        asm_add = copy.deepcopy(asm_eliminate)

        asm_add.cubes[id] = copy.deepcopy(asm_add.cubes[id])
        asm_add.cubes[id].pos = pos

    def do_action(self, action):

        asm_eliminate = self.eliminate_module(action["id"])
        asm_add = self.add_module(asm_eliminate, action["id"], action["pos"])

    def do_action_if(self, action):

        asm_eliminate = self.eliminate_module(action["id"])
        asm_add = self.add_module(asm_eliminate, action["id"], action["pos"])

        return asm_add
