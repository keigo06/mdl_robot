#!/usr/bin/env python3

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
        return hash(tuple(sorted((id, tuple(cube.pos), cube.rot)
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

    def get_able_actions(self):
        self.actions.clear()
        id_list_eliminate = self.asm.cubes.keys()
