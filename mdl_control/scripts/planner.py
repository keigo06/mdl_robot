#!/usr/bin/env python3

from trasition_state import TransitionState
from assembly import Assembly

import numpy as np
import numpy.typing as npt
from scipy.optimize import linear_sum_assignment
from heapq import heappush, heappop
import time

from logging import getLogger, basicConfig, DEBUG, INFO
logger = getLogger(('app'))
basicConfig(
    level=DEBUG, filename='logger.log',
    filemode='w', format='%(asctime)s-%(process)s-%(levelname)s-%(message)s'
)


class Planner:
    def __init__(self):
        self.asm_first: Assembly = Assembly(num_cubes=27)
        self.asm_first.create_cubic_assembly()

        self.asm_purpose: Assembly = Assembly(num_cubes=27)
        self.asm_purpose.create_tower_assembly()

        if len(self.asm_first.cubes) != len(self.asm_purpose.cubes):
            raise ValueError(
                "The number of cubes in the initial state and the goal state must be the same.")

        self.asm_current: Assembly = self.asm_first

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

        total_cost: float = cost_matrix[row, col].sum()

        return total_cost

    def is_same(self, asm_current: Assembly) -> bool:
        """
        Check if the current state is the same as the goal state.
        """
        heuristec_cost: float = self.get_heuristec(asm_current)
        if heuristec_cost == 0:
            return True
        else:
            return False

    def is_done(self, asm_current) -> bool:
        """
        Check if this calculation is finished.
        """
        return self.is_same(asm_current)

    def solver_a_star(self) -> tuple[bool, list[dict], int, list[int], float]:
        """
        A* algorithm.

        Returns:
            bool[bool]:                     True if the goal state is reached, False otherwise.
            actions_solved[list[dict]]:     A list of actions to reach the goal state.
            search_count[int]:              The number of searches.
            action_count_list[list[int]]:   The number of actions for each search.
            process_time[float]:            The time it took to reach the goal state.
        """

        logger.info("solver_a_star start")
        state = TransitionState(
            self.asm_current, pre_state=None, pre_action=None)
        state.heuristic = self.get_heuristec(state.asm)
        state.recalculate_score()
        logger.debug(f"Initial state: {state}")

        # closed_list: list for saving the states that have been searched
        closed_list: dict[int, TransitionState] = {state.__hash__(): state}

        # open_list: heap list for saving the states that have not been searched
        open_list: list[TransitionState] = []
        heappush(open_list, state)
        logger.debug("a star start")

        start = time.time()
        search_count: int = 0
        action_count_list: list[int] = []
        end_state: TransitionState = None

        while end_state is None and open_list:
            search_count += 1
            if search_count % 100 == 0:
                logger.info("search count: {search_count}")

            # Get the state with the smallest score from the open list
            state = heappop(open_list)
            state.get_able_actions()
            logger.debug(f"select from {len(state.actions)} actions")
            action_count: int = 0
            for action in state.actions:
                action_count += 1
                # if action is the last action of state.actions
                if action_count == len(state.actions):
                    logger.info(
                        f"search count: {search_count}, action_count: {action_count}")
                # reward is for RL
                next_asm, reward = state.do_action_if(action)
                next_state: TransitionState = TransitionState(
                    next_asm,
                    pre_state=state,
                    pre_action=action)
                next_state.heuristic = self.get_heuristec(next_state.asm)
                next_state.recalculate_score()

                if self.is_done(next_state):
                    end_state = next_state
                    logger.info("End state found")
                    break

                # if new state is already in closed_list
                # and the new state's score is higher than the closed state's score
                # then skip the new state
                if next_state in closed_list and next_state >= closed_list[next_state.__hash__]:
                    continue

                closed_list[next_state.__hash__] = next_state
                heappush(open_list, next_state)

            action_count_list.append(action_count)
            logger.debug(f"sum_of_action_count_list: {sum(action_count_list)}")

        process_time = time.time() - start

        if end_state is None:
            logger.info("End state not found")
            return False, [], search_count, process_time

        # Save the actions to reach the goal state
        state = end_state
        actions_solved: list[dict] = []

        while state.pre_state is not None:
            actions_solved.append(state.pre_action)
            state = state.pre_state
        actions_solved.reverse()
        return tuple[True, actions_solved, search_count, action_count_list, process_time]

    def show_actions_result(self, actions_solved: list[dict], search_count: int, action_count_list: list[int], process_time: float) -> None:
        """
        Show the result of the actions.
        """
        print("Show actions result:")
        state = TransitionState(
            self.asm_current, pre_asm=None, pre_action=None)
        for action in actions_solved:
            self.asm_current.print_asm()

            # for debug
            logger.debug(f"state.cost: {state.cost}")
            logger.debug(f"state.heuristic: {self.get_heuristec(state.asm)}")
            logger.debug("↓")
            logger.debug(
                f"out_most_cube_ids: {state.asm.get_outermost_cube_ids()}")
            logger.debug(
                f"eliminate_cube_ids: {state.get_able_eliminate_modules_id()}")
            state.get_able_actions()
            able_ids = []
            for able_action in state.actions:
                logger.debug("cube id:", able_action["id"],
                             "to pos:", able_action["pos"],
                             "cost:", able_action["cost"],)
                able_ids.append(able_action["id"])
            logger.debug("able_action_ids:", able_ids)
            logger.debug("action[cost]:", action["cost"])

            state.do_action(action)
            self.asm_current = state.asm
            print("↓")

        self.asm_current.print_asm()

        print(
            f"hands={len(actions_solved)}, search_count={search_count},\
            action_count={sum(action_count_list)}, process_time={process_time} s")


if __name__ == "__main__":
    planner = Planner()
    is_done, actions_solved, search_count, action_count_list, process_time = planner.solver_a_star()
    planner.show_actions_result(
        actions_solved, search_count, action_count_list, process_time)
