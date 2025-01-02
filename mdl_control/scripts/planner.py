#!/usr/bin/env python3

from trasition_state import TransitionState
from assembly import Assembly

import numpy as np
import numpy.typing as npt
from typing import Optional, Any
from scipy.optimize import linear_sum_assignment
from heapq import heappush, heappop
import time
import csv

from logging import getLogger, basicConfig, FileHandler, StreamHandler, Formatter, DEBUG, INFO
logger = getLogger(('app_planner'))
logger.setLevel(DEBUG)

file_handler = FileHandler(filename='logger.log', mode='w')
file_handler.setLevel(DEBUG)
file_formatter = Formatter(
    '%(asctime)s-%(process)s-%(levelname)s-%(message)s')
file_handler.setFormatter(file_formatter)
logger.addHandler(file_handler)

console_handler = StreamHandler()
console_handler.setLevel(INFO)
console_formatter = Formatter(
    '%(asctime)s-%(process)s-%(levelname)s-%(message)s')
console_handler.setFormatter(console_formatter)
logger.addHandler(console_handler)


class Planner:
    def __init__(self) -> None:
        """ Planner Class
        """
        self.mode = "all_connector_active"
        self.asm_first = Assembly(num_cubes=27, mode=self.mode)
        self.asm_first.create_cubic_assembly()

        self.asm_purpose = Assembly(num_cubes=27, mode=self.mode)
        self.asm_purpose.create_tower_assembly()

        if len(self.asm_first.cubes) != len(self.asm_purpose.cubes):
            raise ValueError(
                "The number of cubes in the initial state and the goal state must be the same.")

        self.asm_first = self.asm_first

        # Heuristic type: manhattan or euclidean
        self.cost_calculation_type: Optional[str] = None
        self.cost_round_size: Optional[int] = None
        self.weight_heuristic: float = 10.0

    def get_heuristec(self, asm_current: Assembly) -> float:
        """
        Get the heuristic value of the current state.
        Calculated by matching Algorithm.
        """

        # target_positions: list[npt.NDArray] = [
        #     cube.pos for cube in self.asm_purpose.cubes.values()]

        if self.cost_round_size is None:
            raise ValueError("cost_round_size is not set.")
            return

        cost_matrix: np.ndarray = np.zeros(
            (len(asm_current.cubes), len(self.asm_purpose.cubes)))

        for i, cube_current in enumerate(asm_current.cubes.values()):
            for j, cube_purpose in enumerate(self.asm_purpose.cubes.values()):
                # TODO: Select cost
                if self.cost_calculation_type == "manhattan":
                    # manhattan length between the cubes, rounded to 4 decimal places
                    cost_matrix[i, j] = np.round(
                        np.sum(np.abs(cube_current.pos - cube_purpose.pos)), self.cost_round_size)
                elif self.cost_calculation_type == "euclidean":
                    # linear length between the cubes
                    cost_matrix[i, j] = np.round(
                        np.linalg.norm(cube_current.pos - cube_purpose.pos), self.cost_round_size)
                else:
                    raise ValueError(
                        "heuristec_type must be manhattan or euclidean.")
                if cost_matrix[i, j] != 0:
                    cost_matrix[i, j] += asm_current.action_start_cost

        # Hungarian Algorithm
        row, col = linear_sum_assignment(cost_matrix, maximize=False)

        total_cost: float = cost_matrix[row, col].sum()
        heuristec_cost: float = np.round(
            total_cost * self.weight_heuristic, self.cost_round_size)

        return heuristec_cost

    def is_same(self, asm_current: Assembly) -> bool:
        """
        Check if the current state is the same as the goal state.
        """
        heuristec_cost: float = self.get_heuristec(asm_current)
        if heuristec_cost == 0:
            return True
        else:
            return False

    def is_done(self, state: TransitionState) -> bool:
        """
        Check if this calculation is finished.
        """
        return self.is_same(state.asm)

    def is_explored(self, next_state: TransitionState, closed_dict: dict[float, TransitionState]) -> bool:
        # 探索済みかつコストが大きい場合:       True 探索しない
        # 探索済みかつコストが小さい場合:       False 探索する
        # 探索済みでない場合:                 False 探索する
        next_state_hash = next_state.__hash__()
        if next_state_hash in closed_dict:
            closed_node = closed_dict[next_state_hash]
            if closed_node.cost <= next_state.cost:
                return True
        return False

    def solver_a_star(self) -> tuple[bool, list[Optional[dict[Any, Any]]], int, list[int], float]:
        """
        A* algorithm.

        Returns:
            bool[bool]:                     True if the goal state is reached, False otherwise.
            actions_solved[list[dict]]:     A list of actions to reach the goal state.
            search_count[int]:              The number of searches.
            action_count_list[list[int]]:   The number of actions for each search.
            process_time[float]:            The time it took to reach the goal state.
        """

        initial_state = TransitionState(
            self.asm_first, pre_state=None, pre_action=None)
        self.cost_calculation_type = initial_state.cost_calculation_type
        self.cost_round_size = initial_state.cost_round_size
        initial_state.heuristic = self.get_heuristec(initial_state.asm)
        initial_state.recalculate_score()

        print(f"num_of_cubes: {len(self.asm_first.cubes)},\
          robot_reach_length_wout_cube: {self.asm_first.robot_reach_length_wout_cube},\
          robot_reach_length_with_cube: {self.asm_first.robot_reach_length_with_cube},\
          action_start_cost: {self.asm_first.action_start_cost},\
          cost_calculation_type: {self.cost_calculation_type},\
          weight_heuristic: {self.weight_heuristic}")
        logger.info(f"num_of_cubes: {len(self.asm_first.cubes)},\
          robot_reach_length_wout_cube: {self.asm_first.robot_reach_length_wout_cube},\
          robot_reach_length_with_cube: {self.asm_first.robot_reach_length_with_cube},\
          action_start_cost: {self.asm_first.action_start_cost},\
          cost_calculation_type: {self.cost_calculation_type},\
          weight_heuristic: {self.weight_heuristic}")

        logger.debug(f"asm_first.cubes: {len(self.asm_first.cubes)}")
        self.asm_first.print_asm()

        logger.debug(f"asm_purpose.cubes: {len(self.asm_purpose.cubes)}")
        self.asm_purpose.print_asm()

        # closed_list: list for saving the states that have been searched
        closed_dict: dict[float, TransitionState] = {
            initial_state.__hash__(): initial_state}

        # open_list: heap list for saving the states that have not been searched
        open_list: list[TransitionState] = []
        open_list_size_before: int = 0
        heappush(open_list, initial_state)
        logger.info("A star start")

        start = time.time()
        search_count: int = 0
        action_count_list: list[int] = []
        end_state: Optional[TransitionState] = None

        while end_state is None and open_list:
            search_count += 1
            # Get the state with the smallest score from the open list
            current_state: TransitionState = heappop(open_list)
            if search_count % 100 == 0:
                logger.info(
                    f"search count: {search_count}, num_of_action_count_list: {len(action_count_list)}")
                if logger.level == DEBUG:
                    current_state.asm.print_asm()
            if self.is_done(current_state):
                end_state = current_state
                logger.info("End state found")
                current_state.actions = []
            else:
                current_state.get_able_actions()
                logger.debug(
                    f"search count: {search_count}, closed_list: {len(closed_dict)}, open_list: {len(open_list)}, diff_open_list: {len(open_list) - open_list_size_before}")
                logger.debug(
                    f"state.cost: {current_state.cost}, state.heuristic: {current_state.heuristic}, state.score: {current_state.score}")
                logger.debug(
                    f"select from {len(current_state.actions)} actions")
            open_list_size_before = len(open_list)
            action_count: int = 0
            for action in current_state.actions:
                action_count += 1
                # reward is for RL
                next_asm, reward = current_state.do_action_if(action)
                next_state: TransitionState = TransitionState(
                    next_asm,
                    pre_state=current_state,
                    pre_action=action)
                next_state.heuristic = self.get_heuristec(next_state.asm)
                next_state.recalculate_score()

                if self.is_done(next_state):
                    end_state = next_state
                    logger.info("End state found")
                    break

                if not self.is_explored(next_state, closed_dict):
                    heappush(open_list, next_state)
                    closed_dict[next_state.__hash__()] = next_state

            action_count_list.append(action_count)

        process_time = time.time() - start

        if end_state is None:
            logger.info("End state not found")
            return False, [], search_count, action_count_list, process_time

        # Save the actions to reach the goal state
        current_state = end_state
        actions_solved: list[Optional[dict]] = []

        while current_state.pre_state is not None:
            actions_solved.append(current_state.pre_action)
            current_state = current_state.pre_state
        actions_solved.reverse()
        return True, actions_solved, search_count, action_count_list, process_time

    def show_actions_result(self, actions_solved: list[Optional[dict[str, Any]]], search_count: int, action_count_list: list[int], process_time: float) -> None:
        """
        Show the result of the actions.
        """
        if actions_solved is None:
            print("No actions found.")
            return
        else:
            print("Show actions result:")
            state = TransitionState(
                self.asm_first, pre_state=None, pre_action=None)
            for action in actions_solved:
                self.asm_first.print_asm()

                logger.debug(f"state.cost: {state.cost}")
                logger.debug(
                    f"state.heuristic: {self.get_heuristec(state.asm)}")
                logger.debug("↓")
                if action is not None:
                    logger.debug("action[cube_id]: %s, action[pos]: %s, action[cost]: %s",
                                 action["cube_id"], action["pos"], action["cost"])

                state.do_action(action)
                self.asm_first = state.asm
                print("↓")

            self.asm_first.print_asm()

            # actions_solved を logとして出力
            # {'cube_name': 'mdl_cube_02_body_link','pos': [0.24, 0.00, 0.06], 'attitude': [0.0, 0.0, 0.0, 1.0]},

            logger.info("actions_solved:")
            for action in actions_solved:
                if action is not None:
                    logger.info(
                        f"cube_name: {action['cube_id']}, pos: {action['pos']}, attitude: [0.0, 0.0, 0.0, 1.0]")

            print(
                f"hands={len(actions_solved)}, search_count={search_count},\
              action_count={sum(action_count_list)}, process_time={process_time} s")
            logger.info(
                f"hands={len(actions_solved)}, search_count={search_count},\
              action_count={sum(action_count_list)}, process_time={process_time} s")

    def export_actions_result(self, actions_solved: list[Optional[dict[str, Any]]], search_count: int, action_count_list: list[int], process_time: float) -> None:
        """
        Export the result of the actions as action_log.csv.
        """
        data = []
        for action in actions_solved:
            if action is not None:
                data.append([
                    action['cube_id'],
                    *action['pos'],
                    *[0.0, 0.0, 0.0, 1.0]
                ])
        np.savetxt('action_log.csv', data, delimiter=',',
                   header='cube_name,pos_x,pos_y,pos_z,attitude_x,attitude_y,attitude_z,attitude_w', comments='')


if __name__ == "__main__":
    planner = Planner()
    is_done, actions_solved, search_count, action_count_list, process_time = planner.solver_a_star()
    planner.show_actions_result(
        actions_solved, search_count, action_count_list, process_time)
    planner.export_actions_result(
        actions_solved, search_count, action_count_list, process_time)
