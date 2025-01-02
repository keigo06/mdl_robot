import time
from logging import getLogger, basicConfig, DEBUG, INFO

logger = getLogger(('app_planner'))
logger.setLevel(INFO)


class CustomLogger:
    def __init__(self) -> None:
        self.times: dict[str, float] = {
            "planning_all": 0.0,
            "pop_from_open_list": 0.0,
            "push_to_open_list": 0.0,
            "push_to_closed_list": 0.0,
            "action_generation": 0.0,
            "check_connectivity": 0.0,
            "robot_IK": 0.0,
            "calculate_heuristic": 0.0,
        }
        self.start_times: dict = {}
        self.memory_usage: dict[str, float] = {
            "open_list": 0.0,
            "closed_list": 0.0,
        }

    def start(self, process_name: str):
        self.start_times[process_name] = time.time()

    def stop(self, process_name: str):
        if process_name in self.start_times:
            elapsed_time = time.time() - self.start_times[process_name]
            self.times[process_name] += elapsed_time
            del self.start_times[process_name]

    def report(self):
        for process, total_time in self.times.items():
            print(f"{process}: {total_time:.6f} seconds")
            logger.info(f"{process}: {total_time:.6f} seconds")
