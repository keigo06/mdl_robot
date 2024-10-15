# state.py
class State:
    def __init__(self, pre_assembly, post_assembly, action, cost):
        self.pre_assembly = pre_assembly  # Pre-transition assembly state
        self.post_assembly = post_assembly  # Post-transition assembly state
        self.action = action  # The action taken during the transition
        self.cost = cost  # The cost of the transition

    def get_cost(self):
        return self.cost
