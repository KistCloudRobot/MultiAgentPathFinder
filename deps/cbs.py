from copy import deepcopy
from collections import defaultdict

import printInColor as pic
from planningTools import HighLevelNode, Constraints

class CBS(object):
    def __init__(self, environment):
        self.env = environment
        self.open_set = set()
        self.closed_set = set()
    
    def sum_of_cost(self, e):
        return e.cost

    def sum_of_move(self, e):
        move = 0
        for agent, path in e.solution.items():
            for i in range(1, len(path)):
                if path[i].location != path[i-1].location:
                    move += 1
        return move

    def search(self,print_ = True):
        start = HighLevelNode()
        # TODO: Initialize it in a better way
        start.constraint_dict = {}
        for agent in self.env.agent_dict.keys():
            start.constraint_dict[agent] = Constraints()

        #mvc = VertexConstraint(-1,Location(2,2))
        #start.constraint_dict["agent1"].vertex_constraints.add(mvc)

        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)

        self.open_set |= {start}

        # Hyojeong Edit
        expand = 0
        max_depth = 1

        while self.open_set:
            P = min(self.open_set)

            # Hyojeong Edit
            expand += 1
            depth = 0
            for k, v in P.constraint_dict.items():
                depth += len(v)
            max_depth = max(max_depth, depth)

            # Print
            # print("="*50)
            # print('cost1={}, cost2={}, depth/max_depth={}/{}, expand={}'.format(P.cost, self.sum_of_move(P), depth, max_depth, expand))

            # P.print("", expand)

            # Hyojeong Edit
            # P = min(self.open_set, key=lambda x: (self.sum_of_cost(x), self.sum_of_move(x)))
            self.open_set -= {P}
            self.closed_set |= {P}

            self.env.constraint_dict = P.constraint_dict
            conflict_dict = self.env.get_first_conflict(P.solution)
            # print(conflict_dict)
            if not conflict_dict:
                if(print_==True):
                    pic.printC("solution found",'green')

                return self.generate_plan(P.solution)

            constraint_dict = self.env.create_constraints_from_conflict(conflict_dict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(P)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                self.env.constraint_dict = new_node.constraint_dict
                new_node.solution = self.env.compute_solution()
                if not new_node.solution:
                    continue
                new_node.cost = self.env.compute_solution_cost(new_node.solution)

                # TODO: ending condition
                if new_node not in self.closed_set:
                    self.open_set |= {new_node}

        return {}

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan