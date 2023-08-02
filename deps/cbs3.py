from copy import deepcopy
from collections import defaultdict

import printInColor as pic
from planningTools import HighLevelNode, Constraints

from log.setup import logger

class CBS3(object):
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

    def to_env_constraint(self, node_const):
        # node_const: <dict> (ai, aj) : Constraints
        # env_const: <dict> ai: Constraints
        env_const = defaultdict(Constraints)
        for agents, constraints in node_const.items():
            env_const[agents[0]].add_constraint(constraints)        
        return env_const

    def search(self,print_=True):
        start = HighLevelNode()
        start.constraint_dict = defaultdict(Constraints)
        start.solution = self.env.compute_solution()
        if not start.solution:
            return {}
        start.cost = self.env.compute_solution_cost(start.solution)
        start.name = 0

        self.open_set |= {start}

        # Hyojeong Edit
        expand = 0
        max_depth = 1

        ##################################
        # selected agent pair
        # current_overlap: <tuple> agent pair (ai, aj)
        # goal_overlap_agents: <list> list of agent pair (ai, aj)
        current_overlap = None
        goal_overlap_agents = None
        ##################################

        while self.open_set:
            P = min(self.open_set)
            
            # Hyojeong Edit
            expand += 1
            depth = 0
            for k, v in P.constraint_dict.items():
                depth += len(v)
            max_depth = max(max_depth, depth)

            # Print
            # if print_:
            #     print("="*50)
            #     print('cost={}, depth/max_depth={}/{}, expand={}'.format(P.cost, depth, max_depth, expand))

            # if print_:
            #     P.print("")
            
            self.open_set -= {P}
            self.closed_set |= {P}
            
            ##################################
            # update goal overlapped agent pairs if current overlap doesn't exist
            # if (expand == 1) or (current_overlap is None):
            #   goal_overlap_agents = self.env.get_goal_overlap_agents(P.solution)
            # print(goal_overlap_agents)
            ##################################

            self.env.constraint_dict = self.to_env_constraint(P.constraint_dict)
            # conflict = self.env.get_first_conflict(P.solution)
            ##################################
            # choose conflict with new condition
            # goal_overlap_agents = self.env.get_goal_overlap_agents(P.solution)
            conflict, is_overlap = self.env.get_conflict_cbs3(P.solution, P.parent_overlap)
            if is_overlap:
                current_overlap = (conflict.agent_1, conflict.agent_2)
            else:
                current_overlap = None
            ##################################
            if not conflict:
                # print("solution_cost-initial_cost={}".format(P.cost-start.cost))
                if(print_==True):
                    logger.info("solution found")
                return self.generate_plan(P.solution)

            ##################################
            # create new constraint from conflict
            constraint_dict = self.env.create_constraints_from_conflict_cbs3(conflict, is_overlap)
            ##################################

            ##################################
            # add condition for adding constraint
            for (agent_1, agent_2) in constraint_dict.keys():
                new_node = deepcopy(P)
                # add new constraint only if constraint doesn't exist in reversed agent pair
                if (agent_2, agent_1) not in new_node.constraint_dict.keys():
                    if is_overlap:
                        new_node.constraint_dict[(agent_1, agent_2)].add_constraint(constraint_dict[(agent_1, agent_2)])
                    else:
                        new_node.constraint_dict[(agent_1, -1)].add_constraint(constraint_dict[(agent_1, -1)])
                    self.env.constraint_dict = self.to_env_constraint(new_node.constraint_dict)
                    new_node.solution = self.env.compute_solution()
                    if not new_node.solution:
                        continue
                    new_node.cost = self.env.compute_solution_cost(new_node.solution)
                    new_node.name = expand

                    new_node.parent_overlap = current_overlap

                    # if print_:
                    #     new_node.print("\t")

                    # TODO: ending condition
                    if new_node not in self.closed_set:
                        self.open_set |= {new_node}
            ##################################

        return {}

    def generate_plan(self, solution):
        plan = {}
        for agent, path in solution.items():
            path_dict_list = [{'t':state.time, 'x':state.location.x, 'y':state.location.y} for state in path]
            plan[agent] = path_dict_list
        return plan