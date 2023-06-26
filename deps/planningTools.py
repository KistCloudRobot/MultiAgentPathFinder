import sys
#sys.path.insert(0, '../')
sys.path.insert(0, './deps')
import argparse
import yaml
from math import fabs
from itertools import combinations, permutations
from copy import deepcopy

from a_star import AStar
from a_star1 import AStar1
import time
import mapElements
import printInColor as pic

from collections import defaultdict

def grid2graph(xy_tuple,g2g_map):
    for v in g2g_map:
        if xy_tuple in v:
            return v[1]
    
    print("grid not found in graph")
    return False

def graph2grid(vert_name,g2g_map):
    for v in g2g_map:
        if vert_name in v:
            return v[0]

    print("grid not found in graph")
    return False


class Location(object):
    def __init__(self, x=-1, y=-1):
        self.x = x
        self.y = y
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    def __str__(self):
        return str((self.x, self.y))

#Jeeho Edit
class node:
    def __init__(self):
        self.location_grid = Location()
        self.name = ""
        self.type = ""
        self.temp_open = False
    
    def __init__(self, tuple_in):
        self.location_grid = Location(tuple_in[0][0],tuple_in[0][1])
        self.name = tuple_in[1]
        self.type = tuple_in[2]
        self.temp_open = False

#Jeeho Edit
def to_node_list(tuple_list_in):
    out_list = []
    for item in  tuple_list_in:
        out_list.append(node(item))

    return out_list


class State(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location
    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location.x) + str(self.location.y))
    def is_equal_except_time(self, state):
        return self.location == state.location
    def __str__(self):
        return str((self.time, self.location.x, self.location.y))
    def __repr__(self):
        return str((self.time, self.location.x, self.location.y))

class Conflict(object):
    VERTEX = 1
    EDGE = 2
    def __init__(self):
        self.time = -1
        self.type = -1

        self.agent_1 = ''
        self.agent_2 = ''

        self.location_1 = Location()
        self.location_2 = Location()

    def __str__(self):
        return '(' + str(self.time) + ', ' + self.agent_1 + ', ' + self.agent_2 + \
             ', '+ str(self.location_1) + ', ' + str(self.location_2) + ')'

class VertexConstraint(object):
    def __init__(self, time, location):
        self.time = time
        self.location = location

    def __eq__(self, other):
        return self.time == other.time and self.location == other.location
    def __hash__(self):
        return hash(str(self.time)+str(self.location))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location) + ')'

class EdgeConstraint(object):
    def __init__(self, time, location_1, location_2):
        self.time = time
        self.location_1 = location_1
        self.location_2 = location_2
    def __eq__(self, other):
        return self.time == other.time and self.location_1 == other.location_1 \
            and self.location_2 == other.location_2
    def __hash__(self):
        return hash(str(self.time) + str(self.location_1) + str(self.location_2))
    def __str__(self):
        return '(' + str(self.time) + ', '+ str(self.location_1) +', '+ str(self.location_2) + ')'

class Constraints(object):
    def __init__(self):
        self.vertex_constraints = set()
        self.edge_constraints = set()

    def add_constraint(self, other):
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints

    def __str__(self):
        return "VC: " + str([str(vc) for vc in self.vertex_constraints])  + \
            "EC: " + str([str(ec) for ec in self.edge_constraints])
    def __len__(self):
        return len(self.vertex_constraints) + len(self.edge_constraints)
class Environment(object):
    def __init__(self, dimension, agents, obstacles, vertices_with_name, edges_dict, cat):
        self.dimension = dimension
        self.obstacles = obstacles

        self.agents = agents
        self.agent_dict = {}

        self.make_agent_dict()

        self.constraints = Constraints()
        self.constraint_dict = {}

        # Hyojeong Edit
        if cat:
            self.a_star = AStar1(self)
        else:
            self.a_star = AStar(self)
        self.cat = cat

        #Jeeho Edit
        self.node_list = []
        self.vertices_with_name = vertices_with_name
        self.edges_dict = edges_dict

        #fill node_list
        self.node_list = to_node_list(self.vertices_with_name)
    
    #Jeeho Edit
    def get_agent_goal(self, agent_name):
        for a in self.agents:
            if a["name"] == agent_name:
                return a['goal']

    def get_node_type_by_name(self,node_name):
        for n in self.node_list:
            if(n.name == node_name):
                return n.type

    def get_node_temp_open_by_name(self,node_name):
        for n in self.node_list:
            if(n.name == node_name):
                return n.temp_open

    #Jeeho Edit
    def solution2NodeNames(self,sol):
        #solution is a dictionary of lists of agent dictionaries
        #key: agent_name value: list of {'t','x','y'}
        out_sol = {}
        for sol_xy_agent in sol.keys():
            #for each solution for an agent
            sol_name_agent = []
            for cell in sol[sol_xy_agent]:
                #convert xy to node_name
                xy_tuple = (cell['x'],cell['y'])
                matched_name = grid2graph(xy_tuple,self.vertices_with_name)
                #omitting t
                sol_name_agent.append(matched_name)
                
            out_sol[sol_xy_agent] = sol_name_agent
        
        return out_sol


    def get_neighbors_from_node_name_timeless(self, node_name):
        neighbors = []
        neighbor_nodes = []
        if node_name in self.edges_dict:
            neighbor_nodes = self.edges_dict[node_name]
        for nn in neighbor_nodes:
            #Jeeho Edit
            #get corresponding graph node
            #add if node, only add if station is temp_open
            add_verdict = False
            if(self.get_node_type_by_name(nn)=='node'):
                add_verdict = True
            elif(self.get_node_temp_open_by_name(nn) == True):
                add_verdict = True
            if(add_verdict):
                neighbors.append(nn)
        
        return neighbors

    def get_neighbors(self, state):
        neighbors = []
        neighbor_nodes = []

        node_name = grid2graph((state.location.x,state.location.y),self.vertices_with_name)

        #add current state
        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        
        if node_name in self.edges_dict:
            neighbor_nodes = self.edges_dict[node_name]

        for nn in neighbor_nodes:
            #Jeeho Edit
            #get corresponding graph node
            #add if node, only add if station is temp_open
            add_verdict = False
            if(self.get_node_type_by_name(nn)=='node'):
                add_verdict = True
            elif(self.get_node_temp_open_by_name(nn) == True):
                add_verdict = True
            if(add_verdict):
                temp_tuple = graph2grid(nn,self.vertices_with_name)
                #neighbors in State data type
                n = State(state.time+1,Location(temp_tuple[0],temp_tuple[1]))

                if self.state_valid(n) and self.transition_valid(state, n):                
                    neighbors.append(n)


        """
        # Wait action
        n = State(state.time + 1, state.location)
        if self.state_valid(n):
            neighbors.append(n)
        # Up action
        n = State(state.time + 1, Location(state.location.x, state.location.y+1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Down action
        n = State(state.time + 1, Location(state.location.x, state.location.y-1))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Left action
        n = State(state.time + 1, Location(state.location.x-1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        # Right action
        n = State(state.time + 1, Location(state.location.x+1, state.location.y))
        if self.state_valid(n) and self.transition_valid(state, n):
            neighbors.append(n)
        """

        return neighbors

    def get_first_conflict(self, solution):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        combinations_of_agents = list(combinations(solution.keys(), 2))
        for t in range(max_t):
            for agent_1, agent_2 in combinations_of_agents:
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result

            for agent_1, agent_2 in combinations_of_agents:
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result
        return False

    def create_constraints_from_conflict(self, conflict):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict
    
    # Hyojeong Edit
    def get_goal_overlap_agents(self, solution):        
        agent_path_goal = dict()
        for agent, path in solution.items():
            agent_path_goal[agent] = {'path':path, 'goal':path[-1]}
        
        goal_overlap_agents = []
        for agent_1, agent_2 in permutations(agent_path_goal.keys(), 2):
            # if (a2, a1) already in set, no need to add (a1, a2)
            if (agent_2, agent_1) in goal_overlap_agents:
                continue
            for state in agent_path_goal[agent_1]['path']:
                if state.is_equal_except_time(agent_path_goal[agent_2]['goal']):
                    goal_overlap_agents.append((agent_1, agent_2))
                    break

        return goal_overlap_agents

    def get_conflict_cbs3(self, solution, prev_overlap):
        max_t = max([len(plan) for plan in solution.values()])
        result = Conflict()
        is_overlap = False
        combinations_of_agents = list(combinations(solution.keys(), 2))
        
        # if prev overlap still exist, check conflict btw those agent first
        if prev_overlap:
            agent_1, agent_2 = prev_overlap
            for t in range(max_t):
                # vertex conflict
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    return result, True
                
                # edge conflict
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)
                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)
                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    return result, True

        # if goal overlap agents exist, check conflict btw those agent first
        goal_overlap_agents = self.get_goal_overlap_agents(solution)
        if goal_overlap_agents:
            for t in range(max_t):
                # vertex conflict
                for agent_1, agent_2 in goal_overlap_agents:
                    state_1 = self.get_state(agent_1, solution, t)
                    state_2 = self.get_state(agent_2, solution, t)
                    if state_1.is_equal_except_time(state_2):
                        result.time = t
                        result.type = Conflict.VERTEX
                        result.location_1 = state_1.location
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        # print("\toverlap changed 1:", prev_overlap, "->", (agent_1, agent_2))
                        return result, True
                # edge conflict
                for agent_1, agent_2 in goal_overlap_agents:
                    state_1a = self.get_state(agent_1, solution, t)
                    state_1b = self.get_state(agent_1, solution, t+1)

                    state_2a = self.get_state(agent_2, solution, t)
                    state_2b = self.get_state(agent_2, solution, t+1)

                    if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                        result.time = t
                        result.type = Conflict.EDGE
                        result.agent_1 = agent_1
                        result.agent_2 = agent_2
                        result.location_1 = state_1a.location
                        result.location_2 = state_1b.location
                        # print("\toverlap changed 1:", prev_overlap, "->", (agent_1, agent_2))
                        return result, True

        # if prev_overlap and goal_overlap_agents is None or 
        # conflict doesn't exist from above conditions, choose first conflict
        for t in range(max_t):
            # vertex conflict
            for agent_1, agent_2 in combinations_of_agents:
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)
                if state_1.is_equal_except_time(state_2):
                    result.time = t
                    result.type = Conflict.VERTEX
                    result.location_1 = state_1.location
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    # print("\toverlap changed 2:", prev_overlap, "->", (agent_1, agent_2))
                    return result, False
            # edge conflict
            for agent_1, agent_2 in combinations_of_agents:
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t+1)

                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t+1)

                if state_1a.is_equal_except_time(state_2b) and state_1b.is_equal_except_time(state_2a):
                    result.time = t
                    result.type = Conflict.EDGE
                    result.agent_1 = agent_1
                    result.agent_2 = agent_2
                    result.location_1 = state_1a.location
                    result.location_2 = state_1b.location
                    # print("\toverlap changed 2:", prev_overlap, "->", (agent_1, agent_2))
                    return result, False

        return False, False

    def create_constraints_from_conflict_cbs3(self, conflict, is_overlap):
        constraint_dict = {}
        if conflict.type == Conflict.VERTEX:
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)
            constraint = Constraints()
            constraint.vertex_constraints |= {v_constraint}
            
            if is_overlap:
                constraint_dict[(conflict.agent_1, conflict.agent_2)] = constraint
                constraint_dict[(conflict.agent_2, conflict.agent_1)] = constraint
            else:
                constraint_dict[(conflict.agent_1, -1)] = constraint
                constraint_dict[(conflict.agent_2, -1)] = constraint

        elif conflict.type == Conflict.EDGE:
            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(conflict.time, conflict.location_1, conflict.location_2)
            e_constraint2 = EdgeConstraint(conflict.time, conflict.location_2, conflict.location_1)

            constraint1.edge_constraints |= {e_constraint1}
            constraint2.edge_constraints |= {e_constraint2}

            if is_overlap:
                constraint_dict[(conflict.agent_1, conflict.agent_2)] = constraint1
                constraint_dict[(conflict.agent_2, conflict.agent_1)] = constraint2
            else:
                constraint_dict[(conflict.agent_1, -1)] = constraint1
                constraint_dict[(conflict.agent_2, -1)] = constraint2

        return constraint_dict

    def get_state(self, agent_name, solution, t):
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        else:
            return solution[agent_name][-1]

    def state_valid(self, state):
        return state.location.x >= 0 and state.location.x < self.dimension[0] \
            and state.location.y >= 0 and state.location.y < self.dimension[1] \
            and VertexConstraint(state.time, state.location) not in self.constraints.vertex_constraints \
            and (state.location.x, state.location.y) not in self.obstacles
            #and VertexConstraint(-1, state.location) not in self.constraints.vertex_constraints

    def transition_valid(self, state_1, state_2):
        return EdgeConstraint(state_1.time, state_1.location, state_2.location) not in self.constraints.edge_constraints

    def is_solution(self, agent_name):
        pass

    def admissible_heuristic(self, state, agent_name):
        goal = self.agent_dict[agent_name]["goal"]
        return fabs(state.location.x - goal.location.x) + fabs(state.location.y - goal.location.y)

    def is_at_goal(self, state, agent_name):
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    #Hyojeong Edit
    def is_satisfy_constraints(self, agent_total_path, agent_name):
        # check if agent stays at constraint location after reaching to the goal
        if len(self.constraint_dict[agent_name].vertex_constraints) != 0:
            # path should satisfy all vertex constraints
            for vc in self.constraint_dict[agent_name].vertex_constraints:
                # only consider the case for constraint's time is bigger than length of the path
                if len(agent_total_path)-1 < vc.time:
                    # agent's location at vc.time is same as goal location
                    if agent_total_path[-1].location == vc.location:
                        return False
        return True
    
    #Hyojeong Edit
    def no_wiggle(self, agent_total_path, agent_name):
        if len(agent_total_path) >= 4:
            for i in range(len(agent_total_path)-3):
                if (agent_total_path[i].location == agent_total_path[i+2].location) and (agent_total_path[i+1].location == agent_total_path[i+3].location):
                    return False
        return True

    def make_agent_dict(self):
        for agent in self.agents:
            start_state = State(0, Location(agent['start'][0], agent['start'][1]))
            goal_state = State(0, Location(agent['goal'][0], agent['goal'][1]))

            self.agent_dict.update({agent['name']:{'start':start_state, 'goal':goal_state}})

    #Jeeho Edit
    def close_all_temp_open(self):
        #close all goals
        for agent in self.agent_dict.keys():
            #temp_goal = self.agent_dict[agent]['goal']
            for n in self.node_list:
                n.temp_open = False

    def compute_solution(self):
        solution = {}
        #Jeeho Edit
        #modify goal station accessibility by agent

        # Hyojeong Edit
        conflict_avoidance_table = defaultdict(int)

        #close all goals
        self.close_all_temp_open()

        for agent in self.agent_dict.keys():
            #open agent station goal
            g_loc = self.get_agent_goal(agent)
            g_loc_location = Location(g_loc[0],g_loc[1])
            for n in self.node_list:
                if(g_loc_location == n.location_grid):
                    n.temp_open = True
                    break
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.a_star.search(agent, conflict_avoidance_table)

            if not local_solution:
                return False
            # Hyojeong Edit
            if self.cat:
                for state in local_solution:
                    conflict_avoidance_table[state] += 1
            solution.update({agent:local_solution})
        self.close_all_temp_open()
        return solution

    def compute_solution_cost(self, solution):
        return sum([len(path) for path in solution.values()])

class HighLevelNode(object):
    def __init__(self):
        self.solution = {}
        self.constraint_dict = {}
        self.cost = 0
        self.parent_overlap = None

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.solution == other.solution and self.cost == other.cost

    def __hash__(self):
        return hash((self.cost))

    def __lt__(self, other):
        return self.cost < other.cost

    def print(self, indent, name=None):
        msg = indent+"((((((({})))))))\n".format(name)
        msg += indent+"[cost] {}\n".format(self.cost)
        msg += indent+"[constraint dict]\n"
        for agent, constraint in self.constraint_dict.items():
            msg += (indent + str(agent) + "=" + str(constraint) + "\n")
        msg += indent+"[solution]\n"
        for agent, path in self.solution.items():
            # msg += "{}={}\n".format(agent, path)
            msg += (indent + str(agent) + "=" + str(path) + "\n")
        msg += (indent + "---------------------------------------")
        print(msg)