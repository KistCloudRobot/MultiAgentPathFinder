"""
Python implementation of Conflict-based search
author: Ashwin Bose (@atb033)
Modifed: Ahn, Jeeho
"""
#from os import getresgid, path
import sys
import copy
#sys.path.insert(0, '../')
import argparse
from typing import AsyncGenerator
import yaml
#Dr. Oh Map Parse Tool
from map_parse import MapMOS as mapParser
import time
import deps.mapElements as mapElements

import deps.planningTools as pt
import deps.printInColor as pic
#import handler_tools as ht

#use arbi
"""
from python_arbi_framework.arbi_agent.agent.arbi_agent import ArbiAgent
from python_arbi_framework.arbi_agent.configuration import BrokerType
from python_arbi_framework.arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

class aAgent(ArbiAgent):
    def __init__(self, agent_name, broker_url = "tcp://127.0.0.1:61616"):
        super().__init__()
        self.broker_url = broker_url
        self.agent_name = agent_name
        #self.agent_url = agent_url

    def on_data(self, sender: str, data: str):
        print(self.agent_url + "\t-> receive data : " + data)
    
    def on_request(self, sender: str, request: str) -> str:
        print(self.agent_url + "\t-> receive request : " + request)
        return handleReqest(request)
        #return "(request ok)"

    def on_query(self, sender: str, query: str) -> str:
        print(self.agent_url + "\t-> receive query : " + query)
        #print(query)
        return handleReqest(query)

    def execute(self, broker_type=2):
        arbi_agent_excutor.excute(self.broker_url, self.agent_name, self, broker_type)
        print(self.agent_name + " ready")
"""
#use arbi end

robot_path_delim = ':'
robot_robot_delim = ';'
path_path_delim = '-'

arbiMAPF = "agent://www.arbi.com/MAPF"

args = {"param":"yaml/input.yaml","output":"yaml/output.yaml"}

class two_goals:
    def __init__(self,mGoal,tGoal):
        self.mid_goal = mGoal
        self.target_goal = tGoal

class overlap_robots:
    def __init__(self,this_robot,other_robots_list,this_goal):
        self.this_robot = this_robot
        self.other_robots_list = other_robots_list
        self.this_goal = this_goal

def msg2agentList(msg):
    # name1,start1,goal1;name2,start2,goal2, ...
    agentsList = []
    byRobots = msg.split(robot_robot_delim)
    for r in byRobots:
        elems = r.split(robot_path_delim)
        #convert node name to map index coord.
        start_xy = pt.graph2grid(elems[1],vertices_with_name)
        goal_xy = pt.graph2grid(elems[2],vertices_with_name)
        agentsList.append({'start':[start_xy[0],start_xy[1]], 'goal':[goal_xy[0],goal_xy[1]], 'name':elems[0]})

    return agentsList

#use arbi
"""
#globalized mapElements data
mapElems = mapElements.mapElements()

def msg2arbi(msg, header="MultiRobotPath", pathHeader = "RobotPath", singlePathHeader = "path"):

    if(msg == 'failed'):
        out_msg = "(" + header + " " + msg + ")"
    else:
        # result = agent1:219-220-221-222-223-224-225-15
        # (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), …)
        out_msg = "(" + header + " "
        pathList = []
        if(len(msg)>0):        
            msgList = msg.split(robot_robot_delim)
            for r in msgList:
                name_node = r.split(robot_path_delim)
                nodes = name_node[1].split(path_path_delim)
                resultPath = "(" + singlePathHeader + " " + " ".join(nodes) + ")"
                pathList.append('(' + pathHeader + " " + "\"" + name_node[0] + "\" " + resultPath + ')')
        
        out_msg += " ".join(pathList)
        out_msg += ')'

    return out_msg


def arbi2msg(arbi_msg):    
    # (MultiRobotPath (RobotPath $robot_id $cur_vertex $goal_id), …)
    # name1,start1,goal1;name2,start2,goal2, ...
    gl = GLFactory.new_gl_from_gl_string(arbi_msg)
    robotSet = []
    if(gl.get_name() == "MultiRobotPath"):
        for r in range(gl.get_expression_size()):
            #(RobotPath $robot_id $cur_vertex $goal_id)
            rp = str(gl.get_expression(r))
            #remove paranthesis
            rp_sp = rp.split('(')
            rp_sp = rp_sp[1].split(')')[0]
            #RobotPath $robot_id $cur_vertex $goal_id
            rp_elems = rp_sp.split(" ")
            # idx 1~3
            #remove quotation marks from robot name
            robot_name_sp = rp_elems[1].split('"')
            robot_name = robot_name_sp[1]
            robotSet.append(robot_name+robot_path_delim+rp_elems[2]+robot_path_delim+rp_elems[3])

    return ';'.join(robotSet)

def unique_goal_check(msg):
    verdict = True
    byRobots = msg.split(robot_robot_delim)
    if(len(byRobots)>1):
        for r in byRobots:
            pivot_sp = r.split(robot_path_delim)
            pivot_robot = pivot_sp[0]
            pivot_goal = pivot_sp[2]
            for c in byRobots:
                compare_sp = c.split(robot_path_delim)
                compare_robot = compare_sp[0]
                compare_goal = compare_sp[2]
                #skip itself
                if(pivot_robot != compare_robot):
                    if(pivot_goal == compare_goal):
                        #goals are not unique
                        verdict = False

    return verdict
                

def handleReqest(msg_gl):
    handle_start = time.time()
    print(msg_gl)
    #convert arbi Gl to custom format
    msg = arbi2msg(msg_gl)
    # name1,start1,goal1;name2,start2,goal2, ...
    
    #check if all goals are unique
    if(unique_goal_check(msg) == False):
        #goals are not unique. return fail
        return msg2arbi("failed")
    
    agentsList = []
    byRobots = msg.split(robot_robot_delim)
    for r in byRobots:
        elems = r.split(robot_path_delim)
        #convert node name to map index coord.
        start_xy = pt.graph2grid(elems[1],vertices_with_name)
        goal_xy = pt.graph2grid(elems[2],vertices_with_name)
        agentsList.append({'start':[start_xy[0],start_xy[1]], 'goal':[goal_xy[0],goal_xy[1]], 'name':elems[0]})

    #planResult = planning_loop(agentsList)
    planResult = handle_with_exceptions(agentsList)
    if(planResult == -1):
        return msg2arbi("failed")

    #serialize in string
    msgs_by_agent = []
    if(planResult != -1): #if not failed
        for key in planResult:
            msg = key + robot_path_delim + path_path_delim.join(planResult[key])
            msgs_by_agent.append(msg)

    out_msg = robot_robot_delim.join(msgs_by_agent)
    handle_end = time.time()
    pic.printC("Event Hander spent: " + str(handle_end-handle_start) + " seconds", Warning)

    #convert to arbi format
    conv = msg2arbi(out_msg)
    #return out_msg
    return conv
"""
#use arbi end

def planning_loop(agents_in,print_result=True):
    loop_start = time.time()
    #print('Waiting for a request')
    #repeating starts here
    #get data from server here
    #input()
    #agents -> list of dictionaries
    #['start'],['goal'] = list, ['name'] = str
    #agents_in = []
    #add agents as below
    #agents_in.append({'start':[0,0], 'goal':[1,1], 'name':'agent_new'})

    #wait and get agents from server, do the job, then repeat
    #assume agents info is received
    #agents_in.append({'start':[3,1], 'goal':[9,0], 'name':'agent0'})
    #agents_in.append({'start':[6,1], 'goal':[3,3], 'name':'agent1'})

    #initialize env
    env = pt.Environment(mapElems.dimension, agents_in, mapElems.obstacles, mapElems.vertices_with_name, mapElems.edges_dict)
    #env.vertices_with_name = mapElems.vertices_with_name
    #env.edges_dict = mapElems.edges_dict
    #Jeeho Edit
    #node class list
    #env.node_list = pt.to_node_list(mapElems.vertices_with_name)
    
    # Searching
    cbs = pt.CBS(env)
    start = time.time()
    solution = cbs.search(print_=print_result)
    end = time.time()
    if(print_result == True):
        print(end - start)
    if not solution:
        pic.printC(" Solution not found",'warning')
        return -1

    # Write to output file
    with open(args['output'], 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args['output'], 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)

    #Jeeho Edit
    #convert resulting path to node names
    sol_in_node_name = env.solution2NodeNames(solution)
    if(print_result==True):
        print(sol_in_node_name)

    #send through ARBI
    loop_end = time.time()
    if(print_result==True):
        pic.printC("Planning Loop took: " + str(loop_end - loop_start) + " seconds",'green')
    return sol_in_node_name
    #repeating ends here


def find_free_node(goal_node, overlap_robots, single_path_dict, env_in):
    #expand until it finds a free node. return false if not found
    opened_list = []
    closed_list = []

    nodes_to_avoid = []
    for other_robot in overlap_robots:
        nodes_to_avoid = nodes_to_avoid + single_path_dict[other_robot]

    #initial set of neighbors
    neighbors_list = env_in.edges_dict[goal_node]
    #copy nodes only (exclude stations)
    opened_list = []
    for nn in neighbors_list:
        if(env_in.get_node_type_by_name(nn)=='node'):
            opened_list.append(nn)
    
    while(len(opened_list)>0):
        for neighbor in opened_list:
            searched_in_this_iteration = []
            if(neighbor not in nodes_to_avoid): #found a free node. return without further search
                return neighbor
            else:
                closed_list.append(neighbor)
                searched_in_this_iteration.append(neighbor)

        #no free node found. continue expansion
        for node in searched_in_this_iteration:
            opened_list.remove(node)
            neighbors = env_in.edges_dict[node]
            for n in neighbors:
                if n not in closed_list and env_in.get_node_type_by_name(n)=='node':
                    opened_list.append(n)

    #failed to find a free node. exit with failure
    return False

def write_to_file(path_dict,arg,vert):   
    with open(arg['output'], 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
            output['schedule'] = {}
            len_list = []
            for robot in path_dict:
                #make list of dict
                list_of_dict = []
                for d in range(len(path_dict[robot])): #single list path
                    grid_coord = pt.graph2grid(path_dict[robot][d],vert)
                    single_dict = {'t':int(d),'x':grid_coord[0],'y':grid_coord[1]}
                    list_of_dict.append(single_dict)
                output['schedule'][robot] = list_of_dict
                len_list.append(len(list_of_dict))

            output['cost'] = max(len_list)
        except yaml.YAMLError as exc:
            print(exc)

        with open(args['output'], 'w') as output_yaml:
            yaml.safe_dump(output, output_yaml)


def handle_with_exceptions(agents_in):
    #initialize env
    
    env = pt.Environment(mapElems.dimension, agents_in, mapElems.obstacles, mapElems.vertices_with_name, mapElems.edges_dict)
    #env.vertices_with_name = mapElems.vertices_with_name
    #env.edges_dict = mapElems.edges_dict
    # get single-robot path for all robots
    single_paths_dict={}
    for robot in agents_in:
        single_agent = [robot]
        path = planning_loop(single_agent,print_result=False)
        if(path == -1): #failed to find a path
            pic.printC("Failed to find a single robot path", 'warning')
            #return -1
            single_paths_dict[robot['name']] = []
        else:
            single_paths_dict[robot['name']] = path[robot['name']]

    #check if any goal is in any path {"robot name":[list of other robots with a path has the goal on]}
    overlap_goal = {} #dict of string:overlap_robots class
    for robot in agents_in:
        goal_node = pt.grid2graph((robot['goal'][0],robot['goal'][1]),env.vertices_with_name)
        for path_key in single_paths_dict:
           #skip for itself
            if path_key is not robot['name']:
                #add to overlap goal if goal is on someone else's path
                if goal_node in single_paths_dict[path_key]:
                    #if there's no key for this robot
                    if robot['name'] not in overlap_goal:
                        overlap_ = overlap_robots(robot['name'],[path_key],goal_node)
                        overlap_goal[robot['name']] = overlap_
                    #if there's a overlap found previously, append to the list
                    else:                        
                        overlap_goal[robot['name']].other_robots_list.append(path_key)
    

    #handler_tools = ht.handler_tools(mapElems,args,yaml,agents_in)

    #single_paths_dict, overlap_goal = handler_tools.fill_single_path_overlap_dict(agents_in)
    #env = pt.Environment(mapElems.dimension, agents_in, mapElems.obstacles)
    #env.vertices_with_name = mapElems.vertices_with_name

    #see if overlap_goal is empty or not. proceed if empty, do additional handling if not
    if(bool(overlap_goal)==True): #if not empty (bool(empty dict) == False)
        finished_agents_path_dict = {}
        unfinished_agents_path_list_dict = {}
        remaining_agentList = agents_in
        #todo: overlapping to each other leads to endless loop
        while(len(remaining_agentList)>0):
            mid_goals = {}
            #expend until a node not on any of the overlapped paths is found for each robot (node to avoid collision)
            for o in overlap_goal:
                #find a node to avoid collision
                free_node = find_free_node(overlap_goal[o].this_goal,overlap_goal[o].other_robots_list,single_paths_dict,env)
                if(free_node!=False): #if a free node is found
                    #set a temp goals
                    mid_goals[overlap_goal[o].this_robot] = two_goals(free_node,overlap_goal[o].this_goal)
                else: #no free node found
                    return -1
                
            #plan with free nodes first
            for m in mid_goals:
                for a in remaining_agentList:
                    if a['name'] is m:
                        a['goal'] = pt.graph2grid(mid_goals[m].mid_goal,env.vertices_with_name)

            partial_solution = planning_loop(remaining_agentList,print_result=False)
            if(partial_solution == -1):
                return -1
            #print(p)
            #determine how long robots should stay to avoid collision
            len_list = []
            for p in partial_solution:
                #find the largest competed length
                if p not in mid_goals:
                    len_list.append(len(partial_solution[p]))

            if(len(len_list)>0):
                max_len = max(len_list)

                #append mid_goal to match length
                for p in partial_solution:
                    if p in mid_goals:
                        n_app = max_len - len(partial_solution[p]) - 1
                        list_to_app = [mid_goals[p].mid_goal] * n_app
                        partial_solution[p] += list_to_app

            #print(partial_solution)
            #move finished agent to finished list
            for r in partial_solution:
                #if agent is in not in mid_goal (finished path generation)
                if r not in mid_goals:
                    #if agent is not in unfinished list -> move to finished list
                    if r not in unfinished_agents_path_list_dict:
                        #add to finished list
                        finished_agents_path_dict[r] = partial_solution[r]
                        #remove from remaining agents
                        for i in range(len(remaining_agentList)):
                            if remaining_agentList[i]['name'] == r:
                                del remaining_agentList[i]
                                break
                    #if agent is in unfinished list -> augment then move to finished list
                    else:
                        #augment paths
                        unfinished_agents_path_list_dict[r] += partial_solution[r]
                        #add to finished list
                        finished_agents_path_dict[r] = unfinished_agents_path_list_dict[r]
                        #remove from remaining agents
                        for i in range(len(remaining_agentList)):
                            if remaining_agentList[i]['name'] == r:
                                del remaining_agentList[i]

                else: #still not finished
                    #add to unfinished list
                    if r in unfinished_agents_path_list_dict:
                        unfinished_agents_path_list_dict[r].append(partial_solution[r])
                    else:
                        unfinished_agents_path_list_dict[r] = partial_solution[r]
            
            #reset to original goals for remaining agents
            for r in remaining_agentList:
                for a in agents_in:
                    if r['name'] == a['name']:
                        r['start'] = pt.graph2grid(mid_goals[r['name']].mid_goal,env.vertices_with_name)
                        r['goal'] = pt.graph2grid(mid_goals[r['name']].target_goal,env.vertices_with_name)

            #update overlap
            overlap_goal = {} #dict of string:overlap_robots class
            for robot in remaining_agentList:
                goal_node = pt.grid2graph((robot['goal'][0],robot['goal'][1]),env.vertices_with_name)
                for path_key in single_paths_dict:
                #skip for itself
                    if path_key is not robot['name']:
                        #add to overlap goal if goal is on someone else's path
                        if goal_node in single_paths_dict[path_key] and path_key not in finished_agents_path_dict:
                            #if there's no key for this robot
                            if robot['name'] not in overlap_goal:
                                overlap_ = overlap_robots(robot['name'],[path_key],goal_node)
                                overlap_goal[robot['name']] = overlap_
                            #if there's a overlap found previously, append to the list
                            else:                        
                                overlap_goal[robot['name']].other_robots_list.append(path_key)                        
        
        pic.printC("Returning Result with additional via-points",'green')
        print(finished_agents_path_dict)
        write_to_file(finished_agents_path_dict,args,env.vertices_with_name)
        return finished_agents_path_dict
                
    else: #fine without additional handlings
        return planning_loop(agents_in)


def main():
    #Initialize Arbi Client Agent
    #start an agent

    #use arbi
    """
    arbiAgent = aAgent(agent_name=arbiMAPF)
    arbiAgent.execute()

    arbiAgent.send("agent://www.arbi.com/receiveTest","Hi Bmo");
    """
    #use arbi end
    
    #parser = argparse.ArgumentParser()
    #parser.add_argument("param", help="input file containing map and obstacles")
    #parser.add_argument("output", help="output file with the schedule")
    #args = parser.parse_args()


    # Read from input file
    with open(args['param'], 'r') as arg:
        try:
            param = yaml.load(arg, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)


    dimension = param["map"]["dimensions"]
    #obstacles = param["map"]["obstacles"]
    vertices_yaml = param["map"]["vertices"] #list
    vertices = []
    global vertices_with_name
    vertices_with_name = [] #list of tuple
    for item in vertices_yaml:
        vertices.append((item[0],item[1]))
        vertices_with_name.append(((item[0],item[1]),item[2],item[3]))

    obstacles = []
    agents = param['agents']

    for x in range(dimension[0]):
        for y in range(dimension[1]):
            if((x,y) not in vertices):
                obstacles.append((x,y))

    #Jeeho Edit
    #parse edges
    edges_dict = mapParser.MapMOS("map_parse/map_cloud.txt").Edge

    #initialize Map Elements data
    global mapElems
    mapElems = mapElements.mapElements(dimension,obstacles,vertices_with_name,edges_dict)
    
    #Jeeho Comment
    #Need to wrap from here so we can setup new env before start searching
    #currently predefined map construction may need to be modified to be done automatically
    #from server data
    #Environment should be re-initialized with modified agents

    agents_in = []
    #add agents as below
    #agents_in.append({'start':[0,0], 'goal':[1,1], 'name':'agent_new'})

    #wait and get agents from server, do the job, then repeat
    #assume agents info is received

    # assume each target is not an obstacle node


    #test run
    
    #a1 = ["a1","201","222"]
    #a2 = ["a2","221","216"]
    #b1 = ["b1","203","240"]
    #b2 = ["b2","238","204"]
    a1 = ["a1","233","239"]
    a2 = ["a2","240","204"]
    test_robots = []
    test_robots.append(a1)
    test_robots.append(a2)
    #test_robots.append(b1)
    #test_robots.append(b2)

    #a1_start = pt.graph2grid(a1[0],vertices_with_name)
    #a2_start = pt.graph2grid(a2[0],vertices_with_name)
    #a1_goal = pt.graph2grid(a1[1],vertices_with_name)
    #a2_goal = pt.graph2grid(a2[1], vertices_with_name)

    #agents_in.append({'start':a1_start, 'goal':a1_goal, 'name':'agent0'})
    #agents_in.append({'start':a2_start, 'goal':a2_goal, 'name':'agent1'})
    
    msg_list = []
    for r in test_robots:
        msg_list.append(r[0] + robot_path_delim + r[1] + robot_path_delim + r[2])

    msg = robot_robot_delim.join(msg_list)
    
    agents_in = msg2agentList(msg)
    handle_with_exceptions(agents_in)
    
    #test run end
    
    while(1):
        #   planResult = planning_loop()
        time.sleep(0.01)

    #close arbi agent
    #arbiAgent.close()


if __name__ == "__main__":
    
    main()
    