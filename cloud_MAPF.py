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
from deps.cbs import CBS
from deps.cbs3 import CBS3
#import handler_tools as ht

USE_ARBI = True

sys.path.append("/home/kist/pythonProject/Python-mcArbiFramework")

from arbi_agent.configuration import BrokerType
from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.agent import arbi_agent_executor
from arbi_agent.model import generalized_list_factory as GLFactory

robot_path_delim = ':'
robot_robot_delim = ';'
path_path_delim = '-'

args = {"param":"yaml/input.yaml","output":"yaml/output.yaml"}
MAP_CLOUD_PATH = "map_parse/map_cloud.txt"

agent_name = "agent://www.arbi.com/MultiAgentPathFinder"
# broker_host = "127.0.0.1"
broker_host = "172.16.165.143"
# broker_host = "192.168.100.10"
broker_port = 61316
broker_type = BrokerType.ACTIVE_MQ
# broker_type = BrokerType.ZERO_MQ

#use arbi
if USE_ARBI:

    class aAgent(ArbiAgent):
        def __init__(self):
            super().__init__()
            self.broker_url = broker_host
            self.agent_name = agent_name
            #self.agent_url = agent_url

        def on_data(self, sender: str, data: str):
            print("receive data : " + data)

        def on_request(self, sender: str, request: str) -> str:
            print("receive request : " + request)
            return handleReqest(request)
            #return "(request ok)"

        def on_query(self, sender: str, query: str) -> str:
            print("receive query : " + query)
            #print(query)
            return handleReqest(query)

    def msg2arbi(msg, header="MultiRobotPath", pathHeader = "RobotPath", singlePathHeader = "path"):

        if(msg == 'failed'):
            out_msg = "(" + header + " " + msg + ")"
        else:
            # result = agent1:219-220-221-222-223-224-225-15
            # (MultiRobotPath (RobotPath $robot_id (path $v_id1 $v_id2 $v_id3, ...)), ???)
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
        # (MultiRobotPath (RobotPath $robot_id $cur_vertex $goal_id), ???)
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

        planResult = planning_loop(agentsList)
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
#use arbi end

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


def planning_loop(agents_in,print_result=True):
    #wait and get agents from server, do the job, then repeat
    #assume agents info(agents_in) is received    
    loop_start = time.time()

    #initialize env
    env = pt.Environment(mapElems.dimension, agents_in, mapElems.obstacles, mapElems.vertices_with_name, mapElems.edges_dict)
    
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


def main():
    #Initialize Arbi Client Agent
    #start an agent

    #use arbi
    if USE_ARBI:
        agent = aAgent()
        arbi_agent_executor.execute(broker_host, broker_port, agent_name, agent, broker_type)
    #use arbi end

    # Read from input file
    with open(args['param'], 'r') as arg:
        try:
            param = yaml.load(arg, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # get param
    dimension = param["map"]["dimensions"]
    #obstacles = param["map"]["obstacles"]
    vertices_yaml = param["map"]["vertices"] #list
    agents = param['agents']

    vertices = []
    global vertices_with_name
    vertices_with_name = [] #list of tuple
    for item in vertices_yaml:
        vertices.append((item[0],item[1]))
        vertices_with_name.append(((item[0],item[1]),item[2],item[3]))

    # assume each target is not an obstacle node
    obstacles = [] 
    for x in range(dimension[0]):
        for y in range(dimension[1]):
            if((x,y) not in vertices):
                obstacles.append((x,y))    

    #initialize Map Elements data
    #Environment should be re-initialized with modified agents
    global mapElems
    edges_dict = mapParser.MapMOS(MAP_CLOUD_PATH).Edge
    mapElems = mapElements.mapElements(dimension,obstacles,vertices_with_name,edges_dict)

    #add agents as below
    #agents_in.append({'start':[0,0], 'goal':[1,1], 'name':'agent_new'})
    agents_in = []

    #test run
    if not USE_ARBI:
        # case1
        a1 = ["AMR_LIFT1", "140", "118"]
        a2 = ["AMR_LIFT2", "107", "104"]
        a3 = ["AMR_LIFT3", "150", "116"]
        a4 = ["AMR_LIFT4", "116", "115"]

        # # case2
        # a1 = ["AMR_LIFT1", "138", "138"]
        # a2 = ["AMR_LIFT2", "115", "148"]
        # a3 = ["AMR_LIFT3", "142", "103"]
        # a4 = ["AMR_LIFT4", "102", "102"]

        # # case3
        # a1 = ["AMR_LIFT1", "103", "129"]
        # a2 = ["AMR_LIFT2", "126", "130"]
        # a3 = ["AMR_LIFT3", "150", "127"]
        # a4 = ["AMR_LIFT4", "135", "138"]

        # # case4
        # a1 = ["AMR_LIFT1", "146", "106"]
        # a2 = ["AMR_LIFT2", "107", "104"]
        # a3 = ["AMR_LIFT3", "149", "140"]
        # a4 = ["AMR_LIFT4", "115", "148"]

        test_robots = []
        test_robots.append(a1)    
        test_robots.append(a2)
        test_robots.append(a3)
        test_robots.append(a4)
        
        msg_list = []
        for r in test_robots:
            msg_list.append(r[0] + robot_path_delim + r[1] + robot_path_delim + r[2])

        msg = robot_robot_delim.join(msg_list)
        
        agents_in = msg2agentList(msg)
        planning_loop(agents_in)
    #test run end

    #wait and get agents from server, do the job, then repeat
    #assume agents info is received
    else:    
        while(1):
            time.sleep(0.01)

    #close arbi agent
    #arbiAgent.close()


if __name__ == "__main__":
    main()
    
