"""
Python implementation of Conflict-based search
author: Ashwin Bose (@atb033)
Modifed: Ahn, Jeeho
"""
from os import getresgid, path
import sys
#sys.path.insert(0, '../')
import argparse
import yaml
#Dr. Oh Map Parse Tool
from map_parse import MapMOS as mapParser
import time
import deps.mapElements as mapElements
#import a_agent
import deps.planningTools as pt
import deps.printInColor as pic


from python_arbi_framework.arbi_agent.agent.arbi_agent import ArbiAgent
from python_arbi_framework.arbi_agent.configuration import BrokerType
from python_arbi_framework.arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

robot_path_delim = ':'
robot_robot_delim = ';'
path_path_delim = '-'

arbiMAPF = "agent://www.arbi.com/MAPF"

args = {"param":"yaml/input.yaml","output":"yaml/output.yaml"}


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



#globalized mapElements data
mapElems = mapElements.mapElements()

def msg2arbi(msg, header="MultiRobotPath", pathHeader = "RobotPath", singlePathHeader = "path"):
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


def handleReqest(msg_gl):
    handle_start = time.time()
    print(msg_gl)
    #convert arbi Gl to custom format
    msg = arbi2msg(msg_gl)
    # name1,start1,goal1;name2,start2,goal2, ...
    agentsList = []
    byRobots = msg.split(robot_robot_delim)
    for r in byRobots:
        elems = r.split(robot_path_delim)
        #convert node name to map index coord.
        start_xy = pt.graph2grid(elems[1],vertices_with_name)
        goal_xy = pt.graph2grid(elems[2],vertices_with_name)
        agentsList.append({'start':[start_xy[0],start_xy[1]], 'goal':[goal_xy[0],goal_xy[1]], 'name':elems[0]})

    planResult = planning_loop(agentsList)

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


def planning_loop(agents_in):
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
    env = pt.Environment(mapElems.dimension, agents_in, mapElems.obstacles)
    env.vertices_with_name = mapElems.vertices_with_name
    env.edges_dict = mapElems.edges_dict
    #Jeeho Edit
    #node class list
    env.node_list = pt.to_node_list(mapElems.vertices_with_name)
    
    # Searching
    cbs = pt.CBS(env)
    start = time.time()
    solution = cbs.search()
    end = time.time()
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
    print(sol_in_node_name)

    #send through ARBI
    loop_end = time.time()
    pic.printC("Planning Loop took: " + str(loop_end - loop_start) + " seconds",'green')
    return sol_in_node_name
    #repeating ends here


def main():
    #Initialize Arbi Client Agent
    #start an agent
    arbiAgent = aAgent(agent_name=arbiMAPF)
    arbiAgent.execute()

    arbiAgent.send("agent://www.arbi.com/receiveTest","Hi Bmo");
    
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
    #agents_in.append({'start':[3,1], 'goal':[9,1], 'name':'agent0'})
    #agents_in.append({'start':[9,2], 'goal':[5,1], 'name':'agent1'})
    #planning_loop(agents_in)

    while(1):
    #   planResult = planning_loop()
       time.sleep(0.01)

    #close arbi agent
    arbiAgent.close()


if __name__ == "__main__":
    
    main()
    