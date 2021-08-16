"""
Python implementation of Conflict-based search

author: Ashwin Bose (@atb033)
Modifed: Ahn, Jeeho
"""
import sys
sys.path.insert(0, '../')
import argparse
import yaml
#Dr. Oh Map Parse Tool
from map_parse import MapMOS as mapParser
import time
import mapElements
import a_agent
import planningTools as pt

#globalized mapElements data
mapElems = mapElements.mapElements()

def planning_loop():
    print('Waiting for a request')
    #repeating starts here
    #get data from server here
    input()
    #agents -> list of dictionaries
    #['start'],['goal'] = list, ['name'] = str
    agents_in = []
    #add agents as below
    #agents_in.append({'start':[0,0], 'goal':[1,1], 'name':'agent_new'})

    #wait and get agents from server, do the job, then repeat
    #assume agents info is received
    agents_in.append({'start':[3,1], 'goal':[9,0], 'name':'agent0'})
    agents_in.append({'start':[6,1], 'goal':[3,3], 'name':'agent1'})

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
        print(" Solution not found" )
        return

    """
    # Write to output file
    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    output["schedule"] = solution
    output["cost"] = env.compute_solution_cost(solution)
    with open(args.output, 'w') as output_yaml:
        yaml.safe_dump(output, output_yaml)
    """

    #Jeeho Edit
    #convert resulting path to node names
    sol_in_node_name = env.solution2NodeNames(solution)
    print(sol_in_node_name)

    #send through ARBI
    return sol_in_node_name
    #repeating ends here


def main():
    #Initialize Arbi Client Agent
    #start an agent
    arbiAgent = a_agent.aAgent(agent_name="agent://www.arbi.com/MAPFagent")
    arbiAgent.execute()

    arbiAgent.send("agent://www.arbi.com/receiveTest","Hi Bmo");
    
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    #obstacles = param["map"]["obstacles"]
    vertices_yaml = param["map"]["vertices"] #list
    vertices = []
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

    while(1):
       planResult = planning_loop()
       time.sleep(2)

    #close arbi agent
    arbiAgent.close()


if __name__ == "__main__":
    
    main()
    