"""

AStar search

author: Ashwin Bose (@atb033)

modifed: Hyojeong Kim (@rlagywjd802)

"""

class AStar1():
    def __init__(self, env):
        self.agent_dict = env.agent_dict
        self.admissible_heuristic = env.admissible_heuristic
        self.is_at_goal = env.is_at_goal
        self.is_satisfy_constraints = env.is_satisfy_constraints
        self.no_wiggle = env.no_wiggle
        self.get_neighbors = env.get_neighbors
        self.env = env

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]
    
    # Hyojeong Edit
    def sum_of_move(self, came_from, current):
        move = 0
        total_path = self.reconstruct_path(came_from, current)
        for ii in range(1, len(total_path)):
            if total_path[ii].location != total_path[ii-1].location:
                move += 1
        return move
    ###########################

    def search(self, agent_name, cat):
        """
        low level search 
        """
        initial_state = self.agent_dict[agent_name]["start"]
        step_cost = 1
        
        closed_set = set()
        open_set = {initial_state}

        came_from = {}

        g_score = {} 
        g_score[initial_state] = 0

        # Hyojeong Edit 
        g1_score = {} 
        g1_score[initial_state] = 0
        ###########################

        f_score = {} 
        f_score[initial_state] = self.admissible_heuristic(initial_state, agent_name)
        
        n_lim = self.env.dimension[0]*self.env.dimension[1]
        n_lim1 = (self.env.dimension[0]+self.env.dimension[1])*2
        n_count = 0
        while open_set:
            n_count += 1
            # if(n_count > n_lim): #consider failed
            #     break
            temp_dict = dict()
            temp_cat_dict = dict()
            for open_item in open_set:
                temp_dict[open_item] = f_score.setdefault(open_item, float("inf"))
                temp_cat_dict[open_item] = cat[open_item]

            # Hyojeong Edit 
            current = min(temp_dict, key=lambda x: (temp_dict.get(x), temp_cat_dict.get(x)))
            if (temp_dict[current]) > n_lim1:
                print("{} exceed limit!!!".format(agent_name))
                break
            ###########################

            # Hyojeong Edit - add constraint satisfy condition
            agent_total_path = self.reconstruct_path(came_from, current)
            if self.is_at_goal(current, agent_name) and self.is_satisfy_constraints(agent_total_path, agent_name):
                return agent_total_path
            ###########################

            open_set -= {current}
            closed_set |= {current}

            neighbor_list = self.get_neighbors(current)
            for neighbor in neighbor_list:
                if neighbor in closed_set:
                    continue
                
                # Hyojeong Edit - add g1_score
                tentative_g_score = g_score.setdefault(current, float("inf")) + step_cost

                if neighbor not in open_set:
                    open_set |= {neighbor}
                elif tentative_g_score > g_score.setdefault(neighbor, float("inf")):
                    continue

                came_from[neighbor] = current

                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self.admissible_heuristic(neighbor, agent_name)
                ###########################
        
        return False