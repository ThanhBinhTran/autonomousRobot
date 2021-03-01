import numpy as np
from sys import float_info
from Robot_lib import *    
from collections import defaultdict 

def ranking_score(angle, distance):
        
    # ranking = a/as + b/dist
    a = 1
    b = 1
    angle = angle/math.pi
    distance = distance/100
    #print ("Ranking score: angle {0}, distance {1}".format(angle, distance) )
    if not math.isclose(angle, 0.0) and not math.isclose(distance, 0.0):
        r_score = a/abs(angle**2) + b/(distance**2)
    else:    
        r_score = float_info.max
    return r_score
    
def ranking(center, pt, goal):
    '''
    score the open point by its angle (from center to point and goal) and its distance (to goal)
    '''

    sa =  signed_angle(goal-center, pt - center)
    dist = point_dist (goal, pt)
    rank_score = ranking_score(sa, dist)
    return [rank_score]
    
def pick_next(ao_gobal):
    '''
    return index and value of next point if there exist any active open point
    otherwise -1
    '''
    pick_idx = -1
    next_pt = []
    if len(ao_gobal) > 0:
        ranks = ao_gobal[:,2]
        #print ("global open points: ", ao_gobal[:,0:2])
        #print ("global ranks: ",ranks)
        #print ("pick index ", np.argmax(ranks))
        #print ("picked rank ", np.amax(ranks))
        pick_idx = np.argmax(ranks)
        next_pt = ao_gobal[pick_idx,0:2]
        
    return pick_idx, next_pt 
def all_remaining_point_same_side(a, b, c, obstacle_list):
    ab = np.array([[a,b]])
    # ax + by = c
    dist = np.dot(ab, obstacle_list) - c
    #print ("dist ", dist, "a {0}, b {1}, c {2} ".format(a, b, c))
    sameside1 = (dist >= 0)
    sameside2 = (dist <= 0)
    if sum(sameside1[0]) == len(sameside1[0]) or sum(sameside2[0]) == len(sameside2[0]):
        return True
    return False
    
def get_possible_AH_next_points_from_point(AH_points, obstacle_list, startpoint, goal):
    # return [start point, [all next possible next point], [add blind points from AH_points], reach goal = true/false]
    possible_AH_next_points = []
    blind_point = []
    goal_apprear = False
    print ("start point .........", startpoint)
    for i in range (len(obstacle_list[0])): # get number of column
        [a, b, c] = line_from_points(startpoint, obstacle_list[:,i])
        #print ("From point ", obstacle_list[:,i])
        get_dist = all_remaining_point_same_side(a, b, c, obstacle_list)
        if get_dist:
            # Check if selected point is existed in the path
            point = (obstacle_list[0,i], obstacle_list[1,i])
            if point not in AH_points:
                print ("__Linked ", obstacle_list[:,i])
                # concatenate path to matrix path
                AH_points.append((obstacle_list[0,i], obstacle_list[1,i]))
                print ("__PATH  ", AH_points)
                #plt.plot((startpoint[0],obstacle_list[0][i]), (startpoint[1], obstacle_list[1][i]), "--r")
                #plt.plot(obstacle_list[0][i], obstacle_list[1][i], "or")
                possible_AH_next_points.append(obstacle_list[:,i])
        else:
            blind_point.append(obstacle_list[:,i])
            #plt.plot((startpoint[0],obstacle_list[0][i]), (startpoint[1], obstacle_list[1][i]), "--g")
        #plt.pause(0.1)
        
    [a, b, c] = line_from_points(startpoint, goal)
    #print ("Check the goal ", obstacle_list[:,i])
    goal_apprear = all_remaining_point_same_side(a, b, c, obstacle_list)
    
    return  [startpoint, possible_AH_next_points, blind_point, goal_apprear]
                
def find_AH_paths(ox_b, oy_b, startpoint, goal):
    obstacle_list = np.array([ox_b, oy_b])
    print (obstacle_list)
    AH_paths = []
    AH_points = []
    AH_points.append(startpoint)
    for start_point in AH_points:
        AH_path_temp = get_possible_AH_next_points_from_point(AH_points, obstacle_list, start_point, goal)
        AH_paths.append(AH_path_temp)
    return AH_paths
    
# Function to build the graph 
def build_graph(edges): 
    edges 
    graph = defaultdict(list) 
      
    # Loop to iterate over every  
    # edge of the graph 
    for edge in edges: 
        a, b = edge[0], edge[1] 
          
        # Creating the graph  
        # as adjacency list 
        graph[a].append(b) 
        graph[b].append(a) 
    return graph 
    
# Function to initialize a graph
def graph_intiailze():
    return defaultdict(list) 

# Function to insert edges into graph
def graph_insert(graph, pnode, leafs): 
    #print ("__pnode:", pnode)
    #print ("__leafs:", leafs)
    if len(leafs) > 0:
        for leaf in leafs: 
            graph[tuple(pnode)].append(tuple(leaf)) 
            graph[tuple(leaf)].append(tuple(pnode)) 

# Function to find the shortest 
# path between two nodes of a graph 
def BFS_SP(graph, start, goal): 
    print ("Start:", start)
    print ("Goal:", goal)
    #print ("graph", graph)
    explored = []
      
    # Queue for traversing the  
    # graph in the BFS 
    queue = [[start]] 
      
    # If the desired node is  
    # reached 
    if start == goal: 
        print("Same Node") 
        return start
      
    # Loop to traverse the graph  
    # with the help of the queue 
    while queue: 
        path = queue.pop(0) 
        node = path[-1] 
          
        # Condition to check if the 
        # current node is not visited 
        if node not in explored: 
            neighbours = graph[node] 
            #print ("_NODE:", node)
            # Loop to iterate over the  
            # neighbours of the node 
            for neighbour in neighbours: 
                #print ("___neighbour:", neighbour)
                new_path = list(path) 
                new_path.append(neighbour) 
                queue.append(new_path) 
                  
                # Condition to check if the  
                # neighbour node is the goal 
                if neighbour == goal: 
                    print("Shortest path = ", new_path) 
                    return new_path
            explored.append(node) 
  
    # Condition when the nodes  
    # are not connected 
    print("So sorry, but a connecting path doesn't exist :(") 
    return []