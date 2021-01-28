'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh
'''
import math
import matplotlib.pyplot as plt
import numpy as np
import tripy
from collections import defaultdict 

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import map_display
from Robot_csv_lib import read_map_csv
from Program_config import *
from Robot_control_panel import *



config = Config()
def shortest_trianges_path_to_goal(start_pt, goal_pt, triangles, graph):
    # find triangle that contain start point 
    begin = [point_belong_triangle(start_pt, triangle) for triangle in triangles]
    begin = get_index_true(begin)
    begin = begin[-1]
    
    # find triangle that contain goal point 
    end = [point_belong_triangle(goal_pt, triangle) for triangle in triangles]
    end = get_index_true(end)
    
    print ("Begin: ", begin)
    print ("end", end)
    return BFS_SP(graph, begin, end)
    
# Function to find the shortest 
# path between two nodes of a graph 
def BFS_SP(graph, start, goals): 
    explored = [] 
      
    # Queue for traversing the  
    # graph in the BFS 
    queue = [[start]] 
      
    # If the desired node is  
    # reached 
    for goal in goals:
        if start == goal: 
            print("Same Node") 
            return start
      
    # Loop to traverse the graph  
    # with the help of the queue 
    while queue: 
        path = queue.pop(0) 
        node = path[-1] 
          
        # Codition to check if the 
        # current node is not visited 
        if node not in explored: 
            neighbours = graph[node] 
              
            # Loop to iterate over the  
            # neighbours of the node 
            for neighbour in neighbours: 
                new_path = list(path) 
                new_path.append(neighbour) 
                queue.append(new_path) 
                  
                # Condition to check if the  
                # neighbour node is the goal 
                for goal in goals:
                    if neighbour == goal: 
                        print("Shortest path = ", new_path) 
                        return new_path
            explored.append(node) 
  
    # Condition when the nodes  
    # are not connected 
    print("So sorry, but a connecting path doesn't exist :(") 
    return []
def inside_root_area(root_path, open_path, site, ptC):
    opposite_site = 1 - site
    print ("MAIN - Opp", site, opposite_site)
    if len (root_path[site]) >= 2:
        ptR = root_path[site][-2]
        ptA = root_path[site][-1]
        if len(root_path[opposite_site]) == 1:
            ptB = open_path[opposite_site]
        else:
            ptB = root_path[opposite_site][-1]
        
        Op_in, _ = inside_angle_area(ptC, ptR, (ptA, ptB))
        print (ptR, ptA, " OPP -", ptB, " point ", ptC)
        print ("in or out ", Op_in)
        return Op_in
    else:
        return False
        

    
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


def get_nodeL2(Tri_1, Tri_2):
    Tri2_status = [point_belong_triangle(pt, Tri_1) for pt in Tri_2]
    L2_idx = get_index_false(Tri2_status)
    node_L2 = Tri_1[L2_idx[0]]
    print ("node_L2", node_L2)
    return node_L2

def plot_shortest_path(plt, root_path, open_path, found_goal):

    for i in range(len(root_path[0])-1):
        plot_line(plt, [root_path[0][i],root_path[0][i+1]], ls="-r")

    last_root = root_path[0][-1]
    plot_line(plt, [last_root, open_path[0]], ls="-g") 
    

    for i in range(len(root_path[1])-1):
        plot_line(plt, [root_path[1][i],root_path[1][i+1]], ls="-b")
    last_root = root_path[1][-1]
    plot_line(plt, [last_root, open_path[1]], ls="-k") 
    
    plot_point (plt, open_path[0], "ob")
    plot_point (plt, open_path[1], "ob")
    
    if len(found_goal) > 0:
        plot_line(plt, [found_goal, open_path[0]], ls="-.m")
        plot_line(plt, [found_goal, open_path[1]], ls="-.m")
        
def plot_shortest_triangles_path(center_pts, spath):
    for i in range(len(spath) -1):
        plot_line(plt, [center_pts[spath[i] ],center_pts[spath[i+1]]], ls="-b")
              
def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")

    menu_result = menu()
    run_times = menu_result[0]
    mapname = menu_result[1]
    start = 0 
    goal = 28
    goal = 14
    #goal = 24
    ob = read_map_csv(mapname) 
    ob = np.array(ob)
    print ("OB", ob)
    print ("OB1", ob[0])
    print ("OB2", ob[:,1])
    
    # Polygon can be clockwise or counter-clockwise
    #polygon = np.concatenate((ob, [ob[0]]), axis=0)
    polygon = ob
    #print ("polygon", polygon)
    triangles = tripy.earclip(polygon)
    
    #triangles = np.array(triangles)
    #print ("triangles", triangles)

          
    # draw map obstacles 
    map_display(plt, mapname, ob)
    
    # display triangle
    plot_triangles(plt,triangles)    
    
    center_pts = center_triangles(triangles)
    print ("center_pts", center_pts)
    
    # display center_pts
    #plot_center(plt, center_pts)  
    
    edges  = get_pairs_triangles(triangles)
    edges  = np.array(edges )
     
    # display connected paths 
    #plot_edge(plt, center_pts, edges)
    
    graph = build_graph(edges)
    
    print (graph)
    
    spath = shortest_trianges_path_to_goal(ob[start], ob[goal], triangles, graph)

    #plot_shortest_triangles_path(center_pts, spath)

    #start point
    plot_point_text(plt, ob[start], ls="*m", text="start!")

    #goal
    plot_point_text(plt, ob[goal], ls="*r", text="goal!")

    
    root_path = []
    open_path = []
    first = True
    #start point
    spt = ob[start]
    gpt = ob[goal]
    root_path = [[spt],[spt]]   # for 0 and 1
    print ("len root path 0", len(root_path[0]))
    found_goal = []
    for i in range(len(spath)): # 
    #for i in range(4):
        Tri = triangles[spath[i]]
        print ("\n\n\nProcess spath", spath[i], Tri)
        direction = 0
        # get 2 open points
        
        if first:
            Tri_status = belong_triangle(spt, Tri)       
            Opt_idx = get_index_false(Tri_status)
            Opt0 = Tri[Opt_idx[0]]
            Opt1 = Tri[Opt_idx[1]]

            # check goal
            pd_go_0 = point_dist(gpt, Opt0)
            pd_go_1 = point_dist(gpt, Opt1)
            if  math.isclose(pd_go_0,0) or math.isclose(pd_go_1,0):
                print("find goal")
                return [spt, gpt]
            open_path = [Opt0, Opt1]
            
        else:
            # get the first 2 direction, ignore the last 2
            LastOpt0 = open_path[0]
            LastOpt1 = open_path[1]

            root0 = root_path[0][-1]
            root1 = root_path[1][-1]
            
            print ("last 2 points", LastOpt0, LastOpt1)
            
            # get the next point of triangle (L2)
            Tri_status_0 = belong_triangle(LastOpt0, Tri)
            Tri_status_1 = belong_triangle(LastOpt1, Tri)
            next_pt_status = np.logical_or(Tri_status_0,Tri_status_1)
            NextOp_idx = get_index_false(next_pt_status)
            NextOpt = Tri[NextOp_idx[0]]
            print ("NEXT open point", NextOpt)
            
            # check goal
            pd_go_0 = point_dist(gpt, NextOpt)
            pd_go_1 = point_dist(gpt, Opt1)
            if  math.isclose(pd_go_0,0) or math.isclose(pd_go_1,0):
                print("find goal")
                found_goal = gpt
            else:
                next_Tri = triangles[spath[i+1]]
                if point_belong_triangle(LastOpt0, next_Tri):
                    direction = 0
                    open_path =  [LastOpt0, NextOpt]
                else:
                    direction = 1
                    open_path =  [NextOpt, LastOpt1]
                print ("NEW Open points", open_path, " with direction ", direction)
                
                if direction == 0:
                    # process left path
                    lstpt0_in, _ = inside_angle_area(LastOpt0, root1, (NextOpt, LastOpt1))
                    lstpt1_in, _ = inside_angle_area(LastOpt1, root1, (NextOpt, LastOpt0))
                    print ("L0 in L1 in", lstpt0_in, lstpt1_in)
                    if lstpt0_in:
                        root_path[1] = root_path[0]
                        root_path[1].append(LastOpt0)
                    elif lstpt1_in:
                        root_path[1].append(LastOpt1)
                    else:
                        if inside_root_area(root_path, open_path, 1, NextOpt):
                            root_path[1].pop(-1)
                else:
                    # process right path
                    lstpt0_in, _ = inside_angle_area(LastOpt0, root0, (NextOpt, LastOpt1))
                    lstpt1_in, _ = inside_angle_area(LastOpt1, root0, (NextOpt, LastOpt0))
                    print ("L0 in L1 in", lstpt0_in, lstpt1_in)
                    if lstpt0_in:
                        root_path[0].append(LastOpt0)
                    elif lstpt1_in:
                        root_path[0] = root_path[1]
                        root_path[0].append(LastOpt1)
                    else:
                        if inside_root_area(root_path, open_path, 0, NextOpt):
                            root_path[0].pop(-1)

        print ("root path:", root_path)
        print ("open path:", open_path)
        
        #display for debug
        
        # clear plot
        plt.cla()
        # draw map obstacles 
        map_display(plt, mapname, ob)
        
        # display triangle
        plot_triangles(plt,triangles)  
        #start point
        plot_point_text(plt, ob[start], ls="*m", text="start!")

        #goal
        plot_point_text(plt, ob[goal], ls="*r", text="goal!")
        plot_shortest_path(plt, root_path, open_path, found_goal)
        plt.pause(1)
        first = False

    plt.axis("equal") # make sure ox oy axises are same resolution open local points
     
    print("Done")

    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    #main(robot_type=RobotType.circle)
