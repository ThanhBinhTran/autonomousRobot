"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

from torch import true_divide




sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import matplotlib.pyplot as plt
try:
    from Robot_ranking import Ranking_function
    from Robot_base import Picking_strategy, Ranking_type
    from Robot_paths_lib import *
    from Plotter_lib import *
    from Robot_sight_lib import *
    from Robot_class import Robot
    from Obstacles import Obstacles
    from Robot_ranking import Ranker
    from Robot_map_lib import Map
    from Tree import *
    from Result_log import *
    from logging_ranking import *
except ImportError:
    raise


def main():
    map_name = "_map.csv"
    robot_vision = 20
    ranker = Ranker()
    plotter = Plotter(title="The_ASP_for_Autonomous_Robot_2_node")
    # display map
    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(None, map_name)
    Map().display(plt=plt, title="", obstacles=obstacles.obstacles)
    
    start = tuple((0,0))
    goal = tuple((90,90))
    robot = Robot(start=start, vision_range=robot_vision, goal=goal)
    ranking_tree = False
    ranking_type=Ranking_type.Distance_Angle
    if ranking_tree:
        step_size = robot.vision_range
        sample_size = 500
        # logging ranking
        rank_logger = Logging_ranking()
        r_logger_filename = rank_logger.set_logging_name(map_name=map_name, goal=goal,
            radius=robot_vision, step_size=step_size, sample_size=sample_size)

        # find working space boundary
        boundary_area = robot.find_working_space_boundaries(obstacles=obstacles)
        start_node = Node(goal, cost=0)            # initial root node, cost to root = 0
        if rank_logger.is_existed_log_file(r_logger_filename):
            print ("load existed-RRTreeStart_rank: ", r_logger_filename)
            RRT_star = rank_logger.load(r_logger_filename)
        else:
            print ("Generating new RRTree star then save to: ", r_logger_filename)
            RRT_star = RRTree_star(root=start_node, step_size=step_size, radius=robot.vision_range, 
                            random_area=boundary_area, sample_size=sample_size)
            RRT_star.build(goal_coordinate=start, plotter=plotter, obstacles=obstacles)

            # save ranking tree
            rank_logger.save_tree(RRTree_star=RRT_star, file_name=r_logger_filename)
    else:
        RRT_star = None
    center_points = np.array([[5, 0],
                       [-5, 90],
                       [50, 50.0],
                       [90, 10.0]
                       ])
    vision_list = []
    

    for center in center_points:
        robot.coordinate = tuple(center)
        csight, open_sights  = scan_around(robot, obstacles, goal)

        #open_sights = np.array(open_sights)
        #open_local_pts = open_sights[:, 2]    # open_local_pts
        robot.get_local_active_open_ranking_points(open_sights=open_sights, ranker=ranker, goal=goal,\
                                                        RRT_star=RRT_star, ranking_type=ranking_type)
        # stack local active open point to global set
        robot.expand_global_open_ranking_points(robot.local_active_open_rank_pts)
            
        # add new active open points to graph_insert
        robot.visibility_graph.add_local_open_points(robot.coordinate, robot.local_active_open_pts)
        # pick next point to make a move
        next_pt = robot.pick_next_point(goal, picking_strategy=Picking_strategy.local_first)
        #ranks_new = np.array([ranker.rank(center, pt, goal) for pt in open_local_pts])
        #ao_local = np.concatenate((open_local_pts, ranks_new), axis=1)
        #next_pt  = robot.pick_next_point(goal=goal)
        #next_pt, _ = robot.pick_max_ranking(ao_local)
        #print (next_pt)
        #print (ao_local)
        
        vision_list.append([open_sights, csight, robot.local_active_open_pts, next_pt])

    plotter.goal(goal)            
    
    for i in range(len(center_points)):
        center = center_points[i]
        open_sights = vision_list[i][0]
        csight = vision_list[i][1]
        open_local_pts = vision_list[i][2]
        next_pt = vision_list[i][3]
        # display vision
        robot.coordinate = center
        plotter.vision(robot.coordinate, robot.vision_range, csight, open_sights)
        
        plotter.points(open_local_pts, ls_aopt)
        if i == 0:
            textpoint = (center[0] + 1 , center[1] -3)
        elif i == 1:
            textpoint = (center[0] -15, center[1] - 5)
        elif i == 2:
            textpoint = (center[0] +2 , center[1] + 2)
        elif i == 3:
            textpoint = (center[0] -17, center[1])

        plt.text(textpoint[0], textpoint[1], "c$_{0}$=({1},{2})".format(i, int(center[0]),int(center[1])))
                    
        # display next point if existing
        if next_pt is not None:
            plotter.point(next_pt, "or")
        plotter.robot(robot)

        plt.axis("equal")
    plt.show()
    save_figure(map_name=map_name, range=robot.vision_range, start=start, goal=goal,\
            picking_strategy=Picking_strategy.local_first, ranking_function=Ranking_function.RHS_RRT_base, plotter=plotter)

if __name__ == '__main__':
    main()