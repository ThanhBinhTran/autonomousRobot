"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import sys
import os
# hide/display animation
from Program_config import *
# input from user
from Robot_user_input import menu_Robot

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

from Robot_math_lib import *
from Robot_paths_lib import *
from Robot_sight_lib import *

from Robot_ranking import Ranker, Ranking_function
from Robot_class import Robot
from Robot_base import Picking_strategy, Ranking_type, RobotType
from logging_ranking import Logging_ranking
from Tree import Node
from RRTree_star import RRTree_star


# obstacles class
from Obstacles import *

# get result log for experiment
from Result_log import Result_Log
# plot for animation
from Plotter import Plotter
from datetime import datetime

figure5_ranking_tree= False
figure6_local_sight = False
figure8_step_by_step = False
figure10_connect_visibility_graph = True

def robot_main( start, goal, map_name, world_name, num_iter, 
                robot_vision, robot_type, robot_radius, 
                ranking_type = Ranking_type.Distance_Angle,
                ranking_function =Ranking_function.Angular_similarity,
                picking_strategy= Picking_strategy.local_first,
                sample_size = 2000, log_experiment=False, save_image=False):
    
    # robot ojbect
    robot = Robot(start=start, goal=goal, vision_range= robot_vision, \
                    robot_type=robot_type, robot_radius=robot_radius)
    
    # set alpha and beta only for distance and angle formula
    if ranking_type == Ranking_type.Distance_Angle and ranking_function == Ranking_function.RHS_RRT_base:
        ranking_function = Ranking_function.Angular_similarity
    ranker = Ranker(alpha=0.9, beta= 0.1, ranking_function=ranking_function)

    # declare plotter
    plotter = Plotter(size=(4,4), title="Path Planning for Autonomous Robot{0}".format(map_name))

    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    #obstacles.find_configuration_space(robot.radius)
 
    ''' generate a RRTree_star if ranking type is RRTree_Star_ranking '''
    if ranking_type == Ranking_type.RRTstar:
            # RRT start for ranking scores
        step_size = robot.vision_range
        
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
            RRT_star.build(goal_coordinate=start, robot=robot, plotter=plotter,
                            obstacles=obstacles, ignore_obstacles=True)

            # save ranking tree
            rank_logger.save_tree(RRTree_star=RRT_star, file_name=r_logger_filename)
    else:
        RRT_star = None


    iter_count = 0

    while True:
        iter_count += 1
        robot.update_coordinate(robot.next_coordinate)

        # clean old data
        robot.clear_local()
        print(f"______iteration {iter_count}, robot coordinate {robot.coordinate}")

        # scan to get sights at local
        # closed sights = array of (pointA (x,y), pointB(x,y), angle(angleA, angleB))
        # open sights = array of (pointA (x,y), pointB(x,y), open_point(x, y))
        closed_sights, open_sights = scan_around(robot, obstacles, goal)

        # check whether the robot saw or reach the given goal
        robot.check_goal(goal, closed_sights)
        
        if not robot.saw_goal and not robot.reach_goal:
            # get local active point and its ranking
            robot.get_local_active_open_ranking_points(open_sights=open_sights, ranker=ranker, goal=goal,\
                                                        RRT_star=RRT_star, ranking_type=ranking_type)
            # stack local active open point to global set
            robot.expand_global_open_ranking_points(robot.local_active_open_rank_pts)
            
            # add new active open points to graph_insert
            robot.visibility_graph.add_local_open_points(robot.coordinate, robot.local_active_open_pts)
        
        # pick next point to make a move
        robot.next_point = robot.pick_next_point(goal, picking_strategy=picking_strategy)
        if robot.next_point is not None:
            # find the shortest skeleton path from current position (center) to next point
            if tuple(robot.next_point) == tuple(goal):
                robot.skeleton_path = [robot.coordinate, goal]
            else:
                robot.skeleton_path = robot.visibility_graph.BFS_skeleton_path(robot.coordinate, tuple(robot.next_point))
        else:
            robot.skeleton_path = []
            robot.is_no_way_to_goal(True)

        # record the path and sight
        robot.add_visited_sights(closed_sights, open_sights)

        robot.asp, robot.ls, l_stime, a_time = approximately_shortest_path(robot.skeleton_path, robot.visited_sights, robot.vision_range)
        asp_path_cost = path_cost(robot.asp)
        #robot.asp, robot.ls, l_stime_old, a_time_old = approximately_shortest_path_old(robot.skeleton_path, robot.visited_sights, robot.vision_range)
        #asp_path_cost_old = path_cost(robot.asp)
        # mark visited path
        robot.expand_visited_path(robot.asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            #robot.next_coordinate = motion(robot.coordinate, next_point)  # make smoother path
            robot.next_coordinate = tuple(robot.next_point)
        
        # Run n times for debugging
        if  iter_count == num_iter or robot.finish():
            break
    
    show_grid = False
    hide_axis = True
    if figure8_step_by_step:
        hide_axis = False
        show_grid = False
    plotter.animation(Robot=robot, world_name=world_name, iter_count=iter_count, 
                               obstacles=obstacles, easy_experiment=log_experiment,
                               show_grid=show_grid, hide_axis=hide_axis)
    #plotter.tree_all_nodes(RRTx)
    if ranking_type == Ranking_type.RRTstar and figure5_ranking_tree:
        plotter.tree(RRT_star,color_mode=TreeColor.by_cost)
    
    if figure6_local_sight or figure10_connect_visibility_graph:
        all_centers = robot.visited_sights.all_coordinate()
        for idx, cpt in enumerate(all_centers):
            plotter.text(cpt, f"$C_{idx}$")
    plotter.show()
        
    return robot

if __name__ == '__main__':
    
    # get user input
    menu_result = menu_Robot()
    num_iter = menu_result.n
    #num_iter = 2
    map_name = menu_result.m
    map_name = "_map.csv"
    world_name = menu_result.w
    start = menu_result.sx, menu_result.sy
    start = 50, 50
    goal = menu_result.gx, menu_result.gy
    goal = 90, 90
    robot_radius = menu_result.radius
    robot_vision = menu_result.r
    sample_size = 700
    ranking_type = menu_result.rank_type
    if 'da' in ranking_type:
        ranking_type = Ranking_type.Distance_Angle
    elif 'r' in ranking_type:
        ranking_type = Ranking_type.RRTstar
    

    picking_strategy = menu_result.p
    if 'g' in picking_strategy:
        picking_strategy = Picking_strategy.global_first
    elif 'l' in picking_strategy:
        picking_strategy = Picking_strategy.local_first

    robot_type = RobotType.circle

    ranking_function =Ranking_function.RHS_RRT_base
    if figure5_ranking_tree:
        num_iter = 1
        start = 0, 0
        ranking_type = Ranking_type.RRTstar
        picking_strategy = menu_result.p
    elif figure6_local_sight:
        num_iter = 6
        start = 0, 0
        ranking_type = Ranking_type.RRTstar
        picking_strategy = menu_result.p
    elif figure8_step_by_step:
        #num_iter = 1
        start = 0, 0
        goal = 40, 90
        ranking_type = Ranking_type.RRTstar
        picking_strategy = menu_result.p
    if figure10_connect_visibility_graph:
        map_name = "_paper_connect_visibility_graph.csv"
        start = 24, 5
        goal = 48, 19
        num_iter = 7
        robot_vision = 7
        picking_strategy = Picking_strategy.local_first

    # run robot
    robot_main( start=start, goal=goal, map_name=map_name, world_name=world_name, num_iter=num_iter, 
                robot_vision=robot_vision, robot_type=robot_type, robot_radius=robot_radius, 
                ranking_type = ranking_type, ranking_function =ranking_function, 
                picking_strategy= picking_strategy, sample_size=sample_size)
