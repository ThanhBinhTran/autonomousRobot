"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import platform

from Robot_math_lib import *
from Robot_paths_lib import *
from Robot_sight_lib import *

from Robot_ranking import Ranker, Ranking_function
from Robot_class import Robot
from Robot_base import Picking_strategy, Ranking_type, RobotType
from logging_ranking import Logging_ranking
from Tree import Node
from RRTree_star import RRTree_star

# hide/display animation
from Program_config import *
# obstacles class
from Obstacles import *
# input from user
from Robot_user_input import menu_Robot
# get result log for experiment
from Result_log import Result_Log
# plot for animation
from Plotter import Plotter
import time
from datetime import datetime

from a_star import main as A_star_planner
from RRTree_star_obstacles import RRTree_star
from Graph import *

''' return number of turn actions times '''
def jagged_path(path):
    turn_num = 0
    if len(path) < 2:
        return turn_num
    for i in range (1,len(path)-1):
        if not inside_line_segment(point=path[i], line_segment=(path[i-1], path[i+1])):
            print ("inside")
            turn_num +=1
    return turn_num

def compare_Astar_RRTstar(robot:Robot, plotter:Plotter, obstacles:Obstacles, case_count=int, save_image=False):
    ''' ASTAR '''
    robot.skeleton_path
    robot.vision_range
    obstacles_Astar = []
    grid_size = 1
    goal=robot.skeleton_path[-1]
    start=robot.skeleton_path[0]
    
    visited_area = robot.visibility_graph.get_all_non_leaf()
    x_visited_area = [x for x,y in visited_area]
    y_visited_area = [y for x,y in visited_area]
    x_max, x_min = max(x_visited_area), min(x_visited_area)
    y_max, y_min = max(y_visited_area), min(y_visited_area)
    for x,y in visited_area:
        for i in range (int(x- robot.vision_range - 3), int(x + robot.vision_range + 3)):
            for j in range (int(y- robot.vision_range - 3),int(y + robot.vision_range + 3)):
                pt = (i,j)
                if not inside_visited_sights(pt, robot.vision_range+1, robot.visited_sights):
                    if pt not in obstacles_Astar:
                        obstacles_Astar.append(pt)
    obstacles_x_Astar = []
    obstacles_y_Astar = []
    for ob in obstacles_Astar:
        obstacles_x_Astar.append(ob[0])
        obstacles_y_Astar.append(ob[1])
    Astar_path, Astar_time = A_star_planner(start=start, goal=goal, ox=obstacles_x_Astar, 
        oy=obstacles_y_Astar, robot_radius=robot.radius, plt= plotter.plt, grid_size=grid_size)
    Astar_path_cost = path_cost(Astar_path)
    
    # save image for verification or display purposes
    plotter.clear()
    plotter.show_visited_sights(robot.visited_sights, robot.vision_range)
    plotter.show_map(obstacles=obstacles)
    plotter.robot(robot=robot)
    if robot.next_point is not None:
        plotter.point(robot.next_point, ls_nextpt)
    plotter.path(Astar_path, "-r")
    if save_image:
        plotter.save_figure(f"case{case_count}_Astar", file_extension=".pdf")


    
    ''' RRT '''
    # boundary_area = ([x_min, y_min], [x_max, y_max])
    boundary_area = ((x_min-robot.vision_range, y_min-robot.vision_range),((x_max+robot.vision_range, y_max+robot.vision_range)))
    start_node = Node(coords=robot.start, cost=0)            # initial root node, cost to root = 0
    RRT_star = RRTree_star(root=start_node, step_size=4, radius=5, 
                    #random_area=boundary_area, sample_size=20000)
                    random_area=boundary_area, sample_size=20000)
    
    RRT_star.build(goal_coordinate=robot.skeleton_path[-1], plotter=plotter, obstacles=obstacles, robot=robot) 
    new_node, _, _ = RRT_star.add_node_RRTstar(accepted_coordinate=start)
    if new_node is None:
        print ("FAILED TO ADD _______________________________________________________-")
    all_posible_RRTStar = Graph()
    all_posible_RRTStar_all_nodes = RRT_star.all_nodes()
    for node in all_posible_RRTStar_all_nodes:
        for neighbour in node.neighbours:
            all_posible_RRTStar.graph_create_edge(node.coords, neighbour.coords)
    
    start_time = time.time()
    RRTstar_path = all_posible_RRTStar.BFS_skeleton_path(start=start, goal=goal)
    RRTstar_time = time.time()-start_time    

    RRTstar_path_cost = path_cost(RRTstar_path)

    # save image for verification or display purposes
    plotter.clear()
    plotter.show_visited_sights(robot.visited_sights, robot.vision_range)
    plotter.show_map(obstacles=obstacles)
    plotter.robot(robot=robot)
    if robot.next_point is not None:
        plotter.point(robot.next_point, ls_nextpt)
    plotter.RRTree(tree=RRT_star, neighbour_en=True)
    plotter.path(RRTstar_path, "-.r")
    
    if save_image:
        plotter.save_figure(f"case{case_count}_RRTstar", file_extension=".pdf")
 
    return (Astar_path, Astar_path_cost, Astar_time), (RRTstar_path, RRTstar_path_cost, RRTstar_time)

def robot_main( start, goal, map_name, world_name, num_iter, 
                robot_vision, robot_type, robot_radius, 
                ranking_type = Ranking_type.Distance_Angle,
                ranking_function =Ranking_function.Angular_similarity,
                picking_strategy= Picking_strategy.local_first,
                sample_size = 2000, log_experiment=True, save_image=True):
    
    # robot ojbect
    robot = Robot(start=start, goal=goal, vision_range= robot_vision, \
                    robot_type=robot_type, robot_radius=robot_radius)
    
    # set alpha and beta only for distance and angle formula
    if ranking_type == Ranking_type.Distance_Angle and ranking_function == Ranking_function.RHS_RRT_base:
        ranking_function = Ranking_function.Angular_similarity
    ranker = Ranker(alpha=0.9, beta= 0.1, ranking_function=ranking_function)

    # declare plotter
    plotter = Plotter(title="Path Planning for Autonomous Robot{0}".format(map_name))
    
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    obstacles.line_segments()
    #obstacles.find_configuration_space(robot.radius)
 
    time_stamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

    if platform.system() == 'Linux':
        result_timing = Result_Log(header_csv=["ASP_improve_time", "Astar_time", "RRTStar_time" ])
        result_path_cost = Result_Log(header_csv=["ASP_improve_path_cost", "Astar_path_cost", "RRTStar_path_cost" ])
        result_jagged_path = Result_Log(header_csv=["ASP_improve_jagged_path", "Astar_jagged_path", "RRTStar_jagged_path" ])
    else:
        result_timing = Result_Log(header_csv=["ASP_time", "Astar_time", "RRTStar_time" ])
        result_path_cost = Result_Log(header_csv=["ASP_path_cost", "Astar_path_cost", "RRTStar_path_cost" ])
        result_jagged_path = Result_Log(header_csv=["ASP_jagged_path", "Astar_jagged_path", "RRTStar_jagged_path" ])
    
    result_timing.set_file_name(f"result_ASP_AStar_RRTStar_{time_stamp}.csv")
    result_path_cost.set_file_name(f"result_ASP_AStar_RRTStar_path_cost_{time_stamp}.csv")
    result_jagged_path.set_file_name(f"result_ASP_AStar_RRTStar_jagged_path_{time_stamp}.csv")

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
            RRT_star.build(goal_coordinate=start, plotter=plotter, obstacles=obstacles)

            # save ranking tree
            rank_logger.save_tree(RRTree_star=RRT_star, file_name=r_logger_filename)
    else:
        RRT_star = None


    iter_count = 0
    case_count = 0
    while True:
        iter_count += 1
        print(f"\n__iterations:{iter_count}")

        robot.update_coordinate(robot.next_coordinate)
        
        # clean old data
        robot.clear_local()

        # scan to get sights at local
        closed_sights, open_sights = scan_around(robot, obstacles, goal)
        #print ("\n\nclosed sights : ", closed_sights)
        #print ("\n\nopen sights : ", open_sights)

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
        
        # record the path and sight
        robot.add_visited_sights(closed_sights, open_sights)
        #if platform.system() == 'Linux':
        #    is_pts_center_x, is_pts_center_y, is_pts_x, is_pts_y = robot.bridge_visibility_graph(robot.coordinate, open_sights)

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

        robot.asp, robot.ls, l_stime, a_time = approximately_shortest_path(robot.skeleton_path, robot.visited_sights, robot.vision_range)
        #robot.asp, robot.ls, l_stime, a_time = approximately_shortest_path_old(robot.skeleton_path, robot.visited_sights, robot.vision_range)
        asp_path_cost = path_cost(robot.asp)

        if len(robot.skeleton_path)>2:
            case_count += 1
            Astar_time = 0.0
            RRTstar_time =0.0
            Astar_path_cost = 0.0
            RRTstar_path_cost = 0.0
            Astar_jagged_path = 0.0
            RRTstar_jagged_path = 0.0
            #(Astar_path, Astar_path_cost, Astar_time), (RRTstar_path, RRTstar_path_cost, RRTstar_time) =\
            #                    compare_Astar_RRTstar(robot=robot,plotter=plotter, obstacles=obstacles, 
            #                                          save_image=save_image, case_count = case_count)
            if save_image:
                # showing the final result (for save image and display as well)
                plotter.animation(Robot=robot, world_name=world_name, iter_count=iter_count, 
                                    obstacles=obstacles, easy_experiment=log_experiment)
                
                # draw some fig for paper
                
                #plotter.show()
                if platform.system() == 'Linux':
                    plotter.save_figure(f"case{case_count}_ASP_improve", file_extension=".pdf")
                else:
                    plotter.save_figure(f"case{case_count}_ASP", file_extension=".pdf")

            ASP_jagged_path = jagged_path(robot.asp)
            #Astar_jagged_path = jagged_path(Astar_path)
            #RRTstar_jagged_path = jagged_path(RRTstar_path)
            result_timing.add_result([ l_stime + a_time, Astar_time, RRTstar_time])
            result_path_cost.add_result([ asp_path_cost, Astar_path_cost, RRTstar_path_cost])
            result_jagged_path.add_result([ASP_jagged_path, Astar_jagged_path, RRTstar_jagged_path])
        # mark visited path
        robot.expand_visited_path(robot.asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            #robot.next_coordinate = motion(robot.coordinate, next_point)  # make smoother path
            robot.next_coordinate = tuple(robot.next_point)

        if show_animation and not log_experiment:
            plotter.animation(Robot=robot, world_name=world_name, iter_count=iter_count, 
                                   obstacles=obstacles, easy_experiment=log_experiment)
            #plotter.tree_all_nodes(RRTx)
            if ranking_type == Ranking_type.RRTstar:
                plotter.tree(RRT_star,color_mode=TreeColor.by_cost)

        
        robot.print_infomation()

        # Run n times for debugging
        if  iter_count == num_iter or robot.finish():
            break
    
    if not log_experiment and show_animation: 
        plotter.show()  # show animation for display

    if log_experiment:
        result_timing.write_csv()
        result_path_cost.write_csv()
        result_jagged_path.write_csv()
        print ("\nTo visualize the results, run:\n" +
               f"Python Plotter.py -r result\*{time_stamp}.csv")
    return robot

if __name__ == '__main__':
    
    # get user input
    menu_result = menu_Robot()
    num_iter = menu_result.n
    
    map_name = menu_result.m
    
    world_name = menu_result.w
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    
    robot_radius = menu_result.radius
    robot_vision = menu_result.r
    robot_vision = 20

    num_iter = 98
    num_iter = 50

    map_name = '_MuchMoreFun.csv'
    goal = 40, 60 # for '_MuchMoreFun.csv' never reached goal
    goal = 75, 50 # for '_MuchMoreFun.csv' never reached goal


    sample_size = menu_result.ss
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

    picking_strategy = Picking_strategy.global_first
    robot_type = RobotType.circle

    ranking_function =Ranking_function.RHS_RRT_base

    # run robot
    robot_main( start=start, goal=goal, map_name=map_name, world_name=world_name, num_iter=num_iter, 
                robot_vision=robot_vision, robot_type=robot_type, robot_radius=robot_radius, 
                ranking_type = ranking_type, ranking_function =ranking_function, 
                picking_strategy= picking_strategy, sample_size=sample_size)
