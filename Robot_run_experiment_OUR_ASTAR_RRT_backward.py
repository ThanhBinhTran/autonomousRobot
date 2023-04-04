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
from Robot_class import Robot, Robot_base
from logging_ranking import Logging_ranking
from Tree import Node
from RRTree_star import RRTree_star

from Robot_run import robot_main as robot_check_reachable
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
from RRTree_star import RRTree_star
from Graph import *

enable_compare_AStar = True
enable_compare_RRTStar = True
enable_improve = False
sample_size_each_cell = 100

''' return number of turn actions times '''
def get_path_turn(path):
    turn_num = 0
    if len(path) < 2:
        return turn_num
    for i in range (1,len(path)-1):
        if not inside_line_segment(point=path[i], line_segment=(path[i-1], path[i+1])):
            turn_num +=1
    return turn_num

def RRTStar_initial_build(robot:Robot, plotter:Plotter, obstacles:Obstacles):
    ''' RRT '''
    x, y = robot.coordinate
    # boundary_area = ([x_min, y_min], [x_max, y_max])
    boundary_area =  ((x - robot.vision_range, y - robot.vision_range),
                     ((x + robot.vision_range, y + robot.vision_range)))
    start_node = Node(coords=robot.start, cost=0)            # initial root node, cost to root = 0
    RRT_star = RRTree_star(root=start_node, step_size=3, radius=5, 
                    random_area=boundary_area, sample_size=sample_size_each_cell)
    RRT_star.build(goal_coordinate=robot.goal, plotter=plotter, obstacles=obstacles, robot=robot) 
    return RRT_star

def RRTStar_expand(RRTstar:RRTree_star, robot:Robot, plotter:Plotter, obstacles:Obstacles, goal):
    ''' RRT '''
    x, y = robot.coordinate
    # boundary_area = ([x_min, y_min], [x_max, y_max])
    boundary_area =  ((x - robot.vision_range, y - robot.vision_range),
                     ((x + robot.vision_range, y + robot.vision_range)))
    RRTstar.set_sampling_area(random_area=boundary_area)
    RRTstar.set_sampling_size(sample_size=sample_size_each_cell)
    RRTstar.build(goal_coordinate=goal, plotter=plotter, obstacles=obstacles, robot=robot) 
    return RRTstar

def m_s_g_name_file(star, goal, map_name):
    mn = map_name.replace('.csv','')
    return f"{map_name}_s({star[0]}_{star[1]})_g({goal[0]}_{goal[1]})"

def compare_Astar_RRTstar(robot:Robot, plotter:Plotter, obstacles:Obstacles, RRT_star=RRTree_star,
                          case_count=int, save_image=False, iter_count=int,
                          map_name=str):

    start=robot.skeleton_path[0]
    goal=robot.skeleton_path[-1]
    
    Astar_path =[]
    Astar_path_cost = 0.0
    Astar_time = 0.0

    RRTstar_path =[]
    RRTstar_path_cost = 0.0
    RRTstar_time = 0.0
    
    if enable_compare_AStar: 
        ''' ASTAR '''
        radius = robot.vision_range
        obstacles_Astar = []
        grid_size = 1
        
        visited_area = robot.visibility_graph.get_all_non_leaf()
        for x,y in visited_area:
            for i in range (int(x- radius - 3), int(x + radius + 3)):
                for j in range (int(y- radius - 3),int(y + radius + 3)):
                    pt = (i,j)
                    if not inside_visited_sights(pt, radius +1, robot.visited_sights):
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
        plotter.show_visited_sights(robot.visited_sights, radius)
        plotter.show_map(obstacles=obstacles)
        plotter.robot(robot=robot)
        if robot.next_point is not None:
            plotter.point(robot.next_point, ls_nextpt)
        plotter.path(Astar_path, "-r")
        plotter.goal(robot.goal)
        if save_image:
            m_s_g = m_s_g_name_file(map_name=map_name,star=robot.start, goal=robot.goal)
            plotter.save_figure(f"{m_s_g}_case{case_count}_Astar", file_extension=".png")


    if enable_compare_RRTStar:    
        ''' RRT '''
        new_node, _, _ = RRT_star.add_node_RRTstar(accepted_coordinate=start)
        if new_node is None:
            print ("FAILED TO ADD START_______________________________________________________")

        new_node, _, _ = RRT_star.add_node_RRTstar(accepted_coordinate=goal)
        if new_node is None:
            print ("FAILED TO ADD GOAL_______________________________________________________")

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
        #print ("RRTstar_path: ", RRTstar_path)
        if len(RRTstar_path) > 0:
            plotter.path(RRTstar_path, "-.r")
        else:
            RRTstar_path_cost = -10 # return invalid value
            RRTstar_time = -10
        plotter.goal(robot.goal)
        if save_image:
            m_s_g = m_s_g_name_file(map_name=map_name,star=robot.start, goal=robot.goal)
            plotter.save_figure(f"{m_s_g}_case{case_count}_RRTstar", file_extension=".png")
 
    return (Astar_path, Astar_path_cost, Astar_time), (RRTstar_path, RRTstar_path_cost, RRTstar_time)

def robot_main( start=(0,0), goal=(0, 1), map_name=None, world_name=None, num_iter=1, 
                robot_vision=20, robot_type= Robot_base.RobotType.circle, robot_radius=0.5,
                ranking_type = Robot_base.Open_points_type.Open_Arcs,
                ranking_function =Ranking_function.Angular_similarity,
                picking_strategy= Robot_base.Picking_strategy.neighbor_first,
                sample_size = 2000, log_experiment=True, save_image=True):
    
    # robot ojbect
    robot = Robot(start=start, goal=goal, vision_range= robot_vision, \
                    robot_type=robot_type, robot_radius=robot_radius)
    
    # set alpha and beta only for distance and angle formula
    if ranking_type == Robot_base.Open_points_type.Open_Arcs and ranking_function == Ranking_function.RHS_RRT_base:
        ranking_function = Ranking_function.Angular_similarity
    ranker = Ranker(alpha=0.9, beta= 0.1, ranking_function=ranking_function)

    # declare plotter
    plotter = Plotter(title="Path Planning for Autonomous Robot{0}".format(map_name))
    
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    obstacles.line_segments()
    #obstacles.find_configuration_space(robot.radius)

    if not obstacles.valid_start_goal(start=start, goal=goal):
        return None
    
    csv_head_time = []
    csv_head_cost = []
    csv_head_turn = []
    if platform.system() == 'Linux':
        if enable_improve:
            csv_head_time = ["ASP_improve_time","ASP_improve_time", "Astar_time", "RRTStar_time"]
            csv_head_cost = ["ASP_improve_path_cost","ASP_improve_path_cost", "Astar_path_cost", "RRTStar_path_cost"]
            csv_head_turn = ["ASP_improve_path_turn", "ASP_improve_path_turn","Astar_path_turn", "RRTStar_path_turn"]
        else:
            csv_head_time = ["ASP_time","ASP_time", "Astar_time", "RRTStar_time"]
            csv_head_cost = ["ASP_path_cost","ASP_path_cost", "Astar_path_cost", "RRTStar_path_cost"]
            csv_head_turn = ["ASP_path_turn", "ASP_path_turn","Astar_path_turn", "RRTStar_path_turn"]
    else:
        csv_head_time = ["ASP_time", "Astar_time", "RRTStar_time"]
        csv_head_cost = ["ASP_path_cost", "Astar_path_cost", "RRTStar_path_cost"]
        csv_head_turn = ["ASP_path_turn","Astar_path_turn", "RRTStar_path_turn"]

    result_time = Result_Log(header_csv=csv_head_time)
    result_cost = Result_Log(header_csv=csv_head_cost)
    result_turn = Result_Log(header_csv=csv_head_turn)

    m_s_g = m_s_g_name_file(map_name=map_name,star=robot.start, goal=robot.goal)
    result_time.set_file_name(f"{m_s_g}_ASP_AStar_RRTStar_time_improve{enable_improve}.csv")
    result_cost.set_file_name(f"{m_s_g}_ASP_AStar_RRTStar_path_cost_improve{enable_improve}.csv")
    result_turn.set_file_name(f"{m_s_g}_ASP_AStar_RRTStar_path_turn_improve{enable_improve}.csv")

    ''' generate a RRTree_star if ranking type is RRTree_Star_ranking '''
    if ranking_type == Robot_base.Open_points_type.RRTstar:
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
        robot.update_coordinate(robot.next_coordinate)

        # clean old data
        robot.clear_local()
        print(f"_____iteration {iter_count}, robot coordinate {robot.coordinate}")
        

        # scan to get sights at local
        closed_sights, open_sights = scan_around(robot, obstacles, goal)

        # check whether the robot saw or reach the given goal
        robot.check_goal(goal, closed_sights)
        
        if not robot.saw_goal and not robot.reach_goal:
            # get local active point and its ranking
            robot.get_local_active_open_ranking_points(open_sights=open_sights, ranker=ranker, goal=goal,\
                                                        RRT_star=RRT_star, open_points_type=ranking_type)
            # stack local active open point to global set
            robot.expand_global_open_ranking_points(robot.local_active_open_rank_pts)
            
            # add new active open points to graph_insert
            robot.visibility_graph.add_local_open_points(robot.coordinate, robot.local_active_open_pts)
        
        # record the path and sight
        robot.add_visited_sights(closed_sights, open_sights)

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

        if iter_count == 1:
            RRTstar = RRTStar_initial_build(robot=robot, obstacles=obstacles, plotter=plotter)
        else:
            RRTstar = RRTStar_expand(RRTstar=RRTstar, robot=robot, plotter=plotter, obstacles=obstacles, goal=robot.goal)

        if platform.system() == 'Linux':
            if enable_improve:
                robot.bridge_visibility_graph(robot.coordinate, open_sights)

        #robot.asp, robot.ls, l_stime, a_time = approximately_shortest_path(robot.skeleton_path, robot.visited_sights, robot.vision_range)
        robot.asp, robot.ls, l_stime, a_time = approximately_shortest_path_old(robot.skeleton_path, robot.visited_sights, robot.vision_range)
        asp_path_cost = path_cost(robot.asp)

        if len(robot.skeleton_path)>2:
            print ("back-ward path")
            
            (Astar_path, Astar_path_cost, Astar_time), (RRTstar_path, RRTstar_path_cost, RRTstar_time) =\
                                compare_Astar_RRTstar(robot=robot,plotter=plotter, obstacles=obstacles, 
                                                      save_image=save_image, case_count = case_count,
                                                      iter_count=iter_count, RRT_star=RRTstar,
                                                      map_name=map_name)
            Astar_path_turn = get_path_turn(Astar_path)
            RRTstar_path_turn = get_path_turn(RRTstar_path)
            if save_image:
                # showing the final result (for save image and display as well)
                plotter.animation(Robot=robot, world_name=world_name, iter_count=iter_count, 
                                    obstacles=obstacles, experiment=log_experiment)
                
                # draw some fig for paper
                
                #plotter.show()
                save_name = f"{m_s_g}_case{case_count}_APS"
                if platform.system() == 'Linux' and enable_improve:
                        save_name += "_improve"

                plotter.save_figure(fig_name=save_name, file_extension=".png")

            ASP_path_turn = get_path_turn(robot.asp)
            Astar_path_turn = get_path_turn(Astar_path)
            RRTstar_path_turn = get_path_turn(RRTstar_path)
            
            result_time.add_result([l_stime + a_time, Astar_time, RRTstar_time])
            result_cost.add_result([asp_path_cost, Astar_path_cost, RRTstar_path_cost])
            result_turn.add_result([ASP_path_turn, Astar_path_turn, RRTstar_path_turn])
            
            case_count += 1
        
        
        # mark visited path
        robot.expand_visited_path(robot.asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            #robot.next_coordinate = motion(robot.coordinate, next_point)  # make smoother path
            robot.next_coordinate = tuple(robot.next_point)

        if show_animation and not log_experiment:
            plotter.animation(Robot=robot, world_name=world_name, iter_count=iter_count, 
                                   obstacles=obstacles, experiment=log_experiment)
            #plotter.tree_all_nodes(RRTx)
            if ranking_type == Robot_base.Open_points_type.RRTstar:
                plotter.tree(RRT_star,color_mode=TreeColor.by_cost)
        show_all = False
        if show_all :
            plotter.animation(Robot=robot, world_name=world_name, iter_count=iter_count, 
                                   obstacles=obstacles, experiment=log_experiment)
            plotter.show()

        # Run n times for debugging
        if  iter_count == num_iter or robot.finish():
            break

    if not log_experiment and show_animation: 
        plotter.show()  # show animation for display

    if log_experiment:
        result_time.write_csv()
        result_cost.write_csv()
        result_turn.write_csv()
        print ("\nTo visualize the results, run:\n" +
               f"Python Plotter.py -r {result_repo}\{m_s_g}*.csv")
    return robot

if __name__ == '__main__':
    
    # get user input
    menu_result = menu_Robot()
    num_iter = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    robot_vision = menu_result.r
    sample_size = menu_result.ss
    open_pts_type = menu_result.open_pts_type
    if 'da' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.Open_Arcs
    elif 'r' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.RRTstar

    picking_strategy = menu_result.p
    if 'g' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.global_first
    elif 'n' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.neighbor_first

    map_name = "_MuchMoreFun.csv"

    start = 0, 0
    num_iter = 100  # max
    istart, iend = 10, 101
    jstart, jend = 10, 101
    istep, jstep = 5,5
    for i in range (istart, iend, istep):
        for j in range (jstart, jend, jstep):
            goal = i , j
            # run robot
            #check if robot is reachable or not
            robotA = robot_check_reachable( start=start, goal=goal, map_name=map_name, num_iter=num_iter, robot_vision=robot_vision,  
                                    open_points_type = Robot_base.Open_points_type.Open_Arcs, 
                                    picking_strategy= Robot_base.Picking_strategy.global_first, sample_size=sample_size,
                                    experiment=True,save_image=False)
            if robotA is None:
                continue
            if not robotA.reach_goal:
                continue

            robot_main( start=start, goal=goal, map_name=map_name, world_name=world_name, num_iter=num_iter, robot_vision=robot_vision,
                        ranking_type = open_pts_type, picking_strategy= picking_strategy, sample_size=sample_size)
    print ("DONE!")