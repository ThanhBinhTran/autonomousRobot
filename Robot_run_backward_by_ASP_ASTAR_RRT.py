"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""

from Robot_math_lib import *
from Robot_paths_lib import *
from Robot_sight_lib import *

from Obstacles import *
from Program_config import *
from Robot_ranking import Ranker, Ranking_function
from Robot_class import Robot
from Robot_base import Picking_strategy, Ranking_type, RobotType
from logging_ranking import Logging_ranking
from Tree import Node
#from RRTree_obstacles import RRTree
from RRTree_star_obstacles import RRTree_star
from Robot_user_input import menu_Robot
import time
from a_star import main as A_star_planner
from Result_log import Result_Log
from datetime import datetime

''' plotter lib '''
from Plotter_lib import Plotter

''' map much_more_fun '''
def robot_main( start, goal, map_name, world_name, num_iter, 
                robot_vision, robot_type, robot_radius, 
                ranking_type = Ranking_type.Distance_Angle,
                ranking_function =Ranking_function.Angular_similarity,
                picking_strategy= Picking_strategy.local_first,
                sample_size = 2000, easy_experiment=True, save_image=True):
    
    robot = Robot(start=start, goal=goal, vision_range= robot_vision, \
                    robot_type=robot_type, robot_radius=robot_radius)
    
    # set alpha and beta only for distance and angle formula
    if ranking_type == Ranking_type.Distance_Angle and ranking_function == Ranking_function.RHS_RRT_base:
        ranking_function = Ranking_function.Angular_similarity
    ranker = Ranker(alpha=0.9, beta= 0.1, ranking_function=ranking_function)

    # declare plotter
    title = "Autonomous Robot Path-Planning: {0}".format(map_name)
    plotter = Plotter(title="Path Planning for Autonomous Robot: {0}".format(map_name))
        # result records 
    experiment_results = Result_Log(header_csv=["asp_time","asp_path_cost"
            ,"Astar_time","Astar_path_cost"
            ,"RRTStar_time","RRTStar_path_cost"
            ])

    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    obstacles.line_segments()
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
            RRT_star.build(goal_coordinate=start, plotter=plotter, obstacles=obstacles)

            # save ranking tree
            rank_logger.save_tree(RRTree_star=RRT_star, file_name=r_logger_filename)
    else:
        RRT_star = None


    iter_count = 0

    print("\nRobot is reaching to goal: {0} from start {1}".format(goal, start))
    print("Ranking type: {0}, picking strategy: {1}".format(ranking_type, picking_strategy))
    # Astar _Paths
    Astar_paths = []
    while True:
        iter_count += 1
        robot.update_coordinate(robot.next_coordinate)

        print("\n_number of iterations: {0}, current robot coordinate {1}".format(iter_count, robot.coordinate))

        # clean old data
        next_point = []
        robot.clear_local()

        # scan to get sights at local
        closed_sights, open_sights = scan_around(robot, obstacles, goal)
        
        # check whether the robot saw or reach the given goal
        robot.check_goal(goal, closed_sights)
        
        #robot.show_status()
        if not robot.saw_goal and not robot.reach_goal:
            # get local active point and its ranking
            robot.get_local_active_open_ranking_points(open_sights=open_sights, ranker=ranker, goal=goal,\
                                                        RRT_star=RRT_star, ranking_type=ranking_type)
            # stack local active open point to global set
            robot.expand_global_open_ranking_points(robot.local_active_open_rank_pts)
            
            # add new active open points to graph_insert
            robot.visibility_graph.add_local_open_points(robot.coordinate, robot.local_active_open_pts)
        
        # record the path and sight
        robot.expand_traversal_sights(closed_sights, open_sights)
        center_pts_x = []
        center_pts_y = []
        is_tri_pts_x = []
        is_tri_pts_y = []
        center_pts_x, center_pts_y, is_tri_pts_x, is_tri_pts_y = robot.bridge_visibility_graph(robot.coordinate, open_sights)

        # pick next point to make a move
        next_point = robot.pick_next_point(goal, picking_strategy=picking_strategy)

        if next_point is not None:
            # find the shortest skeleton path from current position (center) to next point
            if tuple(next_point) == tuple(goal):
                skeleton_path = [robot.coordinate, goal]
            else:
                skeleton_path = robot.visibility_graph.BFS_skeleton_path(robot.coordinate, tuple(next_point))
        else:
            skeleton_path = []
            robot.is_no_way_to_goal(True)


        ''' [COMPARASON] apply Astar, RRT to find shortest path in visibility area '''
        Astar_sp = []
        Astar_time =0
        Astar_path_cost = 0
        
        RRT_sp = []
        RRT_time = 0
        RRT_path_cost = 0

        if len(skeleton_path) > 2 and False:
            print ("__backward path:")
            
            ''' ASTAR '''
            obstacles_Astar = []
            grid_size = 1
            visited_area = robot.visibility_graph.get_all_non_leaf()
            x_visited_area = [x for x,y in visited_area]
            y_visited_area = [y for x,y in visited_area]
            x_max, x_min = max(x_visited_area), min(x_visited_area)
            y_max, y_min = max(y_visited_area), min(y_visited_area)
            for x,y in visited_area:
                for i in range (int(x- robot_vision - 3), int(x + robot_vision + 3)):
                    for j in range (int(y- robot_vision - 3),int(y + robot_vision + 3)):
                        pt = (i,j)
                        if not inside_global_true_sight(pt, robot_vision+1, robot.traversal_sights):
                            if pt not in obstacles_Astar:
                                obstacles_Astar.append(pt)
            obstacles_x_Astar = []
            obstacles_y_Astar = []
            for ob in obstacles_Astar:
                obstacles_x_Astar.append(ob[0])
                obstacles_y_Astar.append(ob[1])
            Astar_sp, Astar_time = A_star_planner(start=skeleton_path[0], goal=skeleton_path[-1], ox=obstacles_x_Astar, 
                oy=obstacles_y_Astar, robot_radius=robot.radius, plt= plotter.plt, grid_size=grid_size)
            Astar_path_cost = path_cost(Astar_sp)

            ''' RRT '''
            # boundary_area = ([x_min, y_min], [x_max, y_max])
            boundary_area = ((x_min-robot_vision, y_min-robot_vision),((x_max+robot_vision, y_max+robot_vision)))
            start_node = Node(skeleton_path[0], cost=0)            # initial root node, cost to root = 0
            RRT_star = RRTree_star(root=start_node, step_size=5, radius=5, 
                            random_area=boundary_area, sample_size=20000)
            RRT_start_time = time.time()
            RRT_star.build(goal_coordinate=skeleton_path[-1], plotter=plotter, obstacles=obstacles, robot=robot) 
            RRT_end_time = time.time()     
            RRT_path_cost = RRT_star.total_goal_cost
            RRT_sp = RRT_star.path_to_goal
            
            RRT_time = RRT_end_time-RRT_start_time

        ''' [COMPARASON] approximate shortest path throught bunch of critical edges '''
        asp_start_time = time.time()
        asp, critical_ls = approximately_shortest_path(skeleton_path, robot.traversal_sights, robot.vision_range)
        asp_end_time = time.time()
        asp_time = asp_end_time-asp_start_time
        asp_path_cost = path_cost(asp)
        
        if len(skeleton_path) > 2:
            experiment_results.add_result([ asp_time, asp_path_cost, 
                                            Astar_time, Astar_path_cost,
                                            RRT_time, RRT_path_cost
                                        ])

            plotter.show_animation(robot, world_name, iter_count, obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point, easy_experiment=easy_experiment)
            if len(Astar_sp)>0:
                plotter.path(Astar_sp, "-b")
            if len(RRT_sp) > 0:
                plotter.RRT_path(RRT_sp)
            #if len(Astar_sp)>0:
            save_figure(map_name=map_name, range=iter_count, start=start, goal=goal,\
                    picking_strategy=picking_strategy, ranking_function=ranking_function, plotter=plotter)
        # mark visited path
        robot.expand_visited_path(asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            #robot.next_coordinate = motion(robot.coordinate, next_point)  # simulate robot
            robot.next_coordinate = tuple(next_point)# motion(robot.coordinate, next_point)  # simulate robot

        if show_animation and not easy_experiment:
            plotter.show_animation(robot, world_name, iter_count, obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point, easy_experiment=easy_experiment)
            if len(Astar_sp)>0:
                plotter.path(Astar_sp, "-b")
            if len(RRT_sp) > 0:
                plotter.RRT_path(RRT_sp)
            #if len(Astar_sp)>0:
            #    save_figure(map_name=map_name, range=iter_count, start=start, goal=goal,\
            #        picking_strategy=picking_strategy, ranking_function=ranking_function, plotter=plotter)
            if len(skeleton_path) > 2:
                save_figure(map_name=map_name, range=iter_count, start=start, goal=goal,\
                        picking_strategy=picking_strategy, ranking_function=ranking_function, plotter=plotter)
            if len(center_pts_x) > 0:
                plotter.plt.plot(center_pts_x, center_pts_y, marker = "H", ls="", color = 'red')
                plotter.plt.plot(is_tri_pts_x, is_tri_pts_y, marker = "X", ls="", color = 'red')
            #plotter.plt.pause(1)

            #plotter.tree_all_nodes(RRTx)
            if ranking_type == Ranking_type.RRTstar:
                plotter.tree(RRT_star,color_mode=TreeColor.by_cost)
        
        robot.print_infomation()

        # Run n times for debugging
        if  iter_count == num_iter:
            break
        
        if robot.finish():
            break
    
    if not easy_experiment: # skip printing and showing animation if running easy experiment
        print("Done")
        if show_animation:
            plotter.show()

    elif save_image:
        # showing the final result (for save image and display as well)
        plotter.show_animation(robot, world_name, iter_count, obstacles , goal, closed_sights,\
            open_sights, skeleton_path, asp , critical_ls, next_point, easy_experiment=easy_experiment)
        # draw some fig for paper
        i = 0
        if True:
            for sight in robot.traversal_sights:
                
                plotter.point_text(point=sight[0], ls="ob",text="$C_{0}$".format(i))
                i +=1
        
        #save_figure(map_name=map_name, range=robot.vision_range, start=start, goal=goal,\
        #    picking_strategy=picking_strategy, ranking_function=ranking_function, plotter=plotter)

        # log time and past cost
    if len(experiment_results.results_data) > 0:
        result_file= "result_Astar_ASP_{0}.csv".format(datetime.now().strftime("%Y_%m_%d_%H_%M_%S") )
        experiment_results.write_csv(file_name=result_file)
        print ("saved: {0}".format(result_file))
        print ("\nTo visualize the result, run:\n" +
              "python Robot_run_backward_by_ASP_ASTAR_RRT_plot.py -r {0}".format(result_file))
    return robot

if __name__ == '__main__':
    
    # get user input
    menu_result = menu_Robot()
    num_iter = menu_result.n
    
    map_name = menu_result.m
    
    world_name = menu_result.w
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    #goal = 40, 60 # for '_MuchMoreFun.csv'
    robot_radius = menu_result.radius
    robot_vision = menu_result.r
    robot_vision = 20

    num_iter = 98
    map_name = '_MuchMoreFun.csv'
    goal = 40, 60 # for '_MuchMoreFun.csv'


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
