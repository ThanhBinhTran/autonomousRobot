"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""

# plot for animation
from Plotter import Plotter
# hide/display animation
from RRTree_star import RRTree_star
# get result log for experiment
from Result_log import Result_Log
from Robot_class import Robot, Robot_base
from Robot_paths_lib import *
from Robot_ranking import Ranker, Ranking_function
from Robot_sight_lib import *
# input from user
from Robot_user_input import robot_user_input
from Tree import Node
from logging_ranking import Logging_ranking


def robot_main(start=(0, 0), goal=(0, 1), map_name=None, num_iter=1,
               robot_vision=20, robot_type=Robot_base.RobotType.circle, robot_radius=0.5,
               open_points_type=Robot_base.Open_points_type.Open_Arcs, node_density=6, 
               picking_strategy=Robot_base.Picking_strategy.neighbor_first,
               experiment=False, save_image=False, save_log=False, experiment_title=None):
    # robot ojbect
    robot = Robot(start=start, goal=goal, vision_range=robot_vision, robot_type=robot_type, robot_radius=robot_radius)

    # set alpha and beta only for distance and angle formula
    if open_points_type == Robot_base.Open_points_type.Open_Arcs:
        ranking_function = Ranking_function.Angular_similarity
    else:
        ranking_function = Ranking_function.RHS_RRT_base
    ranker = Ranker(alpha=0.9, beta=0.1, ranking_function=ranking_function)

    # declare plotter
    plotter = Plotter(title="Path Planning for Autonomous Robot{0}".format(map_name))

    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(map_name=map_name)
    obstacles.line_segments()
    # obstacles.find_configuration_space(robot.radius)

    if not obstacles.valid_start_goal(start=start, goal=goal):
        return None

    result_filename = Result_Log.prepare_name(start=start, goal=goal, pick=picking_strategy,
                                       range=robot_vision, open_points_type=open_points_type,
                                       map_name=map_name, experiment_title=experiment_title)
    result_log = Result_Log(header_csv=["asp_time", "asp_path_cost"])
    result_log.set_file_name(result_filename + '.csv')

    l_stime = 0.0
    a_time = 0.0

    ''' generate a RRTree_star if ranking type is RRTree_Star_ranking '''
    if open_points_type == Robot_base.Open_points_type.RRTstar:
        # RRT start for ranking scores
        step_size = robot.vision_range
        
        # find working space boundary
        boundary_area = robot.find_working_space_boundaries(obstacles=obstacles)
        sample_size = robot.calculate_RRTnode_samplenumber(boundary=boundary_area, density=node_density)
        start_node = Node(goal, cost=0)  # initial root node, cost to root = 0

        # logging ranking
        rank_logger = Logging_ranking()
        rrtx_fname = rank_logger.set_logging_name(map_name=map_name, goal=goal, start=start,
                                                  radius=robot_vision, step_size=step_size, sample_size=sample_size)

        if rank_logger.is_existed_log_file(rrtx_fname):
            print("load existed-RRTreeStart_rank: ", rrtx_fname)
            RRT_star = rank_logger.load(rrtx_fname)
        else:
            print("Generating new RRTree star then save to: ", rrtx_fname)
            RRT_star = RRTree_star(root=start_node, step_size=step_size, radius=robot.vision_range,
                                   random_area=boundary_area, sample_size=sample_size)
            RRT_star.build(goal_coordinate=start, plotter=plotter, obstacles=obstacles, ignore_obstacles=True)

            # save ranking tree
            rank_logger.save_tree(rrtree_star=RRT_star, file_name=rrtx_fname)
    else:
        RRT_star = None

    iter_count = 0

    while True:
        iter_count += 1
        robot.update_coordinate(robot.next_coordinate)

        # clean old data
        robot.clear_local()
        # print(f"__iteration {iter_count}, robot coordinate {robot.coordinate}")

        # scan to get sights at local
        # closed sights = array of (pointA (x,y), pointB(x,y), angle(angleA, angleB))
        # open sights = array of (pointA (x,y), pointB(x,y), open_point(x, y))
        closed_sights, open_sights = scan_around(robot, obstacles, goal)

        # check whether the robot saw or reach the given goal
        robot.check_goal(goal, closed_sights)

        if not robot.saw_goal and not robot.reach_goal:
            # get local active point and its ranking
            robot.get_local_active_open_ranking_points(open_sights=open_sights, ranker=ranker, goal=goal,
                                                       RRT_star=RRT_star, open_points_type=open_points_type)
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
                robot.skeleton_path = robot.visibility_graph.BFS_skeleton_path(robot.coordinate,
                                                                               tuple(robot.next_point))
        else:
            robot.skeleton_path = []
            robot.is_no_way_to_goal(True)

        # record the path and sight
        robot.add_visited_sights(closed_sights, open_sights)

        robot.asp, robot.ls, l_stime, a_time = approximately_shortest_path(robot.skeleton_path, robot.visited_sights,
                                                                           robot.vision_range)
        # robot.asp, robot.ls, l_stime_old, a_time_old = approximately_shortest_path_old(robot.skeleton_path,
        # robot.visited_sights, robot.vision_range)
        asp_path_cost = path_cost(robot.asp)

        if len(robot.skeleton_path) > 2:
            result_log.add_result([asp_path_cost, l_stime + a_time])

        # mark visited path
        robot.expand_visited_path(robot.asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            # robot.next_coordinate = motion(robot.coordinate, next_point)  # make smoother path
            robot.next_coordinate = tuple(robot.next_point)

        if show_animation and not experiment:
            plotter.animation(Robot=robot, iter_count=iter_count,
                              obstacles=obstacles, experiment=experiment)
            # plotter.tree_all_nodes(RRTx)
            # if open_points_type == Robot_base.Open_points_type.RRTstar:
            #    plotter.tree(RRT_star,color_mode=TreeColor.by_cost)

        robot.print_infomation()

        # Run n times for debugging
        if iter_count == num_iter or robot.finish():
            break

    if not experiment and show_animation:
        plotter.show()  # show animation for display

    elif experiment:
        if save_log:
            result_log.write_csv()
        if save_image:
            # showing the final result (for save image and display as well)
            plotter.animation(Robot=robot, iter_count=iter_count,
                              obstacles=obstacles, experiment=experiment)
            plotter.save_figure(fig_name=result_filename)

    return robot


if __name__ == '__main__':

    # get user input
    menu_result = robot_user_input()
    num_iter = menu_result.n
    map_name = menu_result.m
    start = menu_result.s
    goal = menu_result.g
    robot_vision = menu_result.r
    node_density = menu_result.d
    open_pts_type = menu_result.open_pts_type
    if 'o' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.Open_Arcs
    elif 'r' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.RRTstar

    picking_strategy = menu_result.p
    if 'g' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.global_first
    elif 'n' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.neighbor_first

    # run robot
    robot_main(start=start, goal=goal, map_name=map_name, num_iter=num_iter, robot_vision=robot_vision,
               open_points_type=open_pts_type, picking_strategy=picking_strategy, node_density=node_density,
               experiment=False, save_log=False, save_image=False)
