"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""

import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import Plot_robot
from Robot_sight_lib import *
from Robot_map_lib import Map
from Robot_csv_lib import *
from Program_config import *
from Robot_ranking import Ranker
from Robot import Robot
from Robot_base import RobotType
import argparse

def robot_main(start, goal, map_name, world_name, num_iter, robot_vision, robot_type, robot_radius):
    
    robot = Robot(start, robot_vision, robot_type, robot_radius)
    ranker = Ranker(alpha=0.9, beta= 0.1)

    # declare potter within window size
    plotter = Plot_robot(title="Path Planning Problem for an Autonomous Robot, map:{0}".format(map_name))
    
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)

    # for display information
    iter_count = 0

    print("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))

    while True:
        iter_count += 1
        print("\n_____number of iteration:{0}, current robot coordinate{1}".format(iter_count, robot.coordinate))
        
        robot.update_coordinate(robot.next_coordinate)

        # clean old data
        next_point = []

        # scan to get sights at local
        closed_sights, open_sights = scan_around(robot, obstacles.data(), goal)
        
        # check whether the robot saw or reach the given goal
        robot.check_goal(goal, closed_sights)
        #Robot.show_status()

        if not robot.saw_goal and not robot.reach_goal:
            # get local open points
            robot.get_local_open_points(open_sights)

            # check whether local open points are active
            robot.get_local_active_open_points()

            ranks_new = []
            # Ranking new active openPts then stack to global set.
            if len(robot.local_active_open_pts) > 0:
                ranks_new = np.array([ranker.rank(robot.coordinate, pt, goal) for pt in robot.local_active_open_pts])

            # stack local active open point to global set
            robot.global_active_open_pts = store_global_active_points(robot.global_active_open_pts, robot.local_active_open_pts, ranks_new)

            # add new active open points to graph_insert
            graph_add_lOpenPts(robot.visibility_graph, robot.coordinate, robot.local_active_open_pts)

            # pick next point to make a move
            next_point, next_pt_idx = robot.pick_next_point(robot.global_active_open_pts)

            if next_point is not None:
                # find the shortest skeleton path from current position (center) to next point
                skeleton_path = BFS_skeleton_path(robot.visibility_graph, robot.coordinate, tuple(next_point))

                # then remove picked point from active global open point
                robot.global_list_remove(next_pt_idx)
            else:
                print("No way to reach the goal!")
                robot.is_no_way_to_goal(True)

        else:
            next_point = goal
            # find the shortest path from center to next point
            skeleton_path = [robot.coordinate, goal]

        # record the path and sight
        robot.expand_traversal_sights(closed_sights, open_sights)

        if print_traversalSights:
            robot.print_traversal_sights()

        asp, critical_ls = approximately_shortest_path(skeleton_path, robot.traversal_sights, robot.vision_range)

        # mark visited path
        robot.expand_visited_path(asp)

        # make a move from current position
        if not robot.no_way_to_goal:
            robot.next_coordinate = motion(robot.coordinate, next_point)  # simulate robot

        plotter.show_animation(robot, world_name, map_name, iter_count, obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point)
        
        # Run n times for debugging
        if  iter_count == num_iter:
            break

        # check reaching goal
        if robot.reach_goal:
            print("Goal!!")
            break
        if robot.no_way_to_goal:
            print("No way to goal!!")
            break

    robot.print_visited_path()
    print("Done")

    plotter.show()


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=1)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_map.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=20.0)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=1)
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=0.0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=0.0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=50.0)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=50.0)
    menu_result = parser.parse_args()

    # get user input
    num_iter = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    # get start point and goal point
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    robot_radius = menu_result.radius
    robot_vision = menu_result.r
    robot_type = RobotType.circle

    # run robot
    robot_main(start, goal, map_name, world_name, num_iter, robot_vision, robot_type, robot_radius)
