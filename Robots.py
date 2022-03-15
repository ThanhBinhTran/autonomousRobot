"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import Map
from Robot_world_lib import World
from Robot_csv_lib import *
from Robot_goal_lib import *
from Program_config import *
from Robot_knowledge import Robot_knowledge

import argparse

config = Config()

def robot_main(start, goal, map_name, world_name, num_iter, robot_vision, robot_type, robot_radius):
    
    Robot = Robot_knowledge(start, robot_vision, robot_radius)

    # set same window size to capture pictures
    fig, ax = plt.subplots(figsize=(6, 6))
    fig.canvas.set_window_title('Path Planning Problem for an Autonomous Robot')

    
    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(world_name, map_name)

    # find configure space
    # ob1 = find_configure_space(ob)

    # for display information
    iter_count = 0

    print("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))

    while True:
        iter_count += 1
        print("\n_____number of iteration:{0}, current robot coordinate{1}".format(iter_count, Robot.coordinate))
        
        Robot.update_coordinate(Robot.next_coordinate)

        # clean old data
        next_point = []

        # scan to get sights at local
        closed_sights, open_sights = scan_around(Robot, obstacles.data(), goal)

        # check whether the robot saw or reach the given goal
        Robot.check_goal(goal, closed_sights)
        #Robot.show_status()

        if not Robot.saw_goal and not Robot.reach_goal:
            # get local open points
            Robot.get_local_open_points(open_sights)

            # check whether local open points are active
            Robot.get_local_active_open_points()

            ranks_new = []
            # Ranking new active openPts then stack to global set.
            if len(Robot.local_active_open_pts) > 0:
                ranks_new = np.array([ranking(Robot.coordinate, pt, goal) for pt in Robot.local_active_open_pts])

            # stack local active open point to global set
            Robot.global_active_open_pts = store_global_active_points(Robot.global_active_open_pts, Robot.local_active_open_pts, ranks_new)

            # add new active open points to graph_insert
            graph_add_lOpenPts(Robot.visibility_graph, Robot.coordinate, Robot.local_active_open_pts)

            # pick next point to make a move
            next_point, next_pt_idx = Robot.pick_next_point()

            if next_point is not None:
                # find the shortest skeleton path from current position (center) to next point
                skeleton_path = BFS_skeleton_path(Robot.visibility_graph, Robot.coordinate, tuple(next_point))

                # then remove picked point from active global open point
                Robot.global_list_remove(next_pt_idx)
            else:
                print("No way to reach the goal!")
                Robot.is_no_way_to_goal(True)

        else:
            next_point = goal
            # find the shortest path from center to next point
            skeleton_path = [Robot.coordinate, goal]

        # record the path and sight
        Robot.expand_traversal_sights(closed_sights, open_sights)

        if print_traversalSights:
            Robot.print_traversal_sights()

        asp, critical_ls = approximately_shortest_path(skeleton_path, Robot.traversal_sights, Robot.vision_range)

        # mark visited path
        Robot.expand_visited_path(asp)

        # make a move from current position
        if not Robot.no_way_to_goal:
            Robot.next_coordinate = motion(Robot.coordinate, next_point)  # simulate robot

        plt_show_animation(plt, Robot, world_name, map_name, iter_count, obstacles ,mpimg , ax, goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point)

        # Run n times for debugging
        if  iter_count == num_iter:
            break

        # check reaching goal
        if Robot.reach_goal:
            print("Goal!!")
            break
        if Robot.no_way_to_goal:
            print("No way to goal!!")
            break

    Robot.print_visited_path()
    print("Done")

    plt.show()


if __name__ == '__main__':
    
    
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=1)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_map.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=-1.0)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.2)
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
    start = np.array([menu_result.sx, menu_result.sy])
    goal = np.array([menu_result.gx, menu_result.gy])
    robot_radius = menu_result.radius
    robot_vision = menu_result.r 
    # get vision range 
    if robot_vision == -1:  # default == -1
        robot_vision = config.robot_vision
    robot_type=RobotType.circle

    # run robot
    robot_main(start, goal, map_name, world_name, num_iter, robot_vision, robot_type, robot_radius)
