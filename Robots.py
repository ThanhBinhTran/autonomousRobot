"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import *
from Robot_world_lib import *
from Robot_csv_lib import *
from Robot_goal_lib import *
from Program_config import *
from Robot_control_panel import *

config = Config()


def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")

    # set configuration of robot
    config.robot_type = robot_type
    robot_vision = config.robot_vision

    # set same window size to capture pictures
    plt.figure(figsize=(6, 6))

    # get user input
    menu_result = menu()
    run_times = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    start = np.array([menu_result.sx, menu_result.sy])
    goal = np.array([menu_result.gx, menu_result.gy])

    # current position of robot
    cpos = start

    # read world map then get obstacles information
    if world_name is not None:
        read_map_from_world(world_name)
        ob = read_map_csv(world_name + ".csv")
    else:
        ob = read_map_csv(map_name)

    # find configure space
    # ob1 = find_configure_space(ob)

    # traversal sights to draw visible visited places
    traversal_sights = []

    # global active open points
    g_active_open_pts = []

    r_goal = True  # reach goal status
    s_goal = True  # saw goal status

    no_way_to_goal = False  # there is no way to reach goal

    # visibility Graph which contains information of visited places
    visibility_graph = graph_intiailze()

    # visited path 
    visited_path = []

    # for display information
    run_count = 0

    print("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))

    while True:
        run_count += 1
        center = (cpos[0], cpos[1])

        print("\n_____Run times:{0}, at {1}".format(run_count, center))

        # clean old data
        next_pt = []

        # scan to get sights at local
        closed_sights, open_sights = scan_around(center, robot_vision, ob, goal)

        # check if the robot saw or reach the goal
        r_goal, s_goal = check_goal(center, goal, config, robot_vision, closed_sights)

        if not s_goal and not r_goal:
            # get local open points
            local_open_pts = get_local_open_points(open_sights)

            # check whether local open points are active
            l_active_open_pts = get_active_open_points(local_open_pts, traversal_sights, robot_vision)

            # Ranking new active openPts then stack to global set.
            if len(l_active_open_pts) > 0:
                ranks_new = np.array([ranking(center, pt, goal) for pt in l_active_open_pts])

            # stack local active open point to global set
            g_active_open_pts = store_global_active_points(g_active_open_pts, l_active_open_pts, ranks_new)

            # add new active open points to graph_insert
            graph_add_lOpenPts(visibility_graph, center, l_active_open_pts)

            # pick next point to make a move
            picked_idx, next_pt = pick_next(g_active_open_pts)

            if picked_idx != -1:
                # find the shortest skeleton path from current position (center) to next point
                skeleton_path = BFS_skeleton_path(visibility_graph, tuple(center), tuple(next_pt))

                # then remove picked point from active global open point
                g_active_open_pts = np.delete(g_active_open_pts, picked_idx, axis=0)
            else:
                print("No way to reach the goal!")
                no_way_to_goal = True

        else:
            next_pt = goal
            # find the shortest path from center to next point
            skeleton_path = [center, goal]

        # record the path
        traversal_sights.append([center, closed_sights, open_sights])

        if print_traversalSights:
            print("traversal_sights:", traversal_sights)

        asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sights, robot_vision)

        # recode visited path
        visited_path.append(asp)

        # make a move from current position
        if not no_way_to_goal:
            cpos = motion(cpos, next_pt)  # simulate robot

        if show_animation:

            # clear plot
            plt.cla()

            ##############################################
            # for stopping simulation with the esc key.
            ##############################################
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            ##############################################
            # draw world and map
            ##############################################
            if show_world and world_name is not None:
                dworld_display(plt, mpimg, world_name)

            # draw map obstacles 
            if show_map:
                if world_name is not None:
                    map_display(plt, world_name + ".csv", ob)
                else:
                    map_display(plt, map_name, ob)

            # show_traversalSights
            if show_traversalSights:
                for local in traversal_sights:
                    lcenter = local[0]  # center of robot at local
                    lc_sight = local[1]  # closed sight at local
                    lo_sight = local[2]  # open sight at local
                    plot_vision(plt, lcenter[0], lcenter[1], robot_vision, lc_sight, lo_sight)

            if show_robot:
                plot_robot(plt, center[0], center[1], 0, config)

            if show_goal:
                plot_goal(plt, goal, r_goal, s_goal)

            # plot robot's vision at local (center)
            plot_vision(plt, center[0], center[1], robot_vision, closed_sights, open_sights)

            if show_active_openpt and len(g_active_open_pts) > 0:
                plot_points(plt, g_active_open_pts, ls_aopt)

            if show_visibilityGraph:
                plot_visibilityGraph(plt, visibility_graph, ls_vg)

            if show_visitedPath:
                plot_paths(plt, visited_path, ls_vp, ls_goingp)

            if show_sketelonPath:
                plot_lines(plt, skeleton_path, ls_sp)

            if show_approximately_shortest_path:
                plot_lines(plt, asp, ls_asp)

            if show_critical_line_segments:
                plot_critical_line_segments(plt, critical_ls, ls_cls)

                # display next point if existing
            if show_next_point:
                if len(next_pt) > 0:
                    plot_point(plt, next_pt, ls_nextpt)

            # to set equal make sure x y axises are same resolution 
            plt.axis("equal")
            plt.grid(True)
            plt.pause(1)

        # Run n times for debugging
        if run_times == run_count:
            break

        # check reaching goal
        if r_goal:
            print("Goal!!")
            break
        if no_way_to_goal:
            break
    print("visitedPath:", visited_path)
    print("Done")

    plt.show()


if __name__ == '__main__':
    # main(robot_type=RobotType.rectangle)
    main(robot_type=RobotType.circle)
