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
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.set_window_title('Path Planning Problem for an Autonomous Robot')
    
    # get user input
    menu_result = menu()
    runtimes = menu_result.n
    mapname = menu_result.m
    worldname = menu_result.w
    start = np.array([menu_result.sx,menu_result.sy])
    goal = np.array([menu_result.gx,menu_result.gy])
    
    # current position of robot
    cpos = start
    
    # read world map
    if worldname is not None:
        read_map_from_world(worldname)
        ob = read_map_csv(worldname + ".csv")
    else:
        ob = read_map_csv(mapname)

    # find configure space
    #ob1 = find_configure_space(ob)
    
    # traversal sight to draw visible visited places
    traversal_sight = []

    # active open points [global]
    ao_gobal = [] 

    r_goal = True
    s_goal = True

    no_way_togoal = False
    
    visible_graph = graph_intiailze()
    visited_path = []

    # for display information
    run_count = 0
    
    print ("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))
    
    while True:
        run_count += 1
        center = (cpos[0], cpos[1])
        
        print ("\n_____Run times:{0}, at {1}".format(run_count, center))
        
        # clean old data
        next_pt = []
        
        # scan to get sights at local
        closed_sights, open_sights = scan_around(center, robot_vision, ob, goal)
        
        # check if the robot saw or reach the goal
        r_goal, s_goal = check_goal(center, goal, config, robot_vision, closed_sights)
        
        if not s_goal and not r_goal:
            # get local open points
            open_local_pts = []
            if len(open_sights) > 0:
                open_sights = np.array(open_sights)
                open_local_pts = open_sights[:, 2]    # open_local_pts
                print ("open_local_pts,", open_local_pts)
                for i in range( len(open_local_pts)):
                    open_local_pts[i][0] = approximately_num(open_local_pts[i][0])
                    open_local_pts[i][1] = approximately_num(open_local_pts[i][1])

            # check whether open local points are active 
            if len(open_local_pts) : # new local found
                if len(traversal_sight) == 0:
                    # ranks new local open points
                    ao_local_pts = open_local_pts
                    ranks_new = np.array([ranking(center, pt, goal) for pt in open_local_pts])
                    # active open points at local
                    ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                    # add local to global
                    ao_gobal = np.array(ao_local)
                else:
                    open_local_pts_status = [inside_global_true_sight(pt, robot_vision, traversal_sight) for pt in open_local_pts]
                    ao_local_pts = open_local_pts[np.logical_not(open_local_pts_status)]
                    print ("ao_local_pts,", ao_local_pts)
                    if len(ao_local_pts) > 0:
                        ranks_new = np.array([ranking(center, pt, goal) for pt in ao_local_pts])
                        # active open points at local
                        ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                        # add local to global
                        ao_gobal = np.concatenate((ao_gobal, ao_local), axis=0)
                    else:
                        ao_local_pts = []
                        #print ("No new open point at this local")
                
                graph_insert(visible_graph, center, ao_local_pts)
            
            else:   # there is no direction 
                print ("there is no direction reaching the goal")
                
            # pick next point to make a move
            picked_idx, next_pt = pick_next(ao_gobal)
            
            # find the shortest skeleton path from current position (center) to next point
            skeleton_path = BFS_skeleton_path(visible_graph, tuple(center), tuple(next_pt))

            # remove picked point from active global open point
            if picked_idx != -1:
                ao_gobal= np.delete(ao_gobal, picked_idx, axis=0)
            else:
                print ("No way to reach the goal!")
                no_way_togoal = True
        else:
            next_pt = goal
            # find the shortest path from center to next point
            skeleton_path = [center, goal]
            
        # record the path
        traversal_sight.append([center, closed_sights, open_sights])
        if print_traversalSights:
            print ("traversal_sight:", traversal_sight)
        
        asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robot_vision)
        #asp = remove_validation(asp)
        visited_path.append(asp)
        
        #make a move from current position
        if not no_way_togoal:
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
            if show_world and worldname is not None:
                world_display(plt, mpimg, worldname)
            
            # draw map obstacles 
            if show_map:
                if worldname is not None:
                    map_display(plt, worldname + ".csv", ob)
                else:    
                    map_display(plt, mapname, ob)

            # show_traversal_sight
            if show_traversalSights:
                i = 0
                for local in traversal_sight:
                    lcenter = local[0]  # center of robot at local
                    lc_sight = local[1] # closed sight at local
                    lo_sight = local[2] # open sight at local
                    plot_vision(plt, ax, lcenter[0], lcenter[1], robot_vision, lc_sight, lo_sight)
                    plt.text(lcenter[0] + 1, lcenter[1] + 1, "C({0})".format(i))
                    i = i + 1
           
            
            if show_robot:
                plot_robot(plt, center[0], center[1], 0, config)
            
            if show_goal:
                plot_goal(plt, goal, r_goal, s_goal)            
                        
            # plot robot's vision at local (center)
            plot_vision(plt, ax, center[0], center[1], robot_vision, closed_sights, open_sights)
            
            if show_active_openpt and len(ao_gobal) > 0:
                plot_points(plt, ao_gobal, ls_aopt)
           
            if show_visibilityGraph:
                plot_visibilityGraph(plt, visible_graph, ls_vg)
                
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
        if runtimes == run_count:
            break
        
        # check reaching goal
        if r_goal:
            print("Goal!!")
            break
        if no_way_togoal:
            break
    print ("visited_path:", visited_path)            
    print("Done")

    plt.show()

if __name__ == '__main__':
    #main(robot_type=RobotType.rectangle)
    main(robot_type=RobotType.circle)