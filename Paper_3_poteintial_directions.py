'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
'''
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
    robotvision = config.robot_vision
    
    # set same window size to capture pictures
    plt.figure(figsize=(6,6))
    
    # get user input
    menu_result = menu()
    runtimes = menu_result.n
    mapname = menu_result.m
    worldname = menu_result.w
    start = np.array([menu_result.sx,menu_result.sy])
    goal = np.array([90,90])
    
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

    r_goal = False
    s_goal = False

    no_way_togoal = False
    
    visible_graph = graph_intiailze()
    visited_path = []

    # for display information
    run_count = 0
    
    print ("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))

    centerpts = np.array([[0, 0],
                       [-5, 90],
                       [50, 50.0],
                       [90, 10.0]
                       ])
    vision_list = []
    plt.figure(figsize=(7,7))
    for center in centerpts:
        csight, osight  = scan_around(center, robotvision, ob, goal)
        osight = np.array(osight)
        open_local_pts = osight[:, 2]    # open_local_pts
        
        ranks_new = np.array([ranking(center, pt, goal) for pt in open_local_pts])
        ao_local = np.concatenate((open_local_pts, ranks_new), axis=1)
        picked_idx, next_pt = pick_next(ao_local)
        vision_list.append([osight, csight, open_local_pts, next_pt])
    # display map
    map_display(plt, mapname, ob)
    plot_goal(plt, goal, r_goal, s_goal)            
    
    for i in range(len(centerpts)):
        center = centerpts[i]
        osight = vision_list[i][0]
        csight = vision_list[i][1]
        open_local_pts = vision_list[i][2]
        next_pt = vision_list[i][3]
        print ("________________")
        print (next_pt)
        # display vision
        plot_vision(plt, center[0], center[1], robotvision, csight, osight)
        
        
        plot_points(plt, open_local_pts, ls_aopt)
        plot_point_text(plt, center, "1r", "center_{0}".format(i) )
                    
        # display next point if existing
        if len(next_pt) > 0:
            plot_point(plt, next_pt, "or")
        plot_robot(plt, center[0], center[1], 0, config)
        
    plt.show()

if __name__ == '__main__':
    #main(robot_type=RobotType.rectangle)
    main(robot_type=RobotType.circle)