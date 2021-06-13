'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
'''
import math
import matplotlib.pyplot as plt
import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import map_display
from Robot_csv_lib import read_map_csv
from Program_config import *
from Robot_control_panel import *


def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    
    robotvision = 3
        
    ob = np.array([
                [4, 0],
                [4, 1],
                [3.8, 0.3],
                [3.2, 0.3],
                [2.8, 2.2],
                [2, 1.5],
                [1.5, 2.5],
                [0,2],
                [0,0]
                 ])
    center = [3, 3.5]
    center1 = [4, 4]
    
    start_line = np.array([
                    [3.7, 0],
                    [3.7, 2]
                    ])
    
    end_line = np.array([
                    [0.05, 0],
                    [0.05, 4]
                    ])
    spts = intersection(center[0], center[1], robotvision, start_line)
    epts = intersection(center[0], center[1], robotvision, end_line) 
    
    if inside_ls(spts[0], start_line):
        start = spts[0]
    else:
        start = spts[1]
        
    if inside_ls(epts[0], end_line):
        end = epts[0]
    else:
        end = epts[1]

    plt.figure(figsize=(7,7))
    
    skeleton_path = np.array([start,center,end])
    csights_0, osights_0 = scan_around(center, robotvision, ob, end)
    csights_1, osights_1 = scan_around(center1, robotvision, ob, end)
    
    draw_vision_area(plt, center[0], center[1], robotvision)
    
    # map drawing
    plt.fill(ob[:,0], ob[:,1], color = "k", alpha = 0.3, ls="-")
    
    traversal_sight = []
    traversal_sight.append([center, csights_0, osights_0])
    traversal_sight.append([center1, csights_1, osights_1])
    asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robotvision)

    if 1: # approximate path and critical edge
        # draw plot_critical_line_segments
        i = 0
        for ls in critical_ls:
            pt = ls[1]
            plot_line(plt, ls[1:3], "-b")
            plt.text(pt[0] -0.1, pt[1] - 0.2, "CE_c{0}".format(i))
            i += 1
        # show_approximately_shortest_path
        if 0:
            plot_lines(plt, asp, "-r")
            print ("________________")
            print (asp)
            print ("________________")
            i = 0
            for pt in asp:
                plot_point(plt, pt, ".r")
                plt.text(pt[0] , pt[1] , "P{0}".format(i))
                i = i + 1
    
    if 0: # collision-free area
        pt = []
        pt.append(start)
        i = 0:
        for ls in critical_ls:
            pt.append(ls[1])
        pt.append(end)
        pt.append(center)
        pt = np.array(pt)
        
        plt.fill(pt[:,0], pt[:,1], color = "g", alpha = 0.3, ls="-")
        
    bound_pts = []
    for ls in critical_ls:
        bound_pts.append(ls[1])
 
    # get middle points
    mid_pts = []
    
    for pt in bound_pts:
        mid_pts.append(midpoint(center, pt))
    

        
    plot_line(plt, (start,end), ls="-..r")
    
    plt.text(start[0] + 0.1, start[1] + 0.1, "S")
    plt.text(end[0] + 0.1, end[1] + 0.1, "E")
    plt.text(center[0] + 0.1, center[1] + 0.1, "C")
    
    # draw midpoint 
    if 0:
        i = 0
        for pt in mid_pts:
            plot_point(plt, pt, ls=".k")
            plt.text(pt[0] + 0.1, pt[1] + 0.1, "p{0}".format(i+1))
            i = i + 1
        plt.text(start[0] + 0.1, start[1] + 0.1, "p0")
        plt.text(end[0] + 0.1, end[1] + 0.1, "p{0}".format(i+1))
        
        
    # skeleton path
    plot_lines(plt, skeleton_path, ls="--.b")
    

    # mid path
    if 0:
        plot_line(plt, (start,mid_pts[0]), "-r")
        plot_line(plt, (end,  mid_pts[-1]), "-r")
        for i in range (len(mid_pts)-1):
            plot_line(plt, (mid_pts[i],mid_pts[i+1]), ":r")
       
     
    # new path
    if 0:
        #new mid point
        new_mid_pts = []
        new_mid_pts.append(midpoint(mid_pts[1], mid_pts[0]))
        new_mid_pts.append(midpoint(mid_pts[1], mid_pts[2]))
        p_new = line_intersection(new_mid_pts, (center, bound_pts[1]))
        
        # draw new p1
        plot_point(plt, p_new, ls=".r")
        plt.text(p_new[0]+0.1, p_new[1] , "p2_new")
        mid_pts[1] = p_new
        for i in range (len(mid_pts)-1):
            plot_line(plt, (mid_pts[i],mid_pts[i+1]), "-r")
        
    # final approximate shortest path
    if 0:
        plot_lines(plt, asp, "-r")
        i = 0
        for pt in asp:
            plt.text(pt[0], pt[1], "p{0}".format(i))
            i = i + 1
            plot_point(plt, pt, ".k")
        
    plt.axis("equal")
    #plt.grid(True)
    
    
    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
