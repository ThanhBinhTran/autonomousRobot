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
    goal = [10,10]
    robotvision = 2
    center = []
    center.append([1,0.5])
    center.append([2,2.2])
    center.append([4,2.2])
    # read map 
    ob = read_map_csv("_paper_draw_map.csv") # obstacles
    ob = np.array(ob)
    
    start_line = np.array([
                    [0.5, 1.5],
                    [0.5, 4]
                    ])
    
    end_line = np.array([
                    [4.7, 0],
                    [4.7, 2]
                    ])
    spts = intersection(center[0][0], center[0][1], robotvision, start_line)
    epts = intersection(center[-1][0], center[-1][1], robotvision, end_line) 
    
    if inside_ls(spts[0], start_line):
        start = spts[0]
    else:
        start = spts[1]
        
    if inside_ls(epts[0], end_line):
        end = epts[0]
    else:
        end = epts[1]
        
    plt.figure(figsize=(7,7))
    
    skeleton_path = []
    skeleton_path.append(start)
    skeleton_path.append(center[0])
    skeleton_path.append(center[1])
    skeleton_path.append(center[2])
    skeleton_path.append(end)

    csights = []
    osights = []
    for centerpt in center:
        cs, os = scan_around(centerpt, robotvision, ob, goal)
        csights.append(cs)
        osights.append(os)
    
    # draw circle range
    if 1:
        for centerpt in center:
            draw_vision_area(plt, centerpt[0], centerpt[1], robotvision)
            #draw_vision_area(plt, centerpt[0], centerpt[1], robotvision/2)
  
    for ob_part in ob:
        plt.fill(ob_part[:,0], ob_part[:,1], color = 'k', alpha = 0.4, hatch='//')
    
    # text points
    plot_point(plt, start, ".b")
    plot_point(plt, end, ".b")
    for pt in center:
        plot_point(plt, pt, ".b")
    plt.text(center[0][0] +0.1, center[0][1] +0.1, "A")
    plt.text(center[1][0] +0.1, center[1][1] +0.1, "B")
    plt.text(center[2][0] +0.1, center[2][1] +0.1, "C")
        
    plt.text(start[0]+0.1, start[1]+0.1, "S")
    plt.text(end[0]+0.1, end[1]+0.1, "E")

    
    
    traversal_sight = []
    i = 0
    for sight in osights: 
        traversal_sight.append([center[i], csights[i], osights[i]])
        i = i + 1

    asp, critical_ls = approximately_shortest_path(skeleton_path, traversal_sight, robotvision)

    if 0: # draw critical edge
        # draw plot_critical_line_segments
        i = 0
        j = 0
        for ls in critical_ls:
            pt = ls[1]
            plot_line(plt, ls[1:3], "-b")
            if 0:
                if i < 3:
                    plt.text(pt[0], pt[1] +0.1, "CE_a{0}".format(j))
                elif i < 7:
                    plt.text(pt[0] , pt[1] - 0.1, "CE_b{0}".format(j))
                else:
                    plt.text(pt[0] , pt[1] + 0.1, "CE_c{0}".format(j))                
                i += 1
                if i == 2 or i == 6:
                    j = 0
                else:
                    j += 1
                
            
    if 1:    # skeleton path
        plot_lines(plt, skeleton_path, ls="--.b")
    
    if 0: # strange line
        plot_line(plt, (start,end), ls="-..r")
    
    if 0: # collision-free area
        pt = []
        pt.append(start)
        for ls in critical_ls:
            if i == 3:
                pt.append(center[1])
                pt.append(center[2])
                pt.append(end)
                break
            pt.append(ls[1])
            i += 1
        len_ce = len(critical_ls)
        for i in range(len_ce):
            ls = critical_ls[len_ce-i-1]
            pt.append(ls[1])
            if i == 6:
                break
        pt.append(center[0])
        pt = np.array(pt)
        
        plt.fill(pt[:,0], pt[:,1], color = "g", alpha = 0.3, ls="-")
        
    # final approximate shortest path
    if 0:
        plot_lines(plt, asp, "-r")
        i = 0
        for pt in asp:
            plt.text(pt[0], pt[1], "p{0}".format(i))
            i = i + 1
            plot_point(plt, pt, ".k")
    pt3 = asp[3]
    pt4 = asp[4]
    ptA = center[0]
    ptB = center[1]
    print (ptA, ptB, pt3, pt4)
    crosspoint = line_intersection((ptA,ptB), (pt3,pt4))
    if 0: # crosspoint
        plot_point(plt, crosspoint, ".r")
        plt.text(crosspoint[0],crosspoint[1],"M")
        plot_line(plt, (start,crosspoint), ls=":k")
        plot_line(plt, (crosspoint,end), ls=":k")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
