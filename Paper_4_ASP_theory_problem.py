"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
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
    start = [0,1]
    
    ob_d = np.array([
                [1, 0],
                [1, 2],
                [2, 1],
                [3.5, 2],
                [3,0]
                 ])
    ob_u = np.array([
                [1, 4],
                [0.8, 3],
                [2, 1.5],
                [3, 3],
                [3, 4]
                 ])
    #line segment
    ls  = np.array( [[ob_d[1],ob_u[1]],
                     [ob_d[2],ob_u[2]],
                     [ob_d[3],ob_u[3]]
                    ])
                    
    end = [4,3]
    plt.figure(figsize=(7,7))
    
    plot_point(plt, start, "or")
    plot_point(plt, end, "or")
    
    plt.fill(ob_d[:,0], ob_d[:,1], color = 'k', alpha = 0.4, hatch='//')
    plt.fill(ob_u[:,0], ob_u[:,1], color = 'k', alpha = 0.4, hatch='//')
    plt.axis("equal")
    #plt.grid(True)

    plt.text(start[0] - 0.1, start[1] - 0.2, "A({0},{1})".format(start[0], start[1]))
    plt.text(end[0] - 0.2, end[1] + 0.1, "B({0},{1})".format(end[0], end[1]))
    #plt.text(start[0] + 0.1, start[1] + 0.2, "P0")
    #plt.text(end[0] - 0.2, end[1]- 0.4, "P4")    
    p1 = midpoint(ls[0][0], ls[0][1])
    p2 = midpoint(ls[1][0], ls[1][1])
    p3 = midpoint(ls[2][0], ls[2][1])
    
    config_space = np.array([
                    start, 
                    ob_d[1], ob_d[2], ob_d[3],
                    end,
                    ob_u[3], ob_u[2], ob_u[1],
                    ])
    # draw config space
    if 1:
        plt.fill(config_space[:,0], config_space[:,1], color = "g", alpha = 0.4)
        
    #draw line segment
    if 1:
        #
        i = 0
        for line in ls:
            plot_line(plt, line, ls="-b")
            spt = line [0]
            ept = line [1]
            if i == 0:
                plt.text(ept[0] - 0.2, ept[1] + 0.1, "ce{0}".format(i))
            elif i == 1:
                plt.text(ept[0] - 0.15 , ept[1] + 0.2, "ce{0}".format(i))
            elif i == 2:
                plt.text(ept[0] + 0.1 , ept[1] + 0.1, "ce{0}".format(i))
            i = i + 1
        
    #shortest path step 1
    if 1:

        plt.text(p1[0] , p1[1]+0.1, "P1") 
        plt.text(p2[0] + 0.1 , p2[1]-0.1, "P2") 
        plt.text(p3[0] , p3[1]+0.1, "P3") 
        
        plot_line(plt, (start,p1), ls="--k")
        plot_line(plt, (p1, p2), ls="--k")

        plot_line(plt, (start,ls[0][0]), ls="-r")
        plot_line(plt, (ls[0][0], p2), ls="-r")
        plot_line(plt, (p3, p2), ls="-r")
        plot_line(plt, (p3, end), ls="-r")
        
        plot_point(plt, p1, ".b")
        plot_point(plt, p2, ".b")
        plot_point(plt, p3, ".b")
        plot_point(plt, ls[0][0], ls = "or")
        plt.text(ls[0][0][0]- 0.2, ls[0][0][1]+ 0.085, "P1_new")
 
    #shortest path final step 
    if 0:
        p3_final = line_intersection((ls[1][1],end), ls[2])
        plot_line(plt, (start,ls[0][0]), ls="-r")
        plot_line(plt, (ls[0][0], ls[1][1]), ls="-r")
        plot_line(plt, (ls[1][1], end), ls="-r")
        plot_point(plt, ls[0][0], ls = ".r")
        plot_point(plt, ls[1][1], ls = ".r")
        plot_point(plt, p3_final, ls = ".r")
        plt.text(ls[0][0][0], ls[0][0][1] + 0.1, "P1")
        plt.text(ls[1][1][0] + 0.1, ls[1][1][1] - 0.1, "P2")
        plt.text(p3_final[0], p3_final[1] + 0.1, "P3")
        
    #shortest path
    if 0:
        plot_line(plt, (start,end), ls="-..r")
        plot_line(plt, (start,ls[0][0]), ls="-r")
        plot_line(plt, (ls[0][0], ls[1][1]), ls="-r")
        plot_line(plt, (ls[1][1], end), ls="-r")
    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    #main(robot_type=RobotType.circle)