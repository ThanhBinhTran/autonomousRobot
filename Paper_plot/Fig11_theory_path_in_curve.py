"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
import random
from re import I
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
import matplotlib.patches as patches
import numpy as np

from torch import true_divide




sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import matplotlib.pyplot as plt
try:
    from Robot_draw_lib import *
    from Robot_sight_lib import *
    import matplotlib.pyplot as plt
    from matplotlib.path import Path
    import matplotlib.patches as patches
    import numpy as np
except ImportError:
    raise


def main():
    codes = [
        Path.MOVETO,
        Path.CURVE4,
        Path.CURVE4,
        Path.CURVE4
    ]
    robot_vision = 5
    plotter = Plot_robot(size=(3.7*1.6,2.0*1.6), title="")
    goal = tuple((27,3))

    center_points = np.array([[0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0]
                       ],float)
                    
    direction_pt = np.array([[ 5, 10],
                             [10,  0],
                            [10.5,-0.5],
                            [14,-4.5],
                            [21,-0.5],
                            [25,0]
                        ])
    

    for i in range (6):
        if i == 4:
            vrange = robot_vision + 2
        else:
            vrange = robot_vision
        intersection_pt = intersection(center_points[i][0], center_points[i][1], vrange, (center_points[i], direction_pt[i]))
        
        distance_0 = point_dist(goal, intersection_pt[0])
        distance_1 = point_dist(goal, intersection_pt[1])
        if distance_0 > distance_1:
            is_pt = intersection_pt[1]
        else:
            is_pt = intersection_pt[0]
        center_points[i+1] = is_pt
        print ("distance ", point_dist(center_points[i],center_points[i+1]))
        print ("center_points[i+1] ", center_points[i+1])
   
        

    control_ptA = np.array([
        [center_points[0][0], center_points[0][1]+3],
        [center_points[1][0]+3, center_points[1][1]],
        [center_points[2][0]+3, center_points[2][1]-3],
        [center_points[3][0]+4, center_points[3][1]],
        [center_points[4][0]+3, center_points[4][1]],
        [center_points[5][0]+2, center_points[5][1]+2],
        [center_points[6][0]+2, center_points[6][1]-4],
    ])
    control_ptB = np.array([
        [center_points[1][0], center_points[1][1]-2],
        [center_points[2][0]-2, center_points[2][1]],
        [center_points[3][0]-2, center_points[3][1]+2],
        [center_points[4][0]-3, center_points[4][1]-2],
        [center_points[5][0], center_points[5][1]-5],
        [center_points[6][0]-4, center_points[6][1]-2],
        [goal[0]+2, goal[1]-4],
    ])
    for i in range (len(center_points)):
        if i == len(center_points) -1:
            endpt = goal
        else:
            endpt = center_points[i+1]
        startpt = center_points[i]
        verts = [
            startpt,   # P0
            control_ptA[i],  # P1
            control_ptB[i],  # P2
            endpt,  # P3
            ]
        path = Path(verts, codes)
        patch = patches.PathPatch(path, facecolor='none', lw=1, edgecolor='b')
        plotter.ax.add_patch(patch)
        #plotter.point(control_ptA[i],'1r')
        #plotter.point(control_ptB[i],'1g')
    
    for i in range(len(center_points)):

        center = center_points[i]
        color = 'k' if i < len(center_points)-1 else 'r'
        plotter.vision_area(center, robot_vision, ls="-",color=color)
        plotter.point(center, ".k")
        txt = ''

        if i == 0:        txt = r"$c$"
        elif i == 1:      txt = r"$c^{'}$"
        elif i == 2:      txt = r"$c^{''}$"
        elif i == 3:      txt = r"$c^{'''}$"
        elif i == 4:      txt = r"$c^{''''}$"
        elif i == 5:      txt = r"$c^{*}$"
        elif i == 6:      txt = r"$c^{*'}$"
        
        if i == 0:        txt_pt = center[0] + 0.5, center[1]
        elif i == 1:      txt_pt = center[0] - 1.2, center[1] + 0.5
        elif i == 2:      txt_pt = center[0] + 0.5, center[1] 
        elif i == 3:      txt_pt = center[0] , center[1] - 1.6
        elif i == 4:      txt_pt = center[0] , center[1] - 1.5
        elif i == 5:      txt_pt = center[0] + 0.3, center[1] - 1.1
        elif i == 6:      txt_pt = center[0] + 0.5, center[1]
        plt.text(txt_pt[0], txt_pt[1], txt)

    plotter.point(goal,ls="*r")  
    plt.text(goal[0]-2.3, goal[1]+0.5, "g")  

    plotter.plt.axis("off")   # turns off axes
    plotter.plt.axis("tight")  # gets rid of white border    
    plt.axis("equal")
    plotter.plt.xlim(-6,31)
    plotter.plt.ylim(-10,10)
    #plt.show()
    plotter.save_figure(fig_name="AR_curve",file_extension='.pgf')
    plotter.save_figure(fig_name="AR_curve",file_extension='.png')
if __name__ == '__main__':
    main()