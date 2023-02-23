"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
from cProfile import label
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
    from Plotter_lib import *
    from Robot_sight_lib import *
    import matplotlib.pyplot as plt
    from matplotlib.path import Path
    import matplotlib.patches as patches
    import numpy as np
except ImportError:
    raise


def main():
    ob = np.array([
                [0, 5.5],
                [1, 2.5],
                [4.75, -0.78],
                [4, 4]
                 ])
                
    robot_vision = 5
    x_min , x_max = -5.5, 16
    y_min, y_max = -6, 14.5
    space_add = 14
    plotter = Plotter(size=((x_max - x_min + space_add)*0.15, (y_max-y_min)*0.15), title="")
    goal = tuple((7 , 5))

    center_points = np.array([[0, 0],
                       [0, 0],
                       [0, 0]],float)
                    
    direction_pt = np.array([ob[2], goal])
    
    # draw obstacles
    plotter.plt.fill(ob[:,0], ob[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.text(ob[0][0], ob[0][1], "OB")
    distcg = point_dist(goal,center_points[0])
    plotter.vision_area(goal, distcg, "-", 'red')
    

    bgpt1 = line_intersection((goal,center_points[0]), (ob[1], ob[2]))
    bug_path = np.array([center_points[0], bgpt1, ob[2], goal])
    plotter.plt.plot(bug_path[:, 0], bug_path[:, 1], label= "Bug2 Algorithm", ls="-", color='b', marker='x')

    for i in range (len(center_points) -1):
        intersection_pt = intersection(center_points[i][0], center_points[i][1], robot_vision, (center_points[i], direction_pt[i]))
        
        distance_0 = point_dist(goal, intersection_pt[0])
        distance_1 = point_dist(goal, intersection_pt[1])
        if distance_0 > distance_1:
            is_pt = intersection_pt[1]
        else:
            is_pt = intersection_pt[0]
        center_points[i+1] = is_pt
    
    for i in range(len(center_points)):

        center = center_points[i]
        color = 'k' if i < len(center_points)-1 else 'r'
        plotter.vision_area(center, robot_vision, ls="-",color=color)
        plotter.point(center, ".k")
        txt = ''

        if i < 5:         txt = "$c_{0}$".format(i)
        elif i == 5:      txt = "$c_m$"
        elif i == 6:      txt = "$c_m+1$"
        
        if i == 0:        txt_pt = center[0] - 2.0, center[1] - 1.5
        elif i == 1:      txt_pt = center[0] + 0.2, center[1] - 1.7
        elif i == 2:      txt_pt = center[0] , center[1] - 1.7 
        elif i == 3:      txt_pt = center[0] + 0.2, center[1] - 1.1
        elif i == 4:      txt_pt = center[0] + 0.2, center[1] -0.8
        elif i == 5:      txt_pt = center[0] + 0.3, center[1] - 0.5
        elif i == 6:      txt_pt = center[0] + 0.5, center[1]
        plt.text(txt_pt[0], txt_pt[1], txt)

    AR_path = np.array([center_points[0], center_points[1], center_points[2], goal])
    plotter.plt.plot(AR_path[:, 0], AR_path[:, 1], label= "[ours] AR Algorithm", ls="-", color='y', marker='*' )

    plotter.point(goal,ls="*r")  
    plt.text(goal[0]-2.3, goal[1]+0.5, "g")  

    plotter.plt.axis("off")   # turns off axes
    plotter.plt.axis("tight")  # gets rid of white border    
    plt.axis("equal")
    plotter.plt.xlim(x_min, x_max + space_add)
    plotter.plt.ylim(y_min, y_max)
    plotter.plt.legend(loc='lower right')
    #plt.show()
    plotter.save_figure(fig_name="AR_BUG_compare",file_extension='.pgf')
    plotter.save_figure(fig_name="AR_BUG_compare",file_extension='.png')
if __name__ == '__main__':
    main()