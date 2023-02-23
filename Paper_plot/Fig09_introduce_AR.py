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
                    [0,3],
                    [6,4],
                    [3.7,7.5],
                    [1,6],
                    [-3,5]
                 ])
    ob1 = np.array([
                    [4,-0.5],
                    [4,1],
                    [7,2],
                    [6,-2]
                 ])

    robot_vision = 5
    x_min , x_max = -5.2, 16
    y_min, y_max = -5.2, 15
    space_add = 0
    plotter = Plotter(size=((x_max - x_min + space_add)*0.15, (y_max-y_min)*0.15), title="")
    goal = tuple((7 , 5))

    center_points = np.array([[0, 0],
                       [0, 0],
                       [0, 0]],float)

    center = center_points[0]
    # draw obstacles
    plotter.plt.fill(ob[:,0], ob[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob1[:,0], ob1[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.text(ob[-1][0]-1, ob[-1][1]+1, "$OB_0$")
    plotter.plt.text(ob1[-2][0], ob1[-2][1]-1, "$OB_1$")

    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, r"$c$")

    plotter.point(goal,ls="*r")  
    plotter.plt.text(goal[0]+ 0.3, goal[1]+0.5, "g")  

    
    distcg = point_dist (goal, center)
    plotter.vision_area(goal, distcg, "-", 'r')
    plotter.vision_area(center, robot_vision, "-", 'k')

    circle_pt = get_intersections_2circles(center_0=center,radius_0=robot_vision, center_1=goal, radius_1=distcg)
    arcs_ptA = intersection(center[0], center[1], robot_vision, (ob[0],ob[1]))
    arcs_ptB = intersection(center[0], center[1], robot_vision, (ob1[1],ob1[2]))
    arcs_ptC = intersection(center[0], center[1], robot_vision, (ob1[0],ob1[-1]))
    # draw arcs
    arc_patches = []

    center_ox = np.add(center, [1,0] )
    for i in range(2):
        if i == 0:
            ptA = arcs_ptB[0]
            ptB = arcs_ptA[0]
        else:
            
            ptA = circle_pt[0]
            ptB = arcs_ptC[0]
        theta1radian = unsigned_angle(center, center_ox, ptA)
        theta2radian = unsigned_angle(center, center_ox, ptB)
        theta1 = math.degrees(theta1radian)
        theta2 = math.degrees(theta2radian)
        wedge = patches.Wedge(center, robot_vision, theta1=theta1, theta2=theta2, width=0.2)
        print (wedge)
        arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor='m', linestyle='solid', edgecolor='m')
        plotter.ax.add_collection(collection)

    # mid point 
    mpt = mid_point(circle_pt[0], arcs_ptC[0])
    next_pt = intersection(center[0], center[1], robot_vision, (center, mpt))
    
    plotter.point(next_pt[0],".b")
    plotter.plt.text(next_pt[0][0], next_pt[0][1]-1, r"$c^{'}$")
    plotter.line_segment(line=(center,next_pt[0]), ls='-b', lw=0.5)
    plotter.plt.axis("off")   # turns off axes
    plotter.plt.axis("tight")  # gets rid of white border    
    plt.axis("equal")
    plotter.plt.xlim(x_min, x_max + space_add)
    plotter.plt.ylim(y_min, y_max)
    #plt.show()
    plotter.save_figure(fig_name="introduce_AR",file_extension='.pgf')
    #plotter.save_figure(fig_name="introduce_AR",file_extension='.png')
if __name__ == '__main__':
    main()