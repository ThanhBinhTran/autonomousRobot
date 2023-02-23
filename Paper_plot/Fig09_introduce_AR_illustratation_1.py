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
    from Robot_theory import *
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
                    [-3,5],
                    [0,3],
                 ])
    
    ob1 = np.array([
                    [4,-0.5],
                    [4,1],
                    [7,2],
                    [6,-2],
                    [4,-0.5],
                 ])

    ob2 = np.array([
                    [9.5,-0.5],
                    [7.5, -3.7],
                    [9.6, -6.8],
                    [11, -5.5],
                    [9.5,-0.5],
                 ])
    ob3 = np.array([
                    [15.72, -10.07],
                    [19.51, -11.32],
                    [25.45,-8.78],
                    [16.24, -7.06]
                 ])
    ob4 = np.array([
                    [21.06, -16.62],
                    [27.09, -11.62],
                    [30.88, -15.41],
                    [25.88, -17.48]
                 ])
    ob5 = np.array([
                    [24.33, -4.13],
                    [25.19, -7.06],
                    [27.6, -7.92],
                    [31.07,-5.25]
                 ])
    ob6 = np.array([
                    [30.10, -2.07],
                    [33.29, 1.12],
                    [32.77, 6.46],
                    [30.10, 1.55],
                    [28.03, 0.09]
                 ])
    ob7 = np.array([
                    [29.76, 7.15],
                    [34.32,8.52],
                    [35.22, 12.04],
                    [30.19, 9.56]
                 ])
    obstacles = []
    obstacles.append(ob)
    obstacles.append(ob1)
    obstacles.append(ob2)
    obstacles.append(ob3)
    obstacles.append(ob4)
    obstacles.append(ob5)
    obstacles.append(ob6)
    obstacles.append(ob7)
    robot_vision = 5
    x_min , x_max = -5.5, 42
    y_min, y_max = -20, 18.5
    space_add = 0
    plotter = Plotter(size=((x_max - x_min + space_add)*0.15, (y_max-y_min)*0.15), title="")
    goal = tuple((25 , 10))

    center_points = np.array(
                        [[0, 0],
                         [21.32, -13.43],
                         [35.35, 0.6]],
                       float)


    # draw obstacles
    plotter.plt.fill(ob[:,0], ob[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob1[:,0], ob1[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob2[:,0], ob2[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob3[:,0], ob3[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob4[:,0], ob4[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob5[:,0], ob5[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob6[:,0], ob6[:,1], color = 'k', alpha = 0.2, hatch='////')
    plotter.plt.fill(ob7[:,0], ob7[:,1], color = 'k', alpha = 0.2, hatch='////')

    plotter.plt.text(ob[-2][0]-1, ob[-2][1]+1, "$OB_0$")
    plotter.plt.text(ob1[2][0], ob1[2][1]-1, "$OB_1$")
    plotter.plt.text(ob2[2][0], ob2[2][1]-1, "$OB_2$")
    plotter.plt.text(ob3[3][0], ob3[3][1]+1, "$OB_3$")
    plotter.plt.text(ob4[-1][0], ob4[-1][1]-1, "$OB_4$")
    plotter.plt.text(ob5[3][0], ob5[3][1]-1, "$OB_5$")
    plotter.plt.text(ob6[4][0], ob6[4][1]+1.7, "$OB_6$")
    plotter.plt.text(ob7[2][0], ob7[2][1], "$OB_7$")



    plotter.point(goal,ls="*r")  
    plotter.plt.text(goal[0]-1.3, goal[1]+0.5, "Goal")  

    #########################################################################
    #
    # illustatation 0
    #
    #########################################################################
    center = center_points[0]
    ###################### start Coordinate ####################################
    C_ID = 0
    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, "$C_{0}$".format(C_ID))
    distcg = point_dist (goal, center)
    plotter.vision_area(goal, distcg, ":", 'r')
    plotter.vision_area(center, robot_vision, "-", 'k')

    plotter.line_segment(line=(center,goal), ls='-k', lw=0.5)
    plotter.plt.text(15.33, 7.9, r'$d_{C_0, g}$')

    boundary_pts = get_boudary_points_theory(center, robot_vision, obstacles, goal)
    print ("boundary_pts", boundary_pts)
    # get arc points in active arc which is limited by parent_arc
    arc_pts, parent_arc = is_inside_active_arc(boundary_pts, robot_vision, center, goal)
    # get local open sights
    open_sights, arc_pts = get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot_vision, obstacles, center)

    # draw arcs for C0
    arc_patches = []
    center_ox = np.add(center, [1,0] )
    for open_sight in open_sights:
        ptA = open_sight[0]
        ptB = open_sight[1]
        theta1radian = unsigned_angle(center, center_ox, ptA)
        theta2radian = unsigned_angle(center, center_ox, ptB)
        theta1 = math.degrees(theta1radian)
        theta2 = math.degrees(theta2radian)
        wedge = patches.Wedge(center, robot_vision, theta1=theta1, theta2=theta2, width=0.2)
        arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor='m', linestyle='solid', edgecolor='m')
        plotter.ax.add_collection(collection)
    
    next_pt = open_sights[0][2]
    plotter.line_segment(line=(center,next_pt), ls='-b', lw=0.5)
    center = next_pt
    ###################### END  Coordinate ####################################


    ###################### start Coordinate ####################################
    C_ID = 1
    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, "$C_{0}$".format(C_ID))
    distcg = point_dist (goal, center)
    plotter.vision_area(goal, distcg, ":", 'r')
    plotter.vision_area(center, robot_vision, "-", 'k')

    plotter.line_segment(line=(center,goal), ls='-k', lw=0.5)
    m_pt_temp = mid_point(goal,center)
    plotter.plt.text(16.97, 4.37, r'$d_{C_1, g}$')

    boundary_pts = get_boudary_points_theory(center, robot_vision, obstacles, goal)
    print ("boundary_pts", boundary_pts)
    # get arc points in active arc which is limited by parent_arc
    arc_pts, parent_arc = is_inside_active_arc(boundary_pts, robot_vision, center, goal)
    # get local open sights
    open_sights, arc_pts = get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot_vision, obstacles, center)
    print (open_sights)
    # draw arcs for C0

    arc_patches = []
    center_ox = np.add(center, [1,0] )
    for open_sight in open_sights:
        ptA = open_sight[0]
        ptB = open_sight[1]
        theta1radian = unsigned_angle(center, center_ox, ptA)
        theta2radian = unsigned_angle(center, center_ox, ptB)
        theta1 = math.degrees(theta1radian)
        theta2 = math.degrees(theta2radian)
        wedge = patches.Wedge(center, robot_vision, theta1=theta1, theta2=theta2, width=0.2)
        arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor='c', linestyle='solid', edgecolor='c')
        plotter.ax.add_collection(collection)
    
    next_pt = open_sights[1][2]
    plotter.line_segment(line=(center,next_pt), ls='-b', lw=0.5)
    center = next_pt
    ###################### END  Coordinate ####################################

    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, "$C_2$")


    #########################################################################
    #
    # illustatation 1
    #
    #########################################################################
    center = center_points[1]
    ###################### start Coordinate ####################################
    C_ID = 0
    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, "$C_{0}$".format(C_ID))
    distcg = point_dist (goal, center)
    plotter.vision_area(goal, distcg, ":", 'r')
    plotter.vision_area(center, robot_vision, "-", 'k')

    plotter.line_segment(line=(center,goal), ls='-k', lw=0.5)
    plotter.plt.text(20.7, 0.76, r'$d_{C_0, g}$')

    boundary_pts = get_boudary_points_theory(center, robot_vision, obstacles, goal)
    print ("boundary_pts", boundary_pts)
    # get arc points in active arc which is limited by parent_arc
    arc_pts, parent_arc = is_inside_active_arc(boundary_pts, robot_vision, center, goal)
    # get local open sights
    open_sights, arc_pts = get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot_vision, obstacles, center)

    # draw arcs for C0
    arc_patches = []
    center_ox = np.add(center, [1,0] )
    for open_sight in open_sights:
        ptA = open_sight[0]
        ptB = open_sight[1]
        theta1radian = unsigned_angle(center, center_ox, ptA)
        theta2radian = unsigned_angle(center, center_ox, ptB)
        theta1 = math.degrees(theta1radian)
        theta2 = math.degrees(theta2radian)
        wedge = patches.Wedge(center, robot_vision, theta1=theta1, theta2=theta2, width=0.2)
        arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor='m', linestyle='solid', edgecolor='m')
        plotter.ax.add_collection(collection)
    
    next_pt = open_sights[0][2]
    plotter.line_segment(line=(center,next_pt), ls='-b', lw=0.5)
    center = next_pt
    ###################### END  Coordinate ####################################


    ###################### start Coordinate ####################################
    C_ID = 1
    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, "$C_{0}$".format(C_ID))
    distcg = point_dist (goal, center)
    plotter.vision_area(goal, distcg, ":", 'r')
    plotter.vision_area(center, robot_vision, "-", 'k')

    plotter.line_segment(line=(center,goal), ls='-k', lw=0.5)
    plotter.plt.text(25.32, -2.43, r'$d_{C_1, g}$')

    boundary_pts = get_boudary_points_theory(center, robot_vision, obstacles, goal)
    print ("boundary_pts", boundary_pts)
    # get arc points in active arc which is limited by parent_arc
    arc_pts, parent_arc = is_inside_active_arc(boundary_pts, robot_vision, center, goal)
    # get local open sights
    open_sights, arc_pts = get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot_vision, obstacles, center)
    print (open_sights)
    # draw arcs for C0

    arc_patches = []
    center_ox = np.add(center, [1,0] )
    for open_sight in open_sights:
        ptA = open_sight[0]
        ptB = open_sight[1]
        theta1radian = unsigned_angle(center, center_ox, ptA)
        theta2radian = unsigned_angle(center, center_ox, ptB)
        theta1 = math.degrees(theta1radian)
        theta2 = math.degrees(theta2radian)
        wedge = patches.Wedge(center, robot_vision, theta1=theta1, theta2=theta2, width=0.2)
        arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor='c', linestyle='solid', edgecolor='c')
        plotter.ax.add_collection(collection)
    
    next_pt = open_sights[0][2]
    plotter.line_segment(line=(center,next_pt), ls='-b', lw=0.5)
    center = next_pt
    ###################### END  Coordinate ####################################

    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, "$C_2$")

    #########################################################################
    #
    # illustatation 2
    #
    #########################################################################
    center = center_points[2]
    ###################### start Coordinate ####################################
    C_ID = 0
    plotter.point(center, ".k")
    plotter.plt.text(center[0]-1.5,center[1]-1, "$C_{0}$".format(C_ID))
    distcg = point_dist (goal, center)
    plotter.vision_area(goal, distcg, ":", 'r')
    plotter.vision_area(center, robot_vision, "-", 'k')

    plotter.line_segment(line=(center,goal), ls='-k', lw=0.5)
    plotter.plt.text(26.26, 5.92, r'$d_{C_0, g}$')

    boundary_pts = get_boudary_points_theory(center, robot_vision, obstacles, goal)
    print ("boundary_pts", boundary_pts)
    # get arc points in active arc which is limited by parent_arc
    arc_pts, parent_arc = is_inside_active_arc(boundary_pts, robot_vision, center, goal)
    # get local open sights
    open_sights, arc_pts = get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot_vision, obstacles, center)

    # draw arcs for C0
    arc_patches = []
    center_ox = np.add(center, [1,0] )
    for open_sight in open_sights:
        ptA = open_sight[0]
        ptB = open_sight[1]
        theta1radian = unsigned_angle(center, center_ox, ptA)
        theta2radian = unsigned_angle(center, center_ox, ptB)
        theta1 = math.degrees(theta1radian)
        theta2 = math.degrees(theta2radian)
        wedge = patches.Wedge(center, robot_vision, theta1=theta1, theta2=theta2, width=0.2)
        arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor='m', linestyle='solid', edgecolor='m')
        plotter.ax.add_collection(collection)
    
    next_pt = open_sights[0][2]
    plotter.line_segment(line=(center,next_pt), ls='-b', lw=0.5)
    center = next_pt
    ###################### END  Coordinate ####################################


    ###################### start Coordinate ####################################
    C_ID = 1
    plotter.point(center, ".k")
    plotter.plt.text(center[0]+0.5,center[1]-1.5, "$C_{0}$".format(C_ID))
    distcg = point_dist (goal, center)
    plotter.vision_area(goal, distcg, ":", 'r')
    plotter.vision_area(center, robot_vision, "-", 'k')

    plotter.line_segment(line=(center,goal), ls='-k', lw=0.5)
    plotter.plt.text(27.64, 9.19, r'$d_{C_0, g}$')

    boundary_pts = get_boudary_points_theory(center, robot_vision, obstacles, goal)
    print ("boundary_pts", boundary_pts)
    # get arc points in active arc which is limited by parent_arc
    arc_pts, parent_arc = is_inside_active_arc(boundary_pts, robot_vision, center, goal)
    # get local open sights
    open_sights, arc_pts = get_open_sights_in_active_arc_theory(arc_pts, parent_arc, robot_vision, obstacles, center)
    print (open_sights)
    # draw arcs for C0

    arc_patches = []
    center_ox = np.add(center, [1,0] )
    for open_sight in open_sights:
        ptA = open_sight[0]
        ptB = open_sight[1]
        theta1radian = unsigned_angle(center, center_ox, ptA)
        theta2radian = unsigned_angle(center, center_ox, ptB)
        theta1 = math.degrees(theta1radian)
        theta2 = math.degrees(theta2radian)
        wedge = patches.Wedge(center, robot_vision, theta1=theta1, theta2=theta2, width=0.2)
        arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor='c', linestyle='solid', edgecolor='c')
        plotter.ax.add_collection(collection)
    
    next_pt = open_sights[0][2]
    plotter.line_segment(line=(center,next_pt), ls='-b', lw=0.5)
    center = next_pt
    ###################### END  Coordinate ####################################

    plotter.point(center, ".k")
    plotter.plt.text(center[0]+0.5,center[1]-1.3, "$C_2$")

    plotter.plt.text(-1.2, -8.71, "$(I)$")
    plotter.plt.text(14.30, -17.15, "$(II)$")
    plotter.plt.text(35.73, -5.44, "$(III)$")
    plotter.plt.text(28.67, 15.72, "$(IV)$")


    center = 23.00, 13.00
    plotter.point(center, ".k")
    plotter.plt.text(center[0]+0.5,center[1]+1.3, "$C_0$")
    plotter.vision_area(center, robot_vision, "-", 'k')  

    plotter.plt.axis("off")   # turns off axes
    plotter.plt.axis("tight")  # gets rid of white border    
    plt.axis("equal")
    plotter.plt.xlim(x_min, x_max + space_add)
    plotter.plt.ylim(y_min, y_max)
    #plt.show()
    plotter.save_figure(fig_name="introduce_AR_illustration",file_extension='.pgf')
    plotter.save_figure(fig_name="introduce_AR_illustration",file_extension='.png')
if __name__ == '__main__':
    main()