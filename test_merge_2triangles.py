'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh
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
import tripy


tri0 = ((0, 0),(0, 4), (4, 0))
tri1 = ((6, 6), (1, 5), (5, 1))

def union_triangles(tri0, tri1):
    '''
    return union of 2 triangles
    '''
    
    ret_result = []
    pts_in_status = [inside_triangle(point, tri0) for point in tri1]
    trIin = [pt_in[0] for pt_in in pts_in_status]
    trIcode = [pt_in[1] for pt_in in pts_in_status]
    incode = np.sum(trIcode)

    # get indexes of points that are inside ref triangle 
    ptin_idx = get_index_true(trIin)
    mutualpoints = len(ptin_idx)

    if mutualpoints == 0:
        # triangle 1 is outside triangle 0
        pts_in_status = [inside_triangle(point, tri1) for point in tri0]
        trIin = [pt_in[0] for pt_in in pts_in_status]
        trIcode = [pt_in[1] for pt_in in pts_in_status]
        # get indexes of points that are inside ref triangle 
        ptin_idx = get_index_true(trIin)
        mutualpoints = len(ptin_idx)
        if mutualpoints == 0:
            print ("2 triangles are outside each others")
            ret_result = tri0, tri1
        else:
            print ("triangle 0 is inside triangle 1")
            ret_result = tri1
        
    elif mutualpoints == 1:
        if incode < 4:
            print ("2 triangles has a mutual vertex")
            ret_result = tri0, tri1
        else:
            # pick point that is inside triangle
            outpts = get_index_false(trIin)
            d = ptin_idx[0]
            e = outpts[0]
            f = outpts[1]
            for i in range(3):
                a = i
                if a == 2:
                    b = 0
                else:
                    b = a + 1
                c = 3 - a - b
                # a b c loop for 0 1 2; 1 2 0, 2 0 1
                newptA = line_across((tri1[d], tri1[e]), (tri0[a],tri0[b]))
                newptB = line_across((tri1[d], tri1[f]), (tri0[a],tri0[b]))
                print ("newpt", newptA, newptB, a, b, c)
                if newptA is not None and newptB is not None:
                    break
            
            polygon = [tri0[c], tri0[a], newptA, tri1[e], tri1[f], newptB, tri0[b]]
            ret_result = tripy.earclip(polygon)

    elif mutualpoints == 2:
        if incode < 4:
            print ("11112 triangles has a mutual vertex")
        else:
            print ("1111a vertex of triangle 1 is in closed triangle 0")
    elif mutualpoints == 3:
        # 3 points of triangle 1 are all inside triangle 0 
        ret_result = tri0
    return ret_result

utriangles = union_triangles(tri0, tri1)



#display union triangles
plot_triangles(plt, (tri0, tri1), "-k")

# display triangle
plot_triangles(plt, utriangles, "--r")
plt.axis("equal") # make sure ox oy axises are same resolution open local points

plt.show()