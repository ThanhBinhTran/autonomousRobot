win_size = 100
from Program_config import ls_map
from Robot_lib import *
import numpy as np

def map_generator(plt, N):
    # displaying the title 
    plt.title("click on plot to generate {0} points of map".format(N))
    plt.axis([0,win_size,0,win_size])

    return plt.ginput(N,show_clicks=True, timeout=-1) # no timeout

def map_display(plt, mapname, ob):
    # displaying the title 
    plt.title("Display map: {0}".format(mapname))
    for ob_part in ob:
        x = [point[0] for point in ob_part]
        y = [point[1] for point in ob_part]
        x.append(ob_part[0][0])
        y.append(ob_part[0][1])
        plt.plot(x, y, ls_map)

def map_display1(plt, mapname, ob):
    # displaying the title 
    plt.title("Display map: {0}".format(mapname))
    x = [point[0] for point in ob]
    y = [point[1] for point in ob]
    plt.plot(x, y, ls_map)
    #plt.plot(ob[:,0], ob[:,1], )
    
def map_serialize(ob_wall, config):
    # divide line into bunch of point 
    map_pts = []
    for i in range(len(ob_wall)-1):
        ptS = ob_wall[i]
        ptE = ob_wall[i+1]
        lenSE = point_dist(ptS, ptE)
        MINSIZE = max(config.robot_length, config.robot_width)
        numparts = int (lenSE/(MINSIZE) )
        vecSE = np.subtract(ptE, ptS)/numparts
        for j in range(numparts):
            ptj = np.add(ptS, np.multiply(vecSE,j) )
            map_pts.append(ptj)
    return map_pts