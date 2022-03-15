import sys
import os
import argparse
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import matplotlib.pyplot as plt
try:
    from Robot_map_lib import *
    from Robot_csv_lib import *
except ImportError:
    raise

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
            description="map (obstacle) generation",
            epilog="Binh Tran (thanhbinh.hcmut@gmail.com)",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-mn", metavar="map name", default="_map_temp.csv", help="map name")
    parser.add_argument("-n", metavar="-number of obstacle(s)", default=1, type= int , help="--number of obstacle(s)")
    args = parser.parse_args()

    obstacle_parts = args.n
    MAX_VERTICES = 10000 #Maximun number of vertices of a obstacle
    map_name = args.mn
    
    win_size = 100
    
    map = Map()
    obstacles = Obstacles()

    plt.figure(figsize=(6,6))
    plt.grid(True)
    plt.axis("equal")

    first = True
    
    for i in range (obstacle_parts):
        # click on plot to generate vertices of obstacle, middle click to turn next obstacle
        # each obstacle content maximun of MAX_VERTICES
        mappoints = map.generate(plt, i, obstacle_parts, MAX_VERTICES)
        
        x = [int(i[0]) for i in mappoints]
        y = [int(i[1]) for i in mappoints]

        # save map to file
        print ("save map points to file: {0}".format(map_name))
        obstacles.write_csv(map_name, mappoints, ["x","y"], first)
        if first: 
            first = False

        plt.cla()
        plt.grid(True)

        obstacles.read_csv(map_name)
        
        # draw map obstacles 
        map.display(plt, map_name, obstacles.obstacles)
    
    plt.axis([0, win_size, 0, win_size])
    plt.show()
