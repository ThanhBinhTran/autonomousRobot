'''
AutonomousRobot
This project simulates our path-planning for an autonomous robot that tries to find a way to reach a goal (target)
in certainly environment 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
'''

import sys
import os
import argparse
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

import matplotlib.pyplot as plt
try:
    from Robot_map_lib import *
    from Obstacles import *
    from Plot_base_lib import *
    
except ImportError:
    raise

if __name__ == '__main__':

    parser = argparse.ArgumentParser( description="map (obstacle) generation")
    parser.add_argument("-n", metavar="number of obstacle(s)", default=1, type= int , help="number of obstacle(s)")
    parser.add_argument("-m", metavar="map_name", default="_map_temp.csv", help="map name")
    parser.add_argument("-img", metavar="convert world's image to map", default=None, help="generate map from imange")
    parser.add_argument("-wsize", metavar="window size", default=100, help="set window size")
    args = parser.parse_args()

    obstacle_parts = args.n
    MAX_VERTICES = 10000 #Maximun number of vertices of a obstacle
    map_name = args.m
    from_image = args.img
    win_size = args.wsize

    # variable declaration
    
    map = Map()
    obstacles = Obstacles()
    plotter = Plot_base(size=(7,7), title= "Map generator")
    plotter.set_equal()
    if from_image is not None:
        print ("Generate map from image {0}".format(from_image))
        World().read_map_from_world(from_image)
        map_name = from_image + ".csv"
        obstacles.read_csv(map_name)
        # draw map obstacles 
        map.display(plotter.plt, map_name, obstacles.obstacles)

    else:
        plotter.set_axis(0, win_size, 0, win_size)
        print ("Generate map by user input")
        first = True
        
        for i in range (obstacle_parts):
            # click on plot to generate vertices of obstacle, middle click to turn next obstacle
            # each obstacle content maximun of MAX_VERTICES
            mappoints = map.generate(plotter.plt, i, obstacle_parts, MAX_VERTICES)
            
            x = [int(i[0]) for i in mappoints]
            y = [int(i[1]) for i in mappoints]

            # save map to file
            print ("saving obstacle ({0}/{1}) to file map: {2}".format(i+1, obstacle_parts, map_name))
            obstacles.write_csv(map_name, mappoints, ["x","y"], first)
            if first: 
                first = False

            plotter.clear()

            obstacles.read_csv(map_name)
            
            # draw map obstacles 
            map.display(plotter.plt, map_name, obstacles.obstacles)
    plotter.show()
