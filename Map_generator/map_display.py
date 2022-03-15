import matplotlib.pyplot as plt
import sys
import os
import argparse
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")

try:
    from Robot_map_lib import *
    from Robot_csv_lib import *
except ImportError:
    raise

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description="map (obstacle) generation",
            epilog="Binh Tran (thanhbinh.hcmut@gmail.com",
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-mn", metavar="map name", default="_map_temp.csv", help="map name")
    parser.add_argument("-n", metavar="-number of obstacle(s)", default=1, type= int , help="--number of obstacle(s)")
    args = parser.parse_args()

    map_name = args.mn
    win_size = 100
    
    map = Map()
    obstacles = Obstacles()
    
    plt.axis("equal")
    plt.axis([0, win_size, 0, win_size])
    plt.grid(True)
    
    # read data obstacles
    obstacles.read_csv(map_name)
    # draw map obstacles 
    map.display(plt, map_name, obstacles.obstacles)


    plt.show()