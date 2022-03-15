"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import *
from Robot_world_lib import *
from Robot_csv_lib import *
from Robot_goal_lib import *
from Program_config import *
from Robot_control_panel import *

config = Config()


def main():
    print(__file__ + " start!!")
    
    # get user input
    menu_result = menu()
    map_name = menu_result.m
    obs = read_map_csv(map_name)
    obs_len = len(obs)
    f = open(map_name + ".txt", "w")
    f.write(str(obs_len) + "\n")
    for ob in obs:
        ob_len = len(ob)
        f.write(str(ob_len) + "\n")
        for pt in ob:
            f.write(str(int(pt[0])) + " " + str(int(pt[1])) + "\n")
    f.close()


if __name__ == '__main__':
    main()
