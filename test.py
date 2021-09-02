'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
'''
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
plt.figure(figsize=(5,5))
ptA = (0, 0)
ptB = (10, 0)
ptC = (6,3)

obstacles=[]
obstacle = (
            (0,0), 
            (10,0), 
            (2.5,2.5),
            (0,12)
            )
obstacles.append(obstacle)
#obstacle = (
#            (12,0), 
#            (22,0), 
#            (8,3),
#            (15,12)
#            )
#obstacles.append(obstacle)
#obstacles = np.array(obstacles)
print ("________________")
print (obstacles)
cspaces = find_configure_space(obstacles, 3.1)

#map_display(plt, "Binh_test_configure_space", obstacles)
print ("________________")
print (cspaces)
print ("________________")
map_display(plt, "Binh_test_configure_space", obstacles)
map_display(plt, "Binh_test_configure_space", cspaces)

plt.grid(True)
plt.show()
