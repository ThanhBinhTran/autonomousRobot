"""
====================
3D plots as subplots
====================

Demonstrate including 3D plots as subplots.
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D, get_test_data
from matplotlib import cm
import numpy as np
import math
from Robot_paths_lib import *
# set up a figure twice as wide as it is tall
fig = plt.figure()

#===============
#  First subplot
#===============
# set up the axes for the first plot
ax = fig.add_subplot(projection='3d')
map_size = 100
map_resolution = 2
angle_resolution = math.pi/90
# plot a 3D surface like in the example mplot3d/surface3d_demo
angle = np.arange(0.001, math.pi, angle_resolution)
distance = np.arange(1, map_size, map_resolution)
X = angle
Y = distance
X, Y = np.meshgrid(X, Y)
print (X)
print (Y)
X1 = X/math.pi
Y1 = Y/map_size

print (X1)
print (Y1)
Raw_Score = 1/abs(X1) + 1/abs(Y1)
Z =np.log(Raw_Score)
print (Z)

surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
#ax.set_zlim(-1.01, 1.01)
fig.colorbar(surf, shrink=0.5)

ax.set_xlabel('Angle')
ax.set_ylabel('Distance')
ax.set_zlabel('Ranking score')

plt.show()
