import numpy as np

show_animation = True
show_boundary_points = True
show_intersection_line = True
show_true_sight = True
# line style 
ls_is = ":c"        #intersection
ls_bp = "-r"        # boundary points
ls_ts = ":m"        # true sight
ls_bp = ":g"        # blind sight
# print out
print_boundary_points = False
print_current_position = True

run_once = True

(gx, gy) = np.random.randint(30, size=(2,1))

start_point = (61, 60)
#goal = np.array([gx, gy])
goal = np.array([90, 90])

mapname = "_mapriver.csv" 
mapname = "_map.csv"