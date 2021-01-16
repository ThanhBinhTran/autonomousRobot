import numpy as np
import math
from enum import Enum

rel_tol = 0.0000001

show_animation = True
run_once = True


show_boundary_points = True
show_intersection_line = False
show_true_sight = True
show_close_sight = False
show_open_sight = False
show_ref_sight = False
show_active_openpt = False
show_inactive_openpt = False

print_close_sight = False
print_open_sight = False
print_ref_sight = False

# line style 
ls_is = "-*r"         # intersection
ls_bp = "-r"          # boundary points
ls_ts = "-"           # true sight
cl_ts = "m"           # color true sight
ls_bp = ":g"          # blind sight
ls_map = "-b"         # map 
ls_cs =  "-m"         # close sight
ls_os =  "-g"         # open sight
ls_goal = "*r"        # goal
ls_nextpt = "xr"      # next points
ls_aopt  = "ob"        # active open points
ls_iopt  = "ok"       # inactive open_point
# print out

print_boundary_points = False
print_current_position = False



(gx, gy) = np.random.randint(30, size=(2,1))
start_point = (50, 50) # error for map _map.csv, fix later

start_point = (0, 0)
#goal = np.array([gx, gy])
goal = np.array([60, 50])

g_t = np.random.randint(20, size=(2,2))
g_t = np.array([
                [10, 0],
                [10, 20],
                [30, 40],
                [60, 30],
                [0, 20]
                ])
mapname = "_mapriver.csv" 
mapname = "_map.csv"
#mapname = "_binh.csv"


# Robots configuration
class RobotType(Enum):
    circle = 0
    rectangle = 1

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 10.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle
        self.robot_vision = 40 # the range of input vision
        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        #self.ob = np.array([[-1, -1],
        #                    [0, 2],
        #                    [4.0, 2.0],
        #                    [5.0, 4.0],
        #                    [5.0, 5.0],
        #                    [5.0, 6.0],
        #                    [5.0, 9.0],
        #                    [8.0, 9.0],
        #                    [7.0, 9.0],
        #                    [8.0, 10.0],
        #                    [9.0, 11.0],
        #                    [12.0, 13.0],
        #                    [12.0, 12.0],
        #                    [15.0, 15.0],
        #                    [13.0, 13.0]
        #                    ])
        #
        self.ob = np.random.randint(100, size=(1,2))
    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value
