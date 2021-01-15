"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np


from Robot_lib import *
from Robot_paths_lib import *
show_animation = True


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
        self.robot_vision = 10 # the range of input vision
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


config = Config()

def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    (gx, gy) = np.random.randint(30, size=(2,1))
    start_point = (0.0, 0.0)
    x = np.array([start_point[0], start_point[1], math.pi / 8.0, 0.0, 0.0])
    #x = np.array([55.0, 60.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])
    goal = np.array([60, 60])
    # input [forward speed, yaw_rate]
    
    o_b = []
    ox_b = [ 0.0, 15.0, 50.0, 10.0, 35.0, 20.0, 60.0,  0.0]
    oy_b = [11.0, 10.0, 20.0, 30.0, 40.0, 50.0, 90.0, 80.0]
    
    direction = []
    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob
    
    # draw obstacles 
    plt.plot(ox_b, oy_b, ":*b")
            
    plt.grid(True)
    plt.plot(goal[0], goal[1], "xr")
    
    # draw obstacles 
    plt.plot(ox_b, oy_b, "*b")
    
    # draw all positive path
    AH_paths = find_AH_paths(ox_b, oy_b, start_point, goal)
    for path in AH_paths:
        AH_sp = path[0]       # start point
        AH_nextps = path[1]   # all next points
        blind_ps = path[2]    # all blind points
        goal_appear = path[3] # goad appear
        print (path)
        # draw AH path segment
        for AH_point in AH_nextps:
            plt.plot((AH_sp[0],AH_point[0]), (AH_sp[1], AH_point[1]), "--or")

        # draw blind points segment
        for bp in blind_ps:
            plt.plot((AH_sp[0],bp[0]), (AH_sp[1], bp[1]), "--y")
            
        if goal_appear:
            plt.plot((AH_sp[0],goal[0]), (AH_sp[1], goal[1]), "-r")
        plt.pause(2)
        
    print("Done")
    plt.show()


if __name__ == '__main__':
    #main(robot_type=RobotType.rectangle)
    main(robot_type=RobotType.circle)
