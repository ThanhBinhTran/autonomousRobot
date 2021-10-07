"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""
from Robot_lib import *
from Robot_sight_lib import *

"""
Check if robot saw goal
"""
def saw_goal(center, radius, t_sight, goal):
    return inside_local_true_sight(goal, center, radius, t_sight)

"""
Check if robot reached goal
"""
def reached_goal(center, goal, config):
    return point_dist(center, goal) <= config.robot_radius

"""
Check goal status from robot
"""
def check_goal(center, goal, config, radius, t_sight):
    s_goal = False
    r_goal = point_dist(center, goal) <= config.robot_radius
    if not r_goal:
        s_goal = saw_goal(center, radius, t_sight, goal)
    return r_goal, s_goal