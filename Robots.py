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


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


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


config = Config()


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

   
def plot_sight(x, y, sight, cl = "g", alpha = 0.3):
    plt.fill([x, sight[0][0],sight[1][0]], [y, sight[0][1], sight[1][1]], color = cl, alpha = alpha, linestyle = ":")
    
    
def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")

def detect_blind_sight(center, ref_sight, check_sight):
    """ check sight  --> C0, C1
        reference sight -->  R0 R1 """
    true_sight = []
    blind_sight = []
    true_blind = False
    reverse_skip = False
    true_sight.append (ref_sight)
    
    pointC_0 = is_inside_area(check_sight[0], center, ref_sight)
    pointC_1 = is_inside_area(check_sight[1], center, ref_sight)
    print ("=== ref {0} check {1} c1 c2 {2}, {3}".format (ref_sight, check_sight, pointC_0, pointC_1))
    if pointC_0 >= 0:
        plt.plot(check_sight[0][0], check_sight[0][1], "*r")
    if pointC_1 >= 0:
        plt.plot(check_sight[1][0], check_sight[1][1], "*r")


    if pointC_0 >= 0 and pointC_1 >= 0: # whole check_sight are inside ref_sight, true blind
        print ("____1")
        blind_sight.append (check_sight)
        true_blind = True
        reverse_skip = True
    elif pointC_0 == 0 or pointC_1 == 0: # ether C0 or C1 is at the boundary segment
        """ check if C0 C1 coverages ref_sight """
        print ("____2")
        pointR_0 = is_inside_area(ref_sight[0], center, check_sight)
        pointR_1 = is_inside_area(ref_sight[1], center, check_sight)
        if pointR_0 < 0 or pointR_1 < 0: # ref sight is outside of the area of AB
            print ("____2 1")
            true_sight.append(check_sight)
            
    elif pointC_0 > 0 and pointC_1 < 0: # C0 inside, C1 outside
        """ check if R0 is inside area of [R1, C1]
            if R0 is outside then R1 is inside area of [R0, C1] for sure
        """
        print ("____3")
        pointR_0 = is_inside_area(ref_sight[0], center, [ref_sight[1], check_sight[1]])
        if pointR_0 >=0 :  # R1 C0 R0 C1 
            print ("____3    1")
            # divide sight into 3 different parts [R_C0_R]
            RCR_point = line_intersection(ref_sight, [check_sight[0], center])
            CRC_point = line_intersection(check_sight, [ref_sight[0], center])
            plt.plot (RCR_point[0],RCR_point[1], "ob")
            plt.plot (CRC_point[0],CRC_point[1], "ob")
            
            true_sight.append([ref_sight[0], check_sight[1]])
            blind_sight.append([check_sight[0], ref_sight[0]])
        else: # R0 C0 R1 C1 
            print ("____3     2")
            # divide sight into 3 different parts [R_C0_R]
            RCR_point = line_intersection(ref_sight, [check_sight[0], center])
            CRC_point = line_intersection(check_sight, [ref_sight[1], center])
            plt.plot (RCR_point[0],RCR_point[1], "ob")
            plt.plot (CRC_point[0],CRC_point[1], "ob")
            
            true_sight.append([ref_sight[1], check_sight[1]])
            blind_sight.append([check_sight[0], ref_sight[1]])
            
    elif pointC_0 < 0 and pointC_1 > 0: # C1 inside, C0 outside
        print ("____4")
        """ check if R0 is inside area of [R1, C0]
            if R0 is outside then R1 is inside area of [R0, C0] for sure
        """
        pointR_0 = is_inside_area(ref_sight[0], center, [ref_sight[1], check_sight[0]])
        if pointR_0 >=0 :  # R1 C1 R0 C0
            print ("____4      1")
            # divide sight into 3 different parts [R_C0_R]
            RCR_point = line_intersection(ref_sight, [check_sight[1], center])
            CRC_point = line_intersection(check_sight, [ref_sight[0], center])
            plt.plot (RCR_point[0],RCR_point[1], "ob")
            plt.plot (CRC_point[0],CRC_point[1], "ob")
            
            true_sight.append([ref_sight[0], check_sight[0]])
            blind_sight.append([check_sight[1], ref_sight[0]])
        else: # R0 C1 R1 C0
            print ("____4       2")
            # divide sight into 3 different parts [R_C0_R]
            RCR_point = line_intersection(ref_sight, [check_sight[1], center])
            CRC_point = line_intersection(check_sight, [ref_sight[1], center])
            plt.plot (RCR_point[0],RCR_point[1], "ob")
            plt.plot (CRC_point[0],CRC_point[1], "ob")
            
            true_sight.append([ref_sight[1], check_sight[0]])
            blind_sight.append([check_sight[1], ref_sight[1]])
            
    else:   # both C1 and C2 are outside
        print ("____5")
        true_sight.append (check_sight)
        
    return [true_sight, blind_sight, true_blind, reverse_skip]
    
def remove_blind_sight(center, boundary_points):
    true_sight = []
    blind_sight = []
    i = 0 # start with index = 0
    while i < len(boundary_points) -1:
        #print ("NEW I", i, boundary_points)
        j = i + 1
        while j < len(boundary_points) :
            #print ("BEFORE ", i,j,  len(boundary_points),boundary_points)
            [true_sight_A, blind_sight_A, true_blind_A, reverse_skip] = detect_blind_sight(center, boundary_points[i], boundary_points[j])
            blind_sight.extend(blind_sight_A)
            if not reverse_skip:
                [true_sight_B, blind_sight_B, true_blind_B,reverse_skip] = detect_blind_sight(center, boundary_points[j], boundary_points[i])
                blind_sight.extend(blind_sight_B)
                for sight in true_sight_B:
                    if sight not in blind_sight  and sight not in true_sight:
                        true_sight.append(sight)
            for sight in true_sight_A:
                if (sight not in blind_sight) and (sight not in true_sight):
                    true_sight.append(sight)

            

            if true_blind_A: # remove A (lager index of check sight) first if its true blind
                #print ("  remove j")
                boundary_points.pop(j)
                j = j - 1
            elif true_blind_B:
                #print ("  remove I")
                boundary_points.pop(i)
                i = i - 1
            #print ("->AFTER ", i,j,  len(boundary_points), boundary_points) 
            j += 1
        i += 1 
    return [true_sight, blind_sight]

def get_all_boardary_pairs(x, y, config, ox_b, oy_b):
    """ find all boundary pairs among all obstacle line segments and circle """
    boundary_points = []
    for i in range(len(ox_b)-1):
        is_points = intersection(x, y, config.robot_vision, [[ox_b[i], oy_b[i]], [ox_b[i+1], oy_b[i+1]]])
        if len(is_points) > 0:
            # draw intersection points
            #for point in is_points:
            #plt.plot(point[0],point[1], ".g"
        
            """ find boundary pair between a single of line segment and circle """
            boundary_point = []
            for point in is_points:
                b_point = is_inside_line_segment(point, [[ox_b[i],oy_b[i]], [ox_b[i+1], oy_b[i+1]]])
                if b_point is not None:            # found intersection point is inside the line segment
                    boundary_point.append(b_point)
                else:                               # intersection point is not inside the line segment
                    b_point = is_inside_line_segment([ox_b[i],oy_b[i]], is_points)
                    if b_point is not None:            # found intersection point is inside the line segment
                        boundary_point.append(b_point)

                    b_point = is_inside_line_segment([ox_b[i+1],oy_b[i+1]], is_points)
                    if b_point is not None:            # found intersection point is inside the line segment
                        boundary_point.append(b_point)
            if len (boundary_point) > 0:
                #print ("boundary_points ", boundary_point)
                boundary_points.append( [boundary_point[0],boundary_point[1]])
    return boundary_points   
def get_true_sight(x, y, boundary_points):
    true_sight = []
    blind_sight = []
    if len( boundary_points ) >=2:
        [true_sight, blind_sight] = remove_blind_sight([x, y], boundary_points)
    else: # only 1 sight then no collision in the given sight
        true_sight = boundary_points
    return [true_sight, blind_sight]

def draw_true_sight(x, y, true_sight, blind_sight):
    for point in true_sight:
        plot_sight(x, y, point)
        
def draw_vision_area(x, y, config):
    """ draw a circle that limits the vision of robot """ 
    vision = plt.Circle((x, y), config.robot_vision, color="red", linestyle  = "--", fill=False)
    #alpha = 0.3
    #vision = plt.Circle((x, y), config.robot_vision, color="red", linestyle  = "--", alpha = 0.3)
    plt.gcf().gca().add_artist(vision)
    
def plot_vision(x, y, config, ox_b, oy_b):
    
    draw_vision_area(x, y, config)
                
    boundary_points = get_all_boardary_pairs(x, y, config, ox_b, oy_b)
    [true_sight, blind_sight] = get_true_sight(x, y, boundary_points)
    print ("true_sight", true_sight)
    print ("blind_sight", blind_sight)
    draw_true_sight(x, y, true_sight, blind_sight) 
        
def plot_AH_paths(AH_paths, goal):
    for path in AH_paths:
        AH_sp = path[0]       # start point
        AH_nextps = path[1]   # all next points
        blind_ps = path[2]    # all blind points
        goal_appear = path[3] # goad appear
        #print (path)
        # draw AH path segment
        for AH_point in AH_nextps:
            plt.plot((AH_sp[0],AH_point[0]), (AH_sp[1], AH_point[1]), "--or")
        
        # draw blind points segment
        for bp in blind_ps:
            plt.plot((AH_sp[0],bp[0]), (AH_sp[1], bp[1]), "--y")
            
        if goal_appear:
            plt.plot((AH_sp[0],goal[0]), (AH_sp[1], goal[1]), "-r")
            
def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    (gx, gy) = np.random.randint(30, size=(2,1))
    start_point = (50.0, 70.0)
    start_point = (0.0, 0.0)
    start_point = (10.0, 5.0)
    x = np.array([start_point[0], start_point[1], math.pi / 8.0, 0.0, 0.0])
    #x = np.array([55.0, 60.0, math.pi / 8.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])
    goal = np.array([60, 80])
    # input [forward speed, yaw_rate]
    
    o_b = []
    ox_b = [ 0.0, 15.0, 50.0, 10.0, 35.0, 20.0, 60.0,  0.0]
    oy_b = [11.0, 10.0, 20.0, 30.0, 40.0, 50.0, 90.0, 80.0]
   
    direction = []
    boundary_points =  []
    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob
    
    # find AH paths            
    #AH_paths = find_AH_paths(ox_b, oy_b, start_point, goal)
    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            # draw obstacles 
            #plt.plot(ox_b, oy_b, "-xb")
            plt.plot(ox_b, oy_b, "-y")
            
            # draw all AH paths
            #plot_AH_paths(AH_paths, goal)
            
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], config)
            
            plot_vision(x[0], x[1], config, ox_b, oy_b)

            #plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
        
        # for debug
        break
        
        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()


if __name__ == '__main__':
    #main(robot_type=RobotType.rectangle)
    main(robot_type=RobotType.circle)
