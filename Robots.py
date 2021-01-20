"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""
import math
import matplotlib.pyplot as plt
import numpy as np

from Robot_lib import *
from Robot_paths_lib import *
from Robot_draw_lib import *
from Robot_sight_lib import *
from Robot_map_lib import map_display
from Robot_csv_lib import read_map_csv
from Program_config import *
from Robot_control_panel import *

def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


config = Config()

def motion(x, u, dt):
    """
    motion model
    """

    #x[2] += u[1] * dt
    #x[0] += u[0] * math.cos(x[2]) * dt
    #x[1] += u[0] * math.sin(x[2]) * dt
    #x[3] = u[0]
    #x[4] = u[1]
    x[0] += u[0] 
    x[1] += u[1] 
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
            
def saw_goal(center, radius, t_sight, goal):
    return inside_local_true_sight(goal, center, radius, t_sight)

def reached_goal(center, goal, config):
    return point_dist(center, goal) <= config.robot_radius
    
def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")

    run_times = menu()
    
            
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([start_point[0], start_point[1], math.pi / 8.0, 0.0, 0.0])

    [ox_b,oy_b] = read_map_csv(mapname)    
    traversal_path = []
    config.robot_type = robot_type
    trajectory = np.array(x)
    ob = config.ob
    next_pt = np.array([0, 1])
    ao_gobal = [] # active open points [global]
    radius = config.robot_vision

    run_coint = 0
    
    no_way_togoal = False
    print ("\n\n\n____Robot is reaching to goal: {0}____".format(goal))
    while True:
        run_coint += 1
        # scan around robot
        
        center = [x[0], x[1]]
        
        print ("_____Run times:{0}, at {1}".format(run_coint, center))
        
        t_sight = get_true_sight(center[0], center[1], config, ox_b, oy_b)
        osight, csight = get_open_close_sight(plt, center[0], center[1], radius, goal, t_sight)
        r_goal = reached_goal(center, goal, config)
        s_goal = saw_goal(center, radius, t_sight, goal)
        print ("\n__open sights local:", osight)
        if not s_goal:
            osight = np.array(osight)
            open_local_pts = osight[:, 2]    # open_local_pts
            print ("\n__open points local:", open_local_pts)
            if len(open_local_pts) : # new local found

                if len(traversal_path) == 0:
                    # ranks new local open points
                    
                    ranks_new = np.array([ranking(center, pt, goal) for pt in open_local_pts])
                    print ("open_local_pts: ",open_local_pts)
                    print ("ranks_new: ", ranks_new)
                    ao_local = np.concatenate((open_local_pts, ranks_new), axis=1)
                    print ("ao_local: ", ao_local)
                    ao_gobal = np.array(ao_local)
                else:
                    open_local_pts_status = [inside_global_true_sight(pt, radius, traversal_path) for pt in open_local_pts]
                    print ("open_local_pts_status", open_local_pts_status)
                    print ("open_local_pts_status _ FALSE", open_local_pts[open_local_pts_status==False])
                    io_local_pts = open_local_pts[open_local_pts_status]
                    ao_local_pts = open_local_pts[np.logical_not(open_local_pts_status)]
                    if len(ao_local_pts) > 0:
                        ranks_new = np.array([ranking(center, pt, goal) for pt in ao_local_pts])
                        print ("ao_local_pts __+: ", ao_local_pts)
                        print ("ranks_new __+ ", ranks_new)
                        ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                        print ("ao_local __+: ", ao_local)
                        ao_gobal = np.concatenate((ao_gobal, ao_local), axis=0)
                    else:
                        print ("No new open point at this local")
            else:   # local has no direction any more
                print ("local has no direction any more")
                
            print ("ao_gobal ", ao_gobal)
                
            traversal_path.append([center, t_sight, osight, csight])
            
            picked_idx, next_pt = pick_next(ao_gobal)
            
            if picked_idx != -1:
                ao_gobal= np.delete(ao_gobal, picked_idx, axis=0)
            else:
                print ("No way to reach the goal!")
                no_way_togoal = True
                
            # make a move
            #u, predicted_trajectory = dwa_control(x, config, goal, ob)
            if picked_idx != -1:
                #u = unit_vector( np.subtract(next_pt,center))
                u = np.subtract(next_pt,center)
            else:
                u = [1,0]
        else:   #saw goal
            print ("_++_ Saw goal")
            u = np.subtract(goal,center)
        
        x = motion(x, u, config.dt)  # simulate robot
        trajectory = np.vstack((trajectory, x))  # store state history
               
        if print_traversal_path:
            print ("traversal_path:", traversal_path)
            
        if show_animation:

            # clear plot
            plt.cla()

            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            # draw map obstacles 
            map_display(plt, mapname, ox_b, oy_b)
            
            # show_traversal_path
            if show_traversal_path:
                for step in traversal_path:
                    scenter = step[0]
                    st_sight = step[1]
                    sosight = step[2]
                    scsight = step[3]
                    #print (sts_circle)
                    plot_vision(plt, scenter[0], scenter[1], radius, st_sight, sosight, sosight)
                    
            #plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(center[0], center[1], "xr")
            
            #plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(center[0], center[1], x[2], config)
            
            # display goal
            plot_goal(plt, goal, r_goal, s_goal)
            
            plot_vision(plt, center[0], center[1], radius, t_sight, osight, csight)
            

            if show_active_openpt:
                plot_points(plt, ao_gobal, ls_aopt)
                if len(ao_gobal)> 0:
                    plt.plot(ao_gobal[:, 0], ao_gobal[:, 1], ls_aopt)
            if show_inactive_openpt: 
                plot_points(plt, io_points, ls_iopt)
                if len(io_points)> 0:
                    plt.plot(io_points[:, 0], io_points[:, 1], ls_iopt)
            
            # display next point if exist
            if picked_idx != -1:
                plot_point(plt, next_pt, ls_nextpt)

            if show_explored_map:
                plot_explored_map(plt, traversal_path, ls_em)
            #plot_arrow(x[0], x[1], x[2])
            plt.axis("equal") # make sure ox oy axises are same resolution open local points
            plt.grid(True)
            plt.pause(0.0001)
            plt.pause(1)
        
        # Run once for debugging
        if run_times == run_coint:
            break
        
        # check reaching goal
        if r_goal:
            print("Goal!!")
            break
        if no_way_togoal:
            break
            
    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)

    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    #main(robot_type=RobotType.circle)
