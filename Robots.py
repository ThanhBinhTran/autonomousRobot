'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh
'''
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



config = Config()

def motion(x, u, dt):
    '''
    motion model
    '''

    #x[2] += u[1] * dt
    #x[0] += u[0] * math.cos(x[2]) * dt
    #x[1] += u[0] * math.sin(x[2]) * dt
    #x[3] = u[0]
    #x[4] = u[1]
    x[0] += u[0] 
    x[1] += u[1] 
    return x


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
    
def check_goal(center, goal, config, radius, t_sight):
    s_goal = False
    r_goal = point_dist(center, goal) <= config.robot_radius
    if not r_goal:
        s_goal = saw_goal(center, radius, t_sight, goal)
    return r_goal, s_goal
    
def main(gx=10.0, gy=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")

    config.robot_type = robot_type
    robotvision = config.robot_vision

    menu_result = menu()
    run_times = menu_result[0]
    mapname = menu_result[1]
    start = menu_result[2] 
    goal = menu_result[3]
    
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([start[0], start[1], math.pi / 8.0, 0.0, 0.0])

    ob = read_map_csv(mapname) # obstacles
    
    traversal_sight = []

    ao_gobal = [] # active open points [global]

    run_count = 0
    r_goal = True
    s_goal = True

    no_way_togoal = False
    
    visible_graph = graph_intiailze()
    visited_path = []
    
    print ("\n____Robot is reaching to goal: {0} from start {1}".format(goal, start))
    while True:
        run_count += 1
        center = [x[0], x[1]]
        
        print ("\n_____Run times:{0}, at {1}".format(run_count, center))
        # clean old data
        next_pt = []
        
        # scan to get sights at local
        tpairs, osight, csight = scan_around(center, robotvision, ob, goal)
        
        # check if the robot saw or reach the goal
        r_goal, s_goal = check_goal(center, goal, config, robotvision, tpairs)
        
        if not s_goal and not r_goal:
            osight = np.array(osight)
            open_local_pts = osight[:, 2]    # open_local_pts
            if len(open_local_pts) : # new local found
                if len(traversal_sight) == 0:
                    # ranks new local open points
                    ao_local_pts = open_local_pts
                    ranks_new = np.array([ranking(center, pt, goal) for pt in open_local_pts])
                    ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                    ao_gobal = np.array(ao_local)
                else:
                    open_local_pts_status = [inside_global_true_sight(pt, robotvision, traversal_sight) for pt in open_local_pts]
                    ao_local_pts = open_local_pts[np.logical_not(open_local_pts_status)]
                    if len(ao_local_pts) > 0:
                        ranks_new = np.array([ranking(center, pt, goal) for pt in ao_local_pts])
                        ao_local = np.concatenate((ao_local_pts, ranks_new), axis=1)
                        ao_gobal = np.concatenate((ao_gobal, ao_local), axis=0)
                    else:
                        ao_local_pts = []
                        #print ("No new open point at this local")
                
                graph_insert(visible_graph, center, ao_local_pts)
            
            else:   # local has no open direction any more
                print ("local has no direction any more")
                
            # pick next point to make a move
            picked_idx, next_pt = pick_next(ao_gobal)
            
            # find the shortest path from center to next point
            sp_fromC = BFS_SP(visible_graph, tuple(center), tuple(next_pt))
            if len(sp_fromC) > 2:
                print ("What i found:", sp_fromC)
            # remove picked point from active global open point
            if picked_idx != -1:
                ao_gobal= np.delete(ao_gobal, picked_idx, axis=0)
            else:
                print ("No way to reach the goal!")
                no_way_togoal = True
 
        else:
            next_pt = goal
            # find the shortest path from center to next point
            sp_fromC = [center, goal]
            
        # record the path
        traversal_sight.append([center, tpairs, osight])
        visited_path.append(sp_fromC)
        
        #make a move 
        u = u = np.subtract(next_pt,center)
        x = motion(x, u, config.dt)  # simulate robot
               
        if print_traversal_sight:
            print ("traversal_sight:", traversal_sight)
            
        if show_animation:

            # clear plot
            plt.cla()

            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            # draw map obstacles 
            map_display(plt, mapname, ob)

            # show_traversal_sight
            if show_traversal_sight:
                for step in traversal_sight:
                    scenter = step[0]
                    st_sight = step[1]
                    sosight = step[2]
                    plot_vision(plt, scenter[0], scenter[1], robotvision, st_sight, sosight, sosight)
                    
            plot_point(plt, center, "Hb")
            plot_goal(plt, goal, r_goal, s_goal)            
            
            #plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(center[0], center[1], x[2], config)
            
            plot_vision(plt, center[0], center[1], robotvision, tpairs, osight, csight)
            

            if show_active_openpt:
                plot_points(plt, ao_gobal, ls_aopt)
            
            # display next point if exist
            if show_next_point:
                if len(next_pt) > 0:
                    plot_point(plt, next_pt, ls_nextpt)
            
            plot_visible_graph(plt, visible_graph, ls_vg)
            plot_paths(plt, visited_path, "-r")

            #plot_arrow(x[0], x[1], x[2])
            plt.axis("equal") # make sure ox oy axises are same resolution open local points
            plt.grid(True)
            #plt.pause(0.0001)
            plt.pause(0.1)
        
        # Run once for debugging
        if run_times == run_count:
            break
        
        # check reaching goal
        if r_goal:
            print("Goal!!")
            break
        if no_way_togoal:
            break
    print ("visited_path:", visited_path)            
    print("Done")

    
    plt.show()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    #main(robot_type=RobotType.circle)
