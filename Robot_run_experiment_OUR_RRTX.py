"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import os
import Program_config

# hide/display animation
# obstacles class
from Obstacles import *
from RRTree_X import robot_RRTX as robot_RRTX
# get result log for experiment
from Result_log import Result_Log
from Robot_class import Robot_base
from Robot_ranking import Ranking_function
from Robot_run import robot_main as robot_OUR
# input from user
from Robot_user_input import robot_user_input


# plot for animation

experiment_title = "experiment2"

def result_logpath(map_name):
    if map_name is None:
        return None
        
    mn = map_name.replace(".csv", '')
    paths = os.path.join(Program_config.result_repo, experiment_title, mn)
    if not os.path.exists(paths):
        os.makedirs(paths)
    return paths

if __name__ == '__main__':

    # get user input
    menu_result = robot_user_input()
    num_iter = menu_result.n
    map_name = menu_result.m
    start = menu_result.s
    goal = menu_result.g
    robot_vision = menu_result.r

    node_density = menu_result.d
    open_pts_type = menu_result.open_pts_type
    if 'da' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.Open_Arcs
    elif 'r' in open_pts_type:
        open_pts_type = Robot_base.Open_points_type.RRTstar

    picking_strategy = menu_result.p
    if 'g' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.global_first
    elif 'n' in picking_strategy:
        picking_strategy = Robot_base.Picking_strategy.neighbor_first

    ranking_function = Ranking_function.RHS_RRT_base 

    ''' get obstacles data whether from world (if indicated) or map (by default)'''

    obstacles_check = Obstacles()

    start = 0, 0
    map_case = 2

    if map_case == 0:
        map_name = '_map_forest.csv' # 500x500 size
        node_density = 50
        istart, iend = 20, 500 
        jstart, jend = 20, 500
        step = 50
    elif map_case == 1:
        map_name = '_map_deadend.csv' # 100x100 size
        node_density = 5
        istart, iend = 20, 100 
        jstart, jend = 20, 100
        step = 10
    elif map_case == 2:
        map_name = '_map_bugtrap.csv' # 200x200 size
        node_density = 5
        istart, iend = 20, 200 
        jstart, jend = 20, 200
        step = 20
    elif map_case == 3:
        map_name = '_map_blocks.csv' # 300X 350 size
        node_density = 5
        istart, iend = 20, 100 
        jstart, jend = 20, 100
        step = 10
    else:
        node_density = 5
        istart, iend = 0, 0 
        jstart, jend = 0, 0
        step = 0
    obstacles_check.read(map_name=map_name)
    obstacles_check.line_segments()

    csv_head = ["start", "goal", "range",
                "[our]_reached_goal_global_pick", "[our]_path_cost_global_pick",
                "[our]_reached_goal_neighbor_pick", "[our]_path_cost_neighbor_pick",
                "[RRTX]_reached_goal", "[RRTX]_path_cost"]
    result = Result_Log(header_csv=csv_head)
    resultpath = result_logpath(map_name=map_name)

    for i in range(istart, iend, step):
        for j in range(jstart, jend, step):

            goal = i, j

            if not obstacles_check.valid_start_goal(start=start, goal=goal):
                continue
            for r in range(10, 35, 5):
                robot_vision = r
                robotA = robot_OUR(start=start, goal=goal, map_name=map_name, num_iter=num_iter,
                                   robot_vision=robot_vision,
                                   open_points_type=Robot_base.Open_points_type.RRTstar,
                                   picking_strategy=Robot_base.Picking_strategy.global_first, node_density=node_density,
                                   experiment=True, save_image=True, experiment_title=experiment_title)
                robotB = robot_OUR(start=start, goal=goal, map_name=map_name, num_iter=num_iter,
                                   robot_vision=robot_vision,
                                   open_points_type=Robot_base.Open_points_type.RRTstar,
                                   picking_strategy=Robot_base.Picking_strategy.neighbor_first, node_density=node_density,
                                   experiment=True, save_image=True, experiment_title=experiment_title)
                #robotC = robot_RRTX(start_cooridinate=start, goal_coordinate=goal, map_name=map_name,
                #                    num_iter=num_iter, robot_vision=robot_vision,
                #                    RRT_radius=5, RRT_step_size=5, RRT_node_density=node_density,
                #                    experiment=True, save_image=True, experiment_title=experiment_title)
                # Log the result, careful with the data order (start, goal, vision....)
                
                result.add_result([start, goal, robot_vision,
                                   robotA.reach_goal, robotA.cost,
                                   robotA.reach_goal, robotA.cost,
                                   0, 0])
    
        
            result_full_path = os.path.join(resultpath, f"_g({goal[0]}_{goal[1]})_OUR_ONLY.csv")
            result.set_file_name(result_full_path)
            result.write_csv()
            result.clear_data()

    print("DONE!")
