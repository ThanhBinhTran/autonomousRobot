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

def result_logpath(start, goal, map_name, range):
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
    world_name = menu_result.w
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
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

    csv_head = ["start", "goal", "range",
                "our_global_reached", "our_global_cost",
                "our_lobal_reached", "our_lobal_cost",
                "RRTX_reached", "RRTX_cost", ]
    resultpath = result_logpath(map_name=map_name, start=start, goal=goal, range=robot_vision)

    ''' get obstacles data whether from world (if indicated) or map (by default)'''

    obstacles_check = Obstacles()

    start = 0, 0
    num_iter = 100
    #map_name = '_map_forest.csv' # 500x500 size
    node_density = 10
    
    map_name = '_map_deadend.csv' # 100x100 size
    node_density = 5
    istart, iend = 75, 76 
    jstart, jend = 50, 51
    step = 5
    # map_name = '_map_bugtrap.csv' # 200x200 size
    # map_name = '_map_blocks.csv' # 300X 350 size
    obstacles_check.read(map_name=map_name)
    obstacles_check.line_segments()

    result = Result_Log(header_csv=csv_head)
    nm = map_name.replace('.csv', '')


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
                robotC = robot_RRTX(start_cooridinate=start, goal_coordinate=goal, map_name=map_name,
                                    num_iter=num_iter, robot_vision=robot_vision,
                                    RRT_radius=5, RRT_step_size=5, RRT_node_density=node_density,
                                    experiment=True, save_image=True, experiment_title=experiment_title)
                # Log the result, careful with the data order (start, goal, vision....)
                result.add_result([start, goal, robot_vision,
                                   robotA.reach_goal, robotA.cost,
                                   robotB.reach_goal, robotB.cost,
                                   robotC.reach_goal, robotC.cost])
    result_full_path = os.path.join(resultpath, f"_g({istart}-{iend}_{jstart}-{jend})_OUR_RRTX.csv")
    result.set_file_name(result_full_path)
    result.write_csv()

    print("DONE!")
