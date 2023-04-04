"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""

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

def m_s_g_r_name_file(start, goal, map_name, range):
    return f"{map_name}_s({start[0]}_{start[1]})_g({goal[0]}_{goal[1]})_r{range}"


if __name__ == '__main__':

    # get user input
    menu_result = robot_user_input()
    num_iter = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    start = menu_result.sx, menu_result.sy
    goal = menu_result.gx, menu_result.gy
    robot_vision = menu_result.r

    sample_size = menu_result.ss
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
    m_s_g = m_s_g_r_name_file(map_name=map_name, start=start, goal=goal, range=robot_vision)

    ''' get obstacles data whether from world (if indicated) or map (by default)'''

    obstacles_check = Obstacles()

    start = 0, 0
    num_iter = 100
    map_name = "_MuchMoreFun.csv"
    # map_name = "_map_u_liked_shape.csv"
    obstacles_check.read(map_name=map_name)
    obstacles_check.line_segments()

    result = Result_Log(header_csv=csv_head)
    nm = map_name.replace('.csv', '')

    istart, iend = 20, 101
    jstart, jend = 10, 101
    step = 10
    for i in range(istart, iend, step):
        for j in range(jstart, jend, step):

            goal = i, j

            if not obstacles_check.valid_start_goal(start=start, goal=goal):
                continue
            for r in range(10, 36, 5):
                robot_vision = r
                robotA = robot_OUR(start=start, goal=goal, map_name=map_name, num_iter=num_iter,
                                   robot_vision=robot_vision,
                                   open_points_type=Robot_base.Open_points_type.RRTstar,
                                   picking_strategy=Robot_base.Picking_strategy.global_first, sample_size=sample_size,
                                   experiment=True, save_image=True)
                robotB = robot_OUR(start=start, goal=goal, map_name=map_name, num_iter=num_iter,
                                   robot_vision=robot_vision,
                                   open_points_type=Robot_base.Open_points_type.RRTstar,
                                   picking_strategy=Robot_base.Picking_strategy.neighbor_first, sample_size=sample_size,
                                   experiment=True, save_image=True)
                robotC = robot_RRTX(start_cooridinate=start, goal_coordinate=goal, map_name=map_name,
                                    num_iter=num_iter, robot_vision=robot_vision,
                                    RRT_radius=5, RRT_step_size=5, RRT_sample_size=sample_size,
                                    experiment=True, save_image=True)
                # Log the result, careful with the data order (start, goal, vision....)
                result.add_result([start, goal, robot_vision,
                                   robotA.reach_goal, robotA.cost,
                                   robotB.reach_goal, robotB.cost,
                                   robotC.reach_goal, robotC.cost])
    result.set_file_name(f"{nm}_g({istart}-{iend}_{jstart}-{jend})OUR_RRTX.csv")
    result.write_csv()

    print("DONE!")
