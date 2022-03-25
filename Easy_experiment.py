"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import argparse
from datetime import datetime

from Robot_lib import plot_img_name
from Robot_base import RobotType
from Robot_main import robot_main as robot_global_ranking_first
from Robot_main_local_strategy import robot_main as robot_local_ranking_first
from Program_config import save_image
from Easy_experiment_lib import Experimental_Result



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=100)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_MuchMoreFun.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=5.0)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.5)

    menu_result = parser.parse_args()

    # get user input
    num_iter = menu_result.n
    map_name = menu_result.m
    world_name = menu_result.w
    robot_radius = menu_result.radius
    robot_vision = menu_result.r
    robot_type = RobotType.circle

    # get start point and goal point
    start_list = []
    start_list.append ((0,0))

    goal_list = []
    goal_list.append ((50,50))
    goal_list.append ((50,70))
    goal_list.append ((70,50))



    # run robot
    
    ''' 
    NOTE: set show_animation = False and easy_experiment = True in prog_config file to save your time :)) 
    '''

    #plotter_esay = Plot_base()
    result = Experimental_Result()

    range_step = 5
    range_max = 10
    range_begin = 5
    for s in start_list:
        start = s
        for g in goal_list:
            goal = g

            range_experiment_list = []

            for i in range (range_max):
                vision_range = range_begin + range_step*i
                range_experiment_list.append(vision_range)

                print("Robot is reaching to goal: {0} from start: {1}, vision range: {2}".format(goal, start, vision_range))
                robot_global = robot_global_ranking_first(start, goal, map_name, world_name, num_iter, vision_range, robot_type, robot_radius)
                robot_local = robot_local_ranking_first(start, goal, map_name, world_name, num_iter, vision_range, robot_type, robot_radius)
                
                result.record_result(s=start, g=goal, r=vision_range, 
                        gpc=robot_global.calculate_traveled_path_cost(), 
                        lpc=robot_local.calculate_traveled_path_cost(), 
                        grg=robot_global.reach_goal, lrg= robot_local.reach_goal)

            if save_image:
                result.compare_imgs(start, goal, range_experiment_list)
    
    result_file= "result_{0}.csv".format(datetime.now().strftime("%m_%d_%H_%M_%S") )
    result.write_csv(file_name=result_file)
    
    print ("\nTo visualize the result run python Easy_experiment_lib with result_file = {0}".format(result_file))
    print ("\nDONE!  easy experiment....")