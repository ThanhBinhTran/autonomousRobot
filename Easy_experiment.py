"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import argparse
from datetime import datetime

from Robot_base import RobotType
from Robot_ranking import Ranking_function

from Robot_main import robot_main as robot_global_ranking_first
from Robot_main_local_strategy import robot_main as robot_local_ranking_first

from Program_config import save_image
from Easy_experiment_lib import Experimental_Result, Experiment_type

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
    range_begin = menu_result.r
    robot_type = RobotType.circle

    # get start point and goal point
    start_list = []
    start_list.append ((0,0))

    goal_list = []
    goal_list.append ((50,50))
    #goal_list.append ((100,100))



    # run robot
    
    ''' 
    NOTE: set show_animation = False and easy_experiment = True in prog_config file to save your time :)) 
    '''

    result = Experimental_Result()
    
    experiment_type= Experiment_type.COMPARE_LOCAL_GLOBAL
    #experiment_type= Experiment_type.COMPARE_RANKING_FUNCTION

    robotA_ranking_function = Ranking_function.Angular_similarity
    robotB_ranking_function = Ranking_function.Cosine_similarity
    robotB_ranking_function = Ranking_function.Angular_similarity

    range_step = 5
    range_max = 80
    range_begin = 20

    print ("\n{0}, RobotA: {1}, RobotB {2}".format(experiment_type, 
                    robotA_ranking_function, robotB_ranking_function ))
    for s in start_list:
        start = s
        for g in goal_list:
            goal = g

            range_experiment_list = []

            for i in range (int ((range_max-range_begin)/range_step)):
                vision_range = range_begin + range_step*i
                range_experiment_list.append(vision_range)

                print("\nStart: {0} --> goal: {1}, range: {2}".format(start, goal, vision_range))
                # compare ranking function
                if experiment_type == Experiment_type.COMPARE_RANKING_FUNCTION:
                    robotA = robot_global_ranking_first(start, goal, map_name, world_name, 
                            num_iter, vision_range, robot_type, robot_radius, robotA_ranking_function)
                    robotB = robot_global_ranking_first(start, goal, map_name, world_name, 
                            num_iter, vision_range, robot_type, robot_radius, robotB_ranking_function)
                
                else:   # compare local vs global pick strategy is default
                    robotA = robot_global_ranking_first(start, goal, map_name, world_name, 
                            num_iter, vision_range, robot_type, robot_radius, robotA_ranking_function)
                    robotB = robot_local_ranking_first(start, goal, map_name, world_name, 
                            num_iter, vision_range, robot_type, robot_radius, robotB_ranking_function)

                # Log the result, careful with the data order (start, goal, vision....)
                result.add_result([start, goal, vision_range, 
                        robotA.reach_goal, robotA.calculate_traveled_path_cost(), 
                        robotB.reach_goal, robotB.calculate_traveled_path_cost() ])
            
            # composite images for easy to analyze
            if save_image:
                result.compare_imgs(start, goal, range_experiment_list, 
                    experiment_type, robotA_ranking_function, robotB_ranking_function)

    # write log file
    if experiment_type == Experiment_type.COMPARE_LOCAL_GLOBAL:
        result.set_header(["start","goal", "range",
                "global_reached_goal","global_cost","local_reached_goal","local_cost"])
        result_file= "result_local_global_{0}.csv".format(datetime.now().strftime("%m_%d_%H_%M_%S") )

    elif experiment_type == Experiment_type.COMPARE_RANKING_FUNCTION:
        result.set_header(["start","goal", "range",
            "reached_goal_ranking_function_1","cost_ranking_function_1",
            "reached_goal_ranking_function_2","cost_ranking_function_2"])
        result_file= "result_ranking_{0}.csv".format(datetime.now().strftime("%m_%d_%H_%M_%S") )

    else:
        result_file= "result_{0}.csv".format(datetime.now().strftime("%m_%d_%H_%M_%S") )

    result.write_csv(file_name=result_file)
    
    # DONE
    print ("\nTo visualize the result run:\n" +
              "python Easy_experiment_lib.py -r {0}".format(result_file))
    print ("\nDONE!  easy experiment....")