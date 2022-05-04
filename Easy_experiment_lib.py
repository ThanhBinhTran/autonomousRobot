"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
from email.policy import default
import os
import cv2
import numpy as np
import argparse
import matplotlib.pyplot as plt
from sklearn.feature_extraction import img_to_graph
from Robot_math_lib import set_figure_name
from Program_config import *
from Result_log import Result_Log

from enum import Enum
class Experiment_type(Enum):
    COMPARE_LOCAL_GLOBAL = 0
    COMPARE_RANKING_FUNCTION = 1

class Experimental_Result(Result_Log):
    def __init__(self, header_csv = ["start","goal", "range","global_reached_goal","global_cost","local_reached_goal","local_cost"]):
        super().__init__(header_csv = header_csv)
    
    ''' visualizate result from result file '''
    def result_plot(self, result_file):
        # read result as frame
        result_data = self.read_csv_as_dataframe(result_file=result_file)

        #get unique vaule of start and goal
        goal_unique_values = result_data["goal"].unique()
        start_unique_values = result_data["start"].unique()
        
        # display all
        for s_value in start_unique_values:
            data_start = result_data[result_data["start"] == s_value]
            for g_value in goal_unique_values:
                data = data_start[result_data["goal"] == g_value]
                self.draw_plot(data, s_value, g_value)
        
                plt.xlabel("vision range")
                plt.ylabel("path length")
                plt.legend(loc='upper left')
                plt.show()

    ''' draw plot '''
    def draw_plot(self, data, s_value, g_value):
        fig, ax = plt.subplots()

        range = data["range"]
        g_cost = data["global_cost"]
        l_cost = data["local_cost"]

        colors_marker = {True:'green', False:'red'}
        legend_reached = {True:'Reached goal', False:'Not reached goal'}
        legend_global = 'global: start{0}, goal{1}'.format(s_value, g_value)
        legend_local = 'local: start{0}, goal{1}'.format(s_value, g_value)


        plt.plot(range, g_cost, label=legend_global)
        plt.plot(range, l_cost, label=legend_local)

        grouped = data.groupby('global_reached_goal')
        for key, group in grouped:
            group.plot(ax=ax, kind='scatter', x='range', y='global_cost', label=legend_reached[key], color=colors_marker[key])
            group.plot(ax=ax, kind='scatter', x='range', y='local_cost', color=colors_marker[key])
    
    ''' read image then note onto it '''
    def image_text(self, img_name, title, start, goal, vision_range):
        # image read
        img = cv2.imread(img_name)
            
        # add note
        note = "{0}, s {1} -> g {2}, range {3}".format(title, start, goal, vision_range)
        return cv2.putText(img, note, org=(20, 80), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                   fontScale=1, color=(255, 0, 0), thickness=2)

    ''' put all images of start and goal into some bigger images for comparison '''
    def compare_imgs(self, map_name, start, goal, range_list, experiment_type: Experiment_type, 
                    robotA_ranking_function, robotB_ranking_function):

        # group all images of start and goal into arrays of images
        row_lim = 10        # image row limmited by 10
        images_array = []
        imgs_array = []
        i = 0
        for vision_range in range_list:
            # read images, add text note
            if experiment_type == Experiment_type.COMPARE_RANKING_FUNCTION:
                image_name_A = set_figure_name(map_name= map_name, range=vision_range, start=start, 
                    goal=goal, fig_type=".png",strategy=g_strategy, ranking_function=robotA_ranking_function)
                image_name_B = set_figure_name(map_name= map_name, range=vision_range, start=start, 
                    goal=goal, fig_type=".png", strategy=g_strategy, ranking_function=robotB_ranking_function)
                img_A = self.image_text(image_name_A, "rank_linear", start, goal, vision_range)
                img_B = self.image_text(image_name_B, "rank_cosin", start, goal, vision_range)
            
            else:   # default compare global and local 
                image_name_A = set_figure_name(map_name= map_name, range=vision_range, start=start, 
                    goal=goal, fig_type=".png",strategy=g_strategy, ranking_function=robotA_ranking_function)
                image_name_B = set_figure_name(map_name= map_name, range=vision_range, start=start, 
                    goal=goal, fig_type=".png", strategy=l_strategy, ranking_function=robotB_ranking_function)
                img_A = self.image_text(image_name_A, "global", start, goal, vision_range)
                img_B = self.image_text(image_name_B, "local", start, goal, vision_range)

            imgs_array.append([img_A, img_B])
            if ((i+1) %row_lim == 0) or (i == len(range_list)-1): # each big_image contains 2*10 imgs

                images_array.append(imgs_array)
                imgs_array = []
            
            # clean up dispace space
            os.remove(image_name_A) 
            os.remove(image_name_B)
            i = i + 1
        
        # compositing image arrays into bigger ones
        i = 0
        for imgs_array in images_array:

            imgStack = self.stack_images(scale=1, imgArray=imgs_array)
            # read images, add text note
            start_goal_name = "start_{0}_{1}_goal_{2}_{3}_part{4}.png".format(start[0], start[1], goal[0],goal[1], i)
            if experiment_type == Experiment_type.COMPARE_LOCAL_GLOBAL:
                imgStack_name = "compare_{0}_local_global_".format(map_name) + start_goal_name
            elif experiment_type == Experiment_type.COMPARE_RANKING_FUNCTION:
                imgStack_name = "compare_{0}_ranking_function_".format(map_name) + start_goal_name
            else:   # default
                imgStack_name = "compare_{0}_".format(map_name) + start_goal_name
            cv2.imwrite(imgStack_name, imgStack)
            i += 1

    ''' stack array of images into a bigger image'''
    def stack_images(self, scale, imgArray):
        rows = len(imgArray)
        cols = len(imgArray[0])
        rowsAvailable = isinstance(imgArray[0], list)
        width = imgArray[0][0].shape[1]
        height = imgArray[0][0].shape[0]
        if rowsAvailable:
            for x in range ( 0, rows):
                for y in range(0, cols):
                    if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                        imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                    else:
                        imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                    if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
            imageBlank = np.zeros((height, width, 3), np.uint8)
            hor = [imageBlank]*rows
            for x in range(0, rows):
                hor[x] = np.hstack(imgArray[x])
            ver = np.vstack(hor)
        else:
            for x in range(0, rows):
                if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                    imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
                else:
                    imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
                if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
            hor= np.hstack(imgArray)
            ver = hor
        return ver

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-r', metavar="result file",  help='result file', default="result.csv")

    menu_result = parser.parse_args()

    # get user input
    result_file = menu_result.r

    result = Experimental_Result()
    result.result_plot(result_file=result_file)