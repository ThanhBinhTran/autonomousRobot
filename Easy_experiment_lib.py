"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import os
import cv2
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.feature_extraction import img_to_graph
from Robot_lib import set_image_name

class Experimental_Result:
    def __init__(self) -> None:
        self.start = []
        self.goal = []
        self.range = []
        self.global_path_cost = []
        self.local_path_cost = []
        self.global_reach_goal = []
        self.local_reach_goal = []
        self.header = ["start","goal","range","global_reached_goal","global_cost","local_reached_goal","local_cost"]
    
    ''' save result in to list '''
    def record_result(self, s, g, r, gpc, lpc, grg, lrg):
        self.start.append(s)
        self.goal.append(g)
        self.range.append(r)
        self.global_path_cost.append(gpc)
        self.local_path_cost.append(lpc)
        self.global_reach_goal.append(grg)
        self.local_reach_goal.append(lrg)
    
    ''' set header '''
    def set_header (self, header_title):
        self.header = header_title

    ''' write result to csv file '''
    def write_csv(self, file_name):
        f = open(file_name, 'w', newline='', encoding="utf-8")
        writer = csv.writer(f, delimiter=",")
        writer.writerow(self.header)
        for s,g,r,grg,gpc,lrg,lpc in zip(self.start, self.goal, self.range, self.global_reach_goal,
            self.global_path_cost,self.local_reach_goal, self.local_path_cost):
            writer.writerow([s,g,r,grg,gpc,lrg,lpc])
        f.close()

    ''' visualizate result from result file '''
    def result_plot(self, result_file):
        # read result as frame
        result_data = pd.read_csv(result_file)

        #get unique vaule of start and goal
        goal_unique_values = result_data["goal"].unique()
        start_unique_values = result_data["start"].unique()
        
        # display all
        #for s_value in start_unique_values:
            #data_start = result_data[result_data["start"] == s_value]
            #for g_value in goal_unique_values:
            #g_value = goal_unique_values[0]
            #data = data_start[result_data["goal"] == g_value]

        s_value = start_unique_values[0]
        g_value = goal_unique_values[0]
        data_start = result_data[result_data["start"] == s_value]
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
    def image_text(self, img_name, strategy, start, goal, vision_range):
        # image read
        img = cv2.imread(img_name)
            
        # add note
        note = "{0}: start {1}, goal {2}, range {3}".format(strategy, start, goal, vision_range)
        return cv2.putText(img, note, org=(50, 80), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                   fontScale=1, color=(255, 0, 0), thickness=2)

    ''' put all images of start and goal into some bigger images for comparison '''
    def compare_imgs(self, start, goal, range_list):

        # group all images of start and goal into arrays of images
        row_lim = 5        # image row limmited by 10
        images_array = []
        imgs_array = []
        i = 0
        for vision_range in range_list:
            file_name = set_image_name(range=vision_range, start=start, goal=goal)
            file_name_g = file_name + "_global_strategy.png"
            file_name_l = file_name + "_local_strategy.png"
            # read images, add text note
            g_img = self.image_text(file_name_g, "global", start, goal, vision_range)
            l_img = self.image_text(file_name_l, "local", start, goal, vision_range)

            imgs_array.append([g_img, l_img])
            if ((i+1) %row_lim == 0) or (i == len(range_list)-1): # each big_image contains 2*10 imgs

                images_array.append(imgs_array)
                imgs_array = []
            
            # clean up dispace space
            os.remove(file_name_g) 
            os.remove(file_name_l)
            i = i + 1
        
        # compositing image arrays into bigger ones
        i = 0
        for imgs_array in images_array:

            imgStack = self.stack_images(1, imgs_array)
            imgStack_name = "compare_start_{0}_{1}_goal_{2}_{3}_part{4}.png".format(start[0], start[1], 
                    goal[0],goal[1], i)
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
            hor_con = [imageBlank]*rows
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
    result = Experimental_Result()
    result.result_plot(result_file="result_03_25_11_58_50.csv")