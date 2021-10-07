"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""

from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import csv

def thresh_callback(threshold, src_gray, world_name):
    # Detect edges using Canny
    canny_output = cv.Canny(src_gray, threshold, threshold * 2)
    # Find contours
    contours, hierarchy = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
       
    file_name = world_name + ".csv"
    data_header = ["x","y"]
    f = open(file_name, 'w', newline='', encoding="utf-8")
    writer = csv.writer(f, delimiter=",")
    for part in contours:
        writer.writerow(data_header)
        for pt in part:
            writer.writerow([pt[0][0],pt[0][1]])
    f.close() 


# Load source image
def read_map_from_world(world_name):
    src = cv.imread(world_name)
    if src is None:
        print('Could not open or find the image:', world_name)
        exit(0)
        
    # Convert image to gray and blur it
    print ("find contours of world:", world_name)
    src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    src_gray = cv.blur(src_gray, (3,3))

    max_thresh = 255
    thresh = 100 # initial threshold
    obstacle = thresh_callback(thresh, src_gray, world_name)
    print (obstacle)

def world_display(plt, mpimg, world_name):
    img = mpimg.imread(world_name)
    plt.imshow(img)