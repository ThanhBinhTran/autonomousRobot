''' put all images of start and goal into some bigger images for comparison '''
from Result_log import Result_Log
import cv2
import numpy as np
import os

def check_exist_file(fname):
    if not os.path.exists(fname):
        print (f"not found {fname}")
        return False
    return True

def compare_imgs():

    # group all images of start and goal into arrays of images
    start = 0,0
    #goal = 30, 60
    #goal = 40, 40
    #goal = 40, 50
    #goal = 40, 70
    #goal = 80, 140
    goal = []
    #for map_name = "_map_bugtrap"
    map_case = 2
    if map_case == 1:
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
    for i in range(istart, iend, step):
        for j in range(jstart, jend, step):
            goal.append((i,j))

    if False:
        map_name = "_map_bugtrap"
        goal = [
                (80, 140),
                (100,140),
                ]
    if False:
        map_name = "_map_deadend"
        goal = [
                (30, 60),
                (40, 40),
                (40, 50),
                (40, 70),
                (40, 80),
                (60, 60),
                (60, 80),
                (70, 50),
                (70, 70),
                (70, 90),
                (75, 50),
                (80, 40),
                (80, 60),
                (80, 70),
                (90, 60),
                ]

    experiment_title ="experiment1"
    #experiment_title ="experiment2"
    for g in goal:

        imgs_array = []
        if experiment_title.find('1') > 0:
            basename = Result_Log.prepare_name(start=start, goal=g, pick=None,
                                       range=20, open_points_type=None,
                                       map_name=map_name, experiment_title=experiment_title)
            for i in range (100):
                nameA = f"{basename}_case{i}_APS.png"
                nameB = f"{basename}_case{i}_APS_improve.png"
                nameC = f"{basename}_case{i}_Astar.png"
                nameD = f"{basename}_case{i}_RRTstar.png"

                if not check_exist_file(nameA) or not check_exist_file(nameB) or \
                        not check_exist_file(nameC) or not check_exist_file(nameD):
                    break
                # Read the images\
                img_A = cv2.imread(nameA)
                img_B = cv2.imread(nameB)
                img_C = cv2.imread(nameC)
                img_D = cv2.imread(nameD)
                imgs_array.append([img_A, img_B, img_C, img_D])
        elif experiment_title.find('2') > 0:
            
            for i in range (10, 30, 5):
                nameA = Result_Log.prepare_name(start=start, goal=g, pick="Picking_strategy.global_first",
                                       range=i, open_points_type="Open_points_type.RRTstar",
                                       map_name=map_name, experiment_title=experiment_title)
                nameB = Result_Log.prepare_name(start=start, goal=g, pick="Picking_strategy.neighbor_first",
                                       range=i, open_points_type="Open_points_type.RRTstar",
                                       map_name=map_name, experiment_title=experiment_title)
                nameC = Result_Log.prepare_name(start=start, goal=g, pick=None,
                                       range=i, open_points_type=None,
                                       map_name=map_name, experiment_title=experiment_title)
                name_A = f"{nameA}.png"
                name_B = f"{nameB}.png"
                name_C = f"{nameA}_improve.png"
                name_D = f"{nameB}_improve.png"
                name_E = f"{nameC}RRTx.png"
                if not check_exist_file(name_A) or not check_exist_file(name_B) or not check_exist_file(name_C)\
                    or not check_exist_file(name_E) or not check_exist_file(name_D):
                    break
                # Read the images\
                img_A = cv2.imread(name_A)
                img_B = cv2.imread(name_B)
                img_C = cv2.imread(name_C)
                img_D = cv2.imread(name_D)
                img_E = cv2.imread(name_E)
                imgs_array.append([img_A, img_B, img_C, img_D, img_E])
        if len(imgs_array) > 0:
            simg = stack_images(scale=1, imgArray=imgs_array)
            # Save the stacked image to a file
            composite_img = f"test/{experiment_title}{map_name}_goal{g}.png"
            #cv2.imwrite(composite_img, simg)

''' stack array of images into a bigger image'''
def stack_images(scale, imgArray):
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


compare_imgs()