'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
'''

import csv

''' 
    write map points into csv file in form of (x,y)
    x,y is casted to integer for easy debug/observe 
'''

def write_map_csv(file_name, f_data, data_header):
    f = open(file_name, 'w', newline='', encoding="utf-8")
    writer = csv.writer(f, delimiter=",")
    writer.writerow(data_header)
    for pt in f_data:
        writer.writerow([int(pt[0]), int(pt[1])])
    f.close()


'''
    read map points which consists of many obstacle
'''

def read_map_csv(mapname):
    obstacle = []   # list of points
    obstacles = []  # list of obstacle
    
    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            if row[0].find('x') != -1:  # header line
                if len(obstacle) > 1:   
                    obstacle.append(obstacle[0])    # append the first point to make a polygon
                    obstacles.append(obstacle)      # append obstacle to obstacles
                obstacle = []                       # clear and prepare for next obstacle
                continue
            else:
                obstacle.append([float(row[0]), float(row[1])]) # get point data
        if len(obstacle) > 1:   # append the last one
            obstacle.append(obstacle[0])
            obstacles.append(obstacle)
    #print(obstacles)
    return obstacles

''' read map for dwa algorithm '''
def read_map_csv_dwa(mapname):
    first_line = True
    obstacles = []

    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            if first_line:
                first_line = False
                continue
            obstacles.append([float(row[0]) / 6.6, float(row[1]) / 6.6])
    return obstacles
