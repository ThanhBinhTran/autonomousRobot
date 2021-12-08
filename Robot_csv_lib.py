"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
"""

import csv

""" write map points into csv file in form of (x,y)
    x,y is casted to integer for easy debug/observe 
"""


def write_map_csv(file_name, f_data, data_header):
    f = open(file_name, 'w', newline='', encoding="utf-8")
    writer = csv.writer(f, delimiter=",")
    writer.writerow(data_header)
    for pt in f_data:
        writer.writerow([int(pt[0]), int(pt[1])])
    f.close()


""" read map points into csv file in form of (x,y) """


def read_map_csv1(mapname):
    first_line = True
    obstacles = []

    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            if first_line:
                first_line = False
                continue
            obstacles.append([float(row[0]), float(row[1])])
    return obstacles


"""read map points  which is consist of many parts """


def read_map_csv(mapname):
    first_line = True
    obstacles = []
    ob_part = []
    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            # print (row[0])
            if row[0].find('x') != -1:
                # print ("find x")
                if len(ob_part) > 1:
                    ob_part.append(ob_part[0])
                    obstacles.append(ob_part)
                ob_part = []
                continue
            ob_part.append([float(row[0]), float(row[1])])
        if len(ob_part) > 1:
            ob_part.append(ob_part[0])
            obstacles.append(ob_part)
    print(obstacles)
    return obstacles


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
