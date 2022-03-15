'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
'''

import csv
from Robot_world_lib import World

class Obstacles:
    def __init__(self):
        self.obstacles = []  # list of obstacles
    
    data = lambda self: self.obstacles
   
    ''' 
    write vertices of obstacles into csv file in form of (x,y)
    x,y is casted to integer for easy debug/observe 
    '''
    def write_csv(self, file_name, f_data, data_header, first):
        if first:
            f = open(file_name, 'w', newline='', encoding="utf-8")
        else:
            f = open(file_name, 'a', newline='', encoding="utf-8")
            
        writer = csv.writer(f, delimiter=",")
        writer.writerow(data_header)
        for pt in f_data:
            writer.writerow([int(pt[0]), int(pt[1])])
        f.close()

    ''' read map from world or csv data (default) '''
    def read(self, world_name=None, map_name=None):
        # read world map then get obstacles information
        if world_name is not None:
            World().read_map_from_world(world_name)
            self.read_csv(world_name + ".csv")
        else:
            self.read_csv(map_name)


    '''
        read map_csv (in form the list of vertix of obstacles)
    '''

    def read_csv(self, mapname):
        self.obstacles = []  # list of obstacles
        
        obstacle = []   # list of vertices

        with open(mapname, newline='') as f:
            reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
            for row in reader:
                if row[0].find('x') != -1:  # header line
                    if len(obstacle) > 1:   
                        obstacle.append(obstacle[0])    # append the first point to make a polygon
                        self.obstacles.append(obstacle)      # append obstacle to obstacles
                    obstacle = []                       # clear and prepare for next obstacle
                    continue
                else:
                    obstacle.append([float(row[0]), float(row[1])]) # get point data
            if len(obstacle) > 1:   # append the last one
                obstacle.append(obstacle[0])
                self.obstacles.append(obstacle)
        return self.obstacles
