'''
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target) 
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn
'''

import csv
from Robot_world_lib import World
from Robot_lib import *

class Obstacles:
    def __init__(self):
        self.obstacles = []     # list of obstacles
        self.config_space = []  # configuration space 
        self.enable_config_space = False    # set True for using configuration space
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
                    obstacle.append( tuple( (float(row[0]), float(row[1])) ) ) # get point data
            if len(obstacle) > 1:   # append the last one
                obstacle.append(obstacle[0])
                self.obstacles.append(obstacle)
        self.config_space = self.obstacles
        return self.obstacles
    
    ''' serialize obstacle in to list of linesegments'''
    def line_segments(self):
        obstacles_line_segments = []
        for obstacle in self.obstacles:
            line_segments = []
            for i in range (len(obstacle) -1):
                line_segments.append( [obstacle[i], obstacle[i+1]] ) 

            obstacles_line_segments.append(line_segments)
        return obstacles_line_segments

    ''' translate line segment by a vector '''
    def translate_line_segment(self, line_segment, vector):
        return [tuple(line_segment[0] +  vector), tuple(line_segment[1] +  vector)]

    ''' translate line segments by vectors '''
    def translate_line_sengments(self, obstacles_line_segments, n_obs_vectors):
        extend_line_segments = []
        for obstacle_line_segments,  n_ob_vectors in zip(obstacles_line_segments, n_obs_vectors):
            linesegments = []
            for ob_line_segment, n_ob_vector in zip(obstacle_line_segments,  n_ob_vectors):
                linesegments.append(self.translate_line_segment(ob_line_segment, n_ob_vector))
            extend_line_segments.append(linesegments)
        return extend_line_segments

    ''' this function is to find a configure space which is robot free-collision ares 
        NOTE: polygon getting from opencv.findcontour is clockwise 
    '''
    def find_configuration_space(self, robot_radius):
        if len(self.obstacles) > 0:
            self.enable_config_space = True
            normal_vectors = []        # list of normal vectors
            self.config_space = []
            obstacles_line_segments = self.line_segments()
            
            # get normal vectors of all obstacles
            for line_segments in obstacles_line_segments:
                nvectors = []
                for line_sengment in line_segments:
                    nvectors.append(normal_vector(line_sengment, robot_radius))
                normal_vectors.append(nvectors)

            # extend obstacles line segments by its normal vectors
            extend_ob_line_segments = self.translate_line_sengments(obstacles_line_segments, normal_vectors)

            # find line segments which are normal vectors of bisector (ptA, midpt, ptB)
            bisector_line_segnments = []
            for obstacle in self.obstacles:
                lim_ls = []
                for i in range(len(obstacle)-1):
                    if i == len(obstacle) - 2:  # last item
                        ls = cal_bisector(obstacle[i - 1], obstacle[i], obstacle[1], robot_radius)
                    else:
                        ls = cal_bisector(obstacle[i - 2], obstacle[i], obstacle[i + 1], robot_radius)
                    lim_ls.append(ls)
                bisector_line_segnments.append(lim_ls)


            # find boundary of configuration space
            for bisector_ls, extend_ob_ls in zip(bisector_line_segnments, extend_ob_line_segments):
                cspace = []
                for i in range(len(bisector_ls)):
                    vectorA = np.subtract (extend_ob_ls[i-1][0], extend_ob_ls[i-1][1])
                    vectorB = np.subtract (extend_ob_ls[i][1], extend_ob_ls[i][0])
                    signed_angleAB = signed_angle(vectorA, vectorB)
                    if signed_angleAB < 0 and signed_angleAB > -math.pi:
                        jointPtA = line_intersection(extend_ob_ls[i], extend_ob_ls[i-1])
                        cspace.append(jointPtA)
                    else:
                        jointPtA = line_intersection(bisector_ls[i], extend_ob_ls[i-1])
                        jointPtB = line_intersection(bisector_ls[i], extend_ob_ls[i])
                        cspace.append(jointPtA)
                        cspace.append(jointPtB)
                cspace.append(cspace[0])    # make a polygon by appending the beginning
                self.config_space.append(cspace)
        else:
            print ("No obststacle detected")
            exit(0)
