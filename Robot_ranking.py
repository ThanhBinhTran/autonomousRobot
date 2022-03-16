"""
autonomousRobot
This project is to simulate an autonomousRobot that try to find a way to reach a goal (target)
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
"""
import numpy as np
from Robot_lib import *

class Ranker:
    def __init__(self, alpha = 0.9, beta = 0.1) :
        self.alpha = alpha
        self.beta = beta
    
    def set_alpha(self, x):
        self.alpha = x
    def set_beta(self, x):
        self.beta = x

    ''' score function'''
    def score_function(self, angle=math.pi/2, distance= 1, alpha = 0.9, beta = 0.1):
        # ranking = alpha/(distance**2) + beta/abs(angle**2)
        
        angle = angle / math.pi
        distance = distance / 150
        # print ("Ranking score: angle {0}, distance {1}".format(angle, distance) )
        score = float('inf')
        if not math.isclose(angle, 0.0) and not math.isclose(distance, 0.0):
            score = alpha / (distance) + beta / abs(angle)
            
        return score

    ''' ranking a given point  by its angle (from center to point and to goal) and its distance (to goal)'''
    def rank(self, center, point, goal):

        vector_cg = np.subtract(center, goal)
        vector_pg = np.subtract(center, point)

        sa = signed_angle(vector_cg, vector_pg)
        dist = point_dist(goal, point)
        rank_score = self.score_function(sa, dist, self.alpha, self.beta)
        return [rank_score]