import numpy as np
from sys import float_info
from Robot_lib import *    

def ranking(center, pt, goal):
    '''
    score the open point by its angle (from center to point and goal) and its distance (to goal)
    '''
    
    # ranking = a/as + b/dist
    a = 1
    b = 1
    sa =  signed_angle(goal-center, pt - center)
    dist = point_dist (goal, pt)
    sa = sa/math.pi
    dist = dist/100
    if sa != 0 or dist != 0:
        rank_score = a/abs(sa) + b/dist
    else:    
        rank_score = float_info.max
    return [rank_score]
    
def pick_next(ao_gobal):
    '''
    return index and value of next point if there exist any active open point
    otherwise -1
    '''
    pick_idx = -1
    next_pt = []
    if len(ao_gobal) > 0:
        ranks = ao_gobal[:,2]
        print ("global open points: ", ao_gobal[:,0:2])
        print ("global ranks: ",ranks)
        print ("pick index ", np.argmax(ranks))
        print ("picked rank ", np.amax(ranks))
        pick_idx = np.argmax(ranks)
        next_pt = ao_gobal[pick_idx,0:2]
        
    return pick_idx, next_pt 
def all_remaining_point_same_side(a, b, c, obstacle_list):
    ab = np.array([[a,b]])
    # ax + by = c
    dist = np.dot(ab, obstacle_list) - c
    #print ("dist ", dist, "a {0}, b {1}, c {2} ".format(a, b, c))
    sameside1 = (dist >= 0)
    sameside2 = (dist <= 0)
    if sum(sameside1[0]) == len(sameside1[0]) or sum(sameside2[0]) == len(sameside2[0]):
        return True
    return False
    
def get_possible_AH_next_points_from_point(AH_points, obstacle_list, startpoint, goal):
    # return [start point, [all next possible next point], [add blind points from AH_points], reach goal = true/false]
    possible_AH_next_points = []
    blind_point = []
    goal_apprear = False
    print ("start point .........", startpoint)
    for i in range (len(obstacle_list[0])): # get number of column
        [a, b, c] = lineFromPoints(startpoint, obstacle_list[:,i])
        #print ("From point ", obstacle_list[:,i])
        get_dist = all_remaining_point_same_side(a, b, c, obstacle_list)
        if get_dist:
            # Check if selected point is existed in the path
            point = (obstacle_list[0,i], obstacle_list[1,i])
            if point not in AH_points:
                print ("__Linked ", obstacle_list[:,i])
                # concatenate path to matrix path
                AH_points.append((obstacle_list[0,i], obstacle_list[1,i]))
                print ("__PATH  ", AH_points)
                #plt.plot((startpoint[0],obstacle_list[0][i]), (startpoint[1], obstacle_list[1][i]), "--r")
                #plt.plot(obstacle_list[0][i], obstacle_list[1][i], "or")
                possible_AH_next_points.append(obstacle_list[:,i])
        else:
            blind_point.append(obstacle_list[:,i])
            #plt.plot((startpoint[0],obstacle_list[0][i]), (startpoint[1], obstacle_list[1][i]), "--g")
        #plt.pause(0.1)
        
    [a, b, c] = lineFromPoints(startpoint, goal)
    #print ("Check the goal ", obstacle_list[:,i])
    goal_apprear = all_remaining_point_same_side(a, b, c, obstacle_list)
    
    return  [startpoint, possible_AH_next_points, blind_point, goal_apprear]
                
def find_AH_paths(ox_b, oy_b, startpoint, goal):
    obstacle_list = np.array([ox_b, oy_b])
    print (obstacle_list)
    AH_paths = []
    AH_points = []
    AH_points.append(startpoint)
    for start_point in AH_points:
        AH_path_temp = get_possible_AH_next_points_from_point(AH_points, obstacle_list, start_point, goal)
        AH_paths.append(AH_path_temp)
    return AH_paths