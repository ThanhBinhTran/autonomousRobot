import numpy as np
from Robot_math_lib import *
from Sight import Sight
import time

def motion(current_position, next_pt):
    '''
    motion model
    '''
    #current_position = (approximately_num(next_pt[0]), approximately_num(next_pt[1]))
    current_position = next_pt[0], next_pt[1]
    return current_position

def all_remaining_point_same_side(a, b, c, obstacle_list):
    ab = np.array([[a, b]])
    # ax + by = c
    dist = np.dot(ab, obstacle_list) - c
    # print ("dist ", dist, "a {0}, b {1}, c {2} ".format(a, b, c))
    sameside1 = (dist >= 0)
    sameside2 = (dist <= 0)
    if sum(sameside1[0]) == len(sameside1[0]) or sum(sameside2[0]) == len(sameside2[0]):
        return True
    return False

def approximately_shortest_path(skeleton_path, visited_sights, robot_vision):
    lstime = 0.0
    atime = 0.0
    critical_ls = []
    asp = skeleton_path
    if len(skeleton_path) > 2:
        spt = skeleton_path[0]
        gpt = skeleton_path[-1]
        stime = time.time()
        critical_ls = get_critical_linesegments(skeleton_path, visited_sights, robot_vision)
        lstime = time.time()- stime
        stime = time.time()
        asp = approximately_sp_ls(critical_ls, spt, gpt)
        atime = time.time()- stime
    return asp, critical_ls, lstime, atime

def approximately_shortest_path_old(skeleton_path, visited_sights, robot_vision):
    lstime = 0.0
    atime = 0.0
    critical_ls = []
    asp = skeleton_path
    if len(skeleton_path) > 2:
        spt = skeleton_path[0]
        gpt = skeleton_path[-1]
        stime = time.time()
        critical_ls = get_critical_linesegments_old(skeleton_path, visited_sights, robot_vision)
        lstime = time.time()- stime
        stime = time.time()
        asp = approximately_sp_ls_old(critical_ls, spt, gpt)
        atime = time.time()- stime
    return asp, critical_ls, lstime, atime

def get_safe_radius(skeleton_path, robot_vision):
    safe_radius = robot_vision
    if len(skeleton_path) == 3:
        safe_radius = robot_vision
    else:
        safe_radius = robot_vision / 2
        for i in range(1, len(skeleton_path) - 2):
            for j in range(i + 1, len(skeleton_path) - 1):
                dist = point_dist(skeleton_path[i], skeleton_path[j])
                if dist < safe_radius:
                    safe_radius = dist
    return safe_radius

''' given sorted list, find the first index of element where give point < those elements'''
def find_anchor(angle_pt, angles_ls, start_idx, end_idx):
    mid_idx = int((end_idx-start_idx)/2) + start_idx
    if end_idx == start_idx:
      if angle_pt > angles_ls[end_idx]:
         return end_idx + 1
      else:
         return end_idx
    if angle_pt < angles_ls[mid_idx]:
      return find_anchor(angle_pt, angles_ls, start_idx, mid_idx)
    elif angle_pt >  angles_ls[mid_idx+1]:
      return find_anchor(angle_pt, angles_ls, mid_idx+1, end_idx)
    else:
       return mid_idx+1
    
def get_disjoint_ls(center, point, safe_radius):
    safe_pt = point
    if point_dist(center, point) > safe_radius:
        vector = np.subtract(point, center)
        uvector = unit_vector(vector)
        direction_vector = np.multiply(uvector, safe_radius)
        safe_pt = np.add(center, direction_vector)
    return list(safe_pt)

def create_fake_linesegment(center, pre_pt, post_pt, vision_range):
    # if 3 point makes a straight line
    if belong_line(center, (pre_pt, post_pt)):
        #print ("center, pre_pt, post_pt are on the same line")
        vector_c_pre = np.subtract(pre_pt,  center )
        vector_c_post = np.subtract(post_pt, center)
        ptA = rotate_vector(vector_c_pre, math.pi/2)
        ptB = rotate_vector(vector_c_post, math.pi/2)

        ptA = np.add(ptA, center)
        ptB = np.add(ptB, center)
        return ptA, ptB
    else:
        #print ("center, pre_pt, post_pt not are on the same line")
        midpt = get_middle_direction(center, vision_range, (pre_pt, post_pt))
        return midpt, center
    
''' closed sighed is sorted in anti-clockwise direction '''
def get_sorted_local_linesegments(closed_sights, center, pre_pt, post_pt, vision_range, safe_radius):
    result = []
    root = center
    side_pts = []
    print ("closed sights: ", closed_sights)
    len_closed_sight = len(closed_sights)
    if len_closed_sight> 0:
        ls_pts = closed_sights[:,0:2]   # get 2 first = a pair
        ls_angles = closed_sights[:,2:4]
        lsptA_angles = closed_sights[:,4:5]
        print ("ls_pts",ls_pts)
        print ("ls_angles",ls_angles)
        print ("lsptA_angles",lsptA_angles)

    if len_closed_sight == 1:
        #print ("________________________1 CLOSED SIGHT, center = " , center )
        inside, _ = inside_angle_area(point=closed_sights[0][0], center=center,ref_boundaries=(pre_pt, post_pt))
        if inside:
            side_pts.append(ls_pts[0][0])
            side_pts.append(ls_pts[0][1])
    elif len_closed_sight > 1:
        v_pre = np.subtract(pre_pt, center)
        pre_angle = math.pi*2 - unsigned_angle_vector_xAxis(v_pre)
        v_post = np.subtract(post_pt, center)
        post_angle = math.pi*2 - unsigned_angle_vector_xAxis(v_post)
        #print (f"________________________>1 CLOSED SIGHT, center = {center}, pre{pre_pt}, post{post_pt} ")
        if pre_angle > post_angle:
            pre_angle, post_angle = post_angle, pre_angle
        start_idx = find_anchor(angle_pt= pre_angle, angles_ls=lsptA_angles, start_idx=0, end_idx=len_closed_sight -1)
        end_idx = find_anchor(angle_pt= post_angle, angles_ls=lsptA_angles, start_idx=0, end_idx=len_closed_sight -1)
        #print (f"return value:{start_idx} for {pre_angle}")
        #print (f"return value:{end_idx} for {post_angle}")
        #print ("ls_ptAs", ls_pts)
        P1_pts = ls_pts[0: start_idx]
        P2_pts = ls_pts[start_idx: end_idx]
        P3_pts = ls_pts[end_idx:]
        P1_angle = ls_angles[0: start_idx]
        P2_angle = ls_angles[start_idx: end_idx]
        P3_angle = ls_angles[end_idx:]
        #print (f"__________{P1_pts}-- {P2_pts}-- {P3_pts}")
        side_temp_pt = []
        if start_idx != end_idx: # partition 2 is not empty, then pick 1 which is inside
            if inside_angle_area(point=ls_pts[start_idx][0],center=center, ref_boundaries=(pre_pt, post_pt))[0]:
                side_temp_pt = P2_pts
                side_temp_angle = P2_angle
            else:
                if len(P1_pts) == 0:
                    side_temp_pt = P3_pts
                    side_temp_angle = P3_angle
                elif len(P3_pts) == 0:
                    side_temp_pt = P1_pts
                    side_temp_angle = P1_angle
                else:
                    side_temp_pt = np.vstack([P3_pts,P1_pts])
                    side_temp_angle = np.vstack([P3_angle,P1_angle])
        else: # parttion 2 is empty, then check if partition 1 and 3 is inside
            #print ("ls_pts", ls_pts)
            if inside_angle_area(point=ls_pts[0][0],center=center, ref_boundaries=(pre_pt, post_pt))[0]:
                if len(P1_pts) == 0:
                    side_temp_pt = P3_pts
                    side_temp_angle = P3_angle
                elif len(P3_pts) == 0:
                    side_temp_pt = P1_pts
                    side_temp_angle = P1_angle
                else:
                    side_temp_pt = np.vstack([P3_pts,P1_pts])
                    side_temp_angle = np.vstack([P3_angle,P1_angle])
        #print ("result_ptA", side_temp_pt)
        for i in range (len(side_temp_pt)):
            duplicate = False
            if i > 0:
                angle_pair_pre = side_temp_angle[i-1][0]
                angle_pair_post = side_temp_angle[i][0]
                if math.isclose(angle_pair_pre[1], angle_pair_post[0]):    # duplicate points
                    duplicate = True
            if not duplicate:
                side_pts.append(side_temp_pt[i][0])
            side_pts.append(side_temp_pt[i][1])


    # no closed sights detected alongside skeleton node
    # then create a fake line segment
    if len(side_pts) ==0: 
        ptA, root = create_fake_linesegment(center=center, pre_pt=pre_pt, post_pt=post_pt, vision_range=vision_range)
        side_pts.append(ptA)

    # sorted line segment according to skeleton path direction
    if len(side_pts)>1:
      v1 = np.subtract(pre_pt, center)
      vfirst = np.subtract(side_pts[0], center)
      vlast = np.subtract(side_pts[-1], center)
      angle_pre_first = signed_angle(v1,vfirst)
      angle_pre_last = signed_angle(v1,vlast)
      if abs(angle_pre_last) < abs(angle_pre_first):
         side_pts = side_pts[::-1]

    #print ("len(side_pts)", len(side_pts))
    for pt in side_pts:
        _,safe_pt = safe_linesegment(begin=root, end=pt, length=safe_radius)
        _,disjont_root = scale_vector(begin=pt, end=root, scale=0.999)
        result.append((safe_pt, disjont_root))
    #print ("len result", len(result))
    return result
    
def get_critical_linesegments(skeleton_path, visited_sights:Sight, robot_vision):
    critical_linesegments = []
    # get safe radius to avoid disjoint among line segments
    safe_radius = get_safe_radius(skeleton_path, robot_vision)
    #safe_radius = robot_vision

    for i in range(1, len(skeleton_path) - 1):
        pre_pt = skeleton_path[i - 1]  # pre point
        post_pt = skeleton_path[i + 1]  # post point
        center_pt = skeleton_path[i]  # center point

        # get closed sights according to its center
        closed_sights = visited_sights.get_closed_sights(center_pt)
        
        local_ls = get_sorted_local_linesegments(closed_sights, center_pt, pre_pt, post_pt,robot_vision, safe_radius)

        critical_linesegments.extend(local_ls)
    return critical_linesegments

def get_critical_linesegments_old(skeleton_path, visited_sights:Sight, robot_vision):
    critical_linesegments = []
    # get safe radius to avoid disjoint among line segments
    safe_radius = get_safe_radius(skeleton_path, robot_vision)
    # safe_radius = robot_vision

    for i in range(1, len(skeleton_path) - 1):
        pre_pt = skeleton_path[i - 1]  # pre point
        post_pt = skeleton_path[i + 1]  # post point
        center_pt = skeleton_path[i]  # center point

        # get closed sights according to its center
        closed_sights = visited_sights.get_closed_sights(center_pt)

        local_ls = []
        base_edge = np.subtract(center_pt, pre_pt)
        for pair in closed_sights:
            pt0 = list(pair[0])
            pt1 = list(pair[1])
            in_side, _ = inside_angle_area(pt0, center_pt, (pre_pt, post_pt))
            if in_side:
                buff_ls0 = np.subtract(center_pt, pt0)
                buff_ls1 = np.subtract(center_pt, pt1)
                angle0 = abs(signed_angle(base_edge, buff_ls0))
                angle1 = abs(signed_angle(base_edge, buff_ls1))
                pt0 = get_disjoint_ls(center_pt, pt0, safe_radius)
                pt1 = get_disjoint_ls(center_pt, pt1, safe_radius)
                local_ls.append([angle0, pt0, center_pt])
                local_ls.append([angle1, pt1, center_pt])
        # print ("local_ls ", local_ls)
        local_ls.sort()

        # remove duplicate (mutual) line segment
        i = 0
        while i < len(local_ls) - 1:
            # if 2 angles are close then remove 1
            if math.isclose(local_ls[i][0], local_ls[i + 1][0]):
                local_ls.pop(i)
                continue
            i = i + 1

        # if there is no local across line, create a fake across ls
        if len(local_ls) == 0:
            # if 3 point makes a straight line
            if belong_line(center_pt, (pre_pt, post_pt)):
                vector_c_pre = np.subtract(pre_pt,  center_pt )
                vector_c_post = np.subtract(post_pt, center_pt)
                ptA = rotate_vector(vector_c_pre, math.pi/2)
                ptB = rotate_vector(vector_c_post, math.pi/2)
                ptA = np.add(ptA, center_pt)
                ptB = np.add(ptB, center_pt)
                local_ls.append([0, ptA, ptB])

            else:
                midpt = get_middle_direction(center_pt, robot_vision, (pre_pt, post_pt))
                midpt = get_disjoint_ls(center_pt, midpt, safe_radius)
                local_ls.append([0, midpt, center_pt])
        #print ("local ls", local_ls)
        # disjoint line segment
        for ls in local_ls:
            _, disjoint_root = scale_vector(begin=ls[1], end=ls[2], scale=0.999)
            ls[2] = disjoint_root
        critical_linesegments.extend(local_ls)
    return critical_linesegments
def total_length(path_length):
    temp = np.array(path_length)
    return temp.sum()

def approximately_sp_ls(critical_ls, spt, gpt):
    #print ("critical_ls", critical_ls)
    path = []  # list of points
    path_dist = []
    pre_total_dist = float('inf')
    total_dist = 0.0
    if len(critical_ls) > 0:
        # initialize the path by choosing mid points of critical line segment
        path.append(spt)  # start point
        for ls in critical_ls:
            pt = mid_point(ls[0], ls[1])
            path.append(pt)  # middle point of critical line segments
        path.append(gpt)  # end point

        # calculate path_dist
        for i in range (len(path)-1):
            path_dist.append(point_dist(path[i], path[i+1]))
        pre_total_dist = total_length(path_dist)

        for j in range(1000):
            for i in range(1, len(path) - 1):
                # find cross point of p(n-1), p(n) and p(n+1)
                line1 = (path[i - 1], path[i + 1])
                line2 = critical_ls[i - 1]
                pn_new = line_across(line1, line2)
                if pn_new is not None:
                    path[i] = pn_new
                else:
                    d1 = point_dist(path[i - 1], critical_ls[i - 1][0]) + point_dist(path[i + 1], critical_ls[i - 1][0])
                    d2 = point_dist(path[i - 1], critical_ls[i - 1][1]) + point_dist(path[i + 1], critical_ls[i - 1][1])
                    if d1 > d2:
                        path[i] = critical_ls[i - 1][1]
                    else:
                        path[i] = critical_ls[i - 1][0]
                path_dist[i-1] = point_dist(path[i-1],path[i])
                path_dist[i] = point_dist(path[i],path[i+1])
            total_dist = total_length(path_dist)
            if total_dist >= pre_total_dist:
                break
            pre_total_dist = total_dist
    return path

def approximately_sp_ls_old(critical_ls, spt, gpt):
    #print ("critical_ls", critical_ls)
    path = []  # list of points
    path_dist = []
    pre_total_dist = float('inf')
    total_dist = 0.0
    if len(critical_ls) > 0:
        # initialize the path by choosing mid points of critical line segment
        path.append(spt)  # start point
        for ls in critical_ls:
            pt = mid_point(ls[1], ls[2])
            path.append(pt)  # middle point of critical line segments
        path.append(gpt)  # end point

        # calculate path_dist
        for i in range (len(path)-1):
            path_dist.append(point_dist(path[i], path[i+1]))
        pre_total_dist = total_length(path_dist)

        for j in range(1000):
            for i in range(1, len(path) - 1):
                # find cross point of p(n-1), p(n) and p(n+1)
                line1 = (path[i - 1], path[i + 1])
                line2 = critical_ls[i - 1][1:3]
                pn_new = line_across(line1, line2)
                if pn_new is not None:
                    path[i] = pn_new
                else:
                    d1 = point_dist(path[i - 1], critical_ls[i - 1][1]) + point_dist(path[i + 1], critical_ls[i - 1][1])
                    d2 = point_dist(path[i - 1], critical_ls[i - 1][2]) + point_dist(path[i + 1], critical_ls[i - 1][2])
                    if d1 > d2:
                        path[i] = critical_ls[i - 1][2]
                    else:
                        path[i] = critical_ls[i - 1][1]
                path_dist[i-1] = point_dist(path[i-1],path[i])
                path_dist[i] = point_dist(path[i],path[i+1])
            total_dist = total_length(path_dist)
            if total_dist >= pre_total_dist:
                break
            pre_total_dist = total_dist
    return path


'''
    check if local open points are inside active arc (arc_limA, arc_limB)
'''
def is_inside_active_arc(local_open_pts, robot_vision, center, goal):
    # find 2 intersection points (arc_limA, arc_limB) between 2 circles (center, robot_vision)
    #  and (goal, robot_to_goal)
    
    robot_to_goal = point_dist(center, goal)
    if robot_to_goal > robot_vision:
        arc_limA, arc_limB = get_intersections_2circles(center, robot_vision, goal, robot_to_goal)

        inside_active_arc = [inside_angle_area(pt, center, (arc_limA, arc_limB))[0] for pt in local_open_pts]
        local_active_open_pts = local_open_pts[inside_active_arc]
        return local_active_open_pts, np.array((arc_limA, arc_limB))
    else:
        return None, None

    ''' calculate path cost'''
def path_cost(path):
    cost = 0.0
    for i in range(len(path)-1):
        cost += point_dist(path[i], path[i+1])
    return cost