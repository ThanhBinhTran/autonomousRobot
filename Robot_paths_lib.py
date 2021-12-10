import numpy as np
from sys import float_info
from Robot_lib import *
from Robot_sight_lib import inside_global_true_sight
from collections import defaultdict

from Robot_draw_lib import *


def motion(current_position, next_pt):
    '''
    motion model
    '''
    current_position[0] = approximately_num(next_pt[0])
    current_position[1] = approximately_num(next_pt[1])
    return current_position


def ranking_score(angle, distance):
    # ranking = alpha/(distance**2) + beta/abs(angle**2)
    alpha = 0.9
    beta = 0.1
    angle = angle / math.pi
    distance = distance / 150
    # print ("Ranking score: angle {0}, distance {1}".format(angle, distance) )
    if not math.isclose(angle, 0.0) and not math.isclose(distance, 0.0):
        r_score = alpha / (distance) + beta / abs(angle)
    else:
        r_score = float_info.max
    return r_score


def ranking(center, pt, goal):
    '''
    score the open point by its angle (from center to point and goal) and its distance (to goal)
    '''

    sa = signed_angle(goal - center, pt - center)
    dist = point_dist(goal, pt)
    rank_score = ranking_score(sa, dist)
    return [rank_score]


def pick_next(ao_gobal):
    '''
    return index and value of next point if there exist any active open point
    otherwise -1
    '''
    pick_idx = -1
    next_pt = []
    if len(ao_gobal) > 0:
        ranks = ao_gobal[:, 2]
        # print ("global open points: ", ao_gobal[:,0:2])
        # print ("global ranks: ",ranks)
        # print ("pick index ", np.argmax(ranks))
        # print ("picked rank ", np.amax(ranks))
        pick_idx = np.argmax(ranks)
        next_pt = ao_gobal[pick_idx, 0:2]

    return pick_idx, next_pt


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


def get_possible_AH_next_points_from_point(AH_points, obstacle_list, startpoint, goal):
    # return [start point, [all next possible next point], [add blind points from AH_points], reach goal = true/false]
    possible_AH_next_points = []
    blind_point = []
    goal_apprear = False
    print("start point .........", startpoint)
    for i in range(len(obstacle_list[0])):  # get number of column
        [a, b, c] = line_from_points(startpoint, obstacle_list[:, i])
        # print ("From point ", obstacle_list[:,i])
        get_dist = all_remaining_point_same_side(a, b, c, obstacle_list)
        if get_dist:
            # Check if selected point is existed in the path
            point = (obstacle_list[0, i], obstacle_list[1, i])
            if point not in AH_points:
                print("__Linked ", obstacle_list[:, i])
                # concatenate path to matrix path
                AH_points.append((obstacle_list[0, i], obstacle_list[1, i]))
                print("__PATH  ", AH_points)
                # plt.plot((startpoint[0],obstacle_list[0][i]), (startpoint[1], obstacle_list[1][i]), "--r")
                # plt.plot(obstacle_list[0][i], obstacle_list[1][i], "or")
                possible_AH_next_points.append(obstacle_list[:, i])
        else:
            blind_point.append(obstacle_list[:, i])
            # plt.plot((startpoint[0],obstacle_list[0][i]), (startpoint[1], obstacle_list[1][i]), "--g")
        # plt.pause(0.1)

    [a, b, c] = line_from_points(startpoint, goal)
    # print ("Check the goal ", obstacle_list[:,i])
    goal_apprear = all_remaining_point_same_side(a, b, c, obstacle_list)

    return [startpoint, possible_AH_next_points, blind_point, goal_apprear]


def find_AH_paths(ox_b, oy_b, startpoint, goal):
    obstacle_list = np.array([ox_b, oy_b])
    print(obstacle_list)
    AH_paths = []
    AH_points = []
    AH_points.append(startpoint)
    for start_point in AH_points:
        AH_path_temp = get_possible_AH_next_points_from_point(AH_points, obstacle_list, start_point, goal)
        AH_paths.append(AH_path_temp)
    return AH_paths


# Function to build the graph 
def build_graph(edges):
    edges
    graph = defaultdict(list)

    # Loop to iterate over every  
    # edge of the graph 
    for edge in edges:
        a, b = edge[0], edge[1]

        # Creating the graph  
        # as adjacency list 
        graph[a].append(b)
        graph[b].append(a)
    return graph


# Function to initialize a graph
def graph_intiailze():
    return defaultdict(list)


# Function to add local active point to graph
def graph_add_lOpenPts(graph, center, lActive_OpenPts):
    if len(lActive_OpenPts) > 0:
        graph_insert(graph, center, lActive_OpenPts)


# Function to insert edges into graph
def graph_insert(graph, pnode, leafs):
    # print ("__pnode:", pnode)
    # print ("__leafs:", leafs)
    if len(leafs) > 0:
        for leaf in leafs:
            graph[tuple(pnode)].append(tuple(leaf))
            graph[tuple(leaf)].append(tuple(pnode))

        # Function to find the shortest


# path between two nodes of a graph
def BFS_skeleton_path(graph, start, goal):
    print("Current {0}, Next {1}".format(start, goal))
    # print ("graph", graph)
    explored = []

    # Queue for traversing the  
    # graph in the BFS 
    queue = [[start]]

    # If the desired node is  
    # reached 
    if start == goal:
        print("Same Node")
        return start

    # Loop to traverse the graph  
    # with the help of the queue 
    while queue:
        path = queue.pop(0)
        node = path[-1]

        # Condition to check if the 
        # current node is not visited 
        if node not in explored:
            neighbours = graph[node]
            # print ("_NODE:", node)
            # Loop to iterate over the  
            # neighbours of the node 
            for neighbour in neighbours:
                # print ("___neighbour:", neighbour)
                new_path = list(path)
                new_path.append(neighbour)
                queue.append(new_path)

                # Condition to check if the  
                # neighbour node is the goal 
                if neighbour == goal:
                    print("BFS_skeleton_path = ", new_path)
                    return new_path
            explored.append(node)

            # Condition when the nodes
    # are not connected 
    print("So sorry, but a connecting path doesn't exist :(")
    return []


def approximately_shortest_path(skeleton_path, traversal_sight, robot_vision):
    if len(skeleton_path) <= 2:
        return skeleton_path, []
    else:
        print("skeleton_path:", skeleton_path)
        spt = skeleton_path[0]
        gpt = skeleton_path[-1]
        critical_ls = get_critical_ls(skeleton_path, traversal_sight, robot_vision)
        return approximately_sp_ls(critical_ls, spt, gpt), critical_ls


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


def get_disjoint_ls(c_pt, pt, safe_radius):
    safe_pt = pt
    if point_dist(c_pt, pt) > safe_radius:
        vector = np.subtract(pt, c_pt)
        uvector = unit_vector(vector)
        direction_vector = np.multiply(uvector, safe_radius)
        safe_pt = np.add(c_pt, direction_vector)
    return list(safe_pt)


def get_critical_ls(skeleton_path, traversal_sight, robot_vision):
    critical_ls = []
    # get safe radius to avoid disjoint among line segments
    safe_radius = get_safe_radius(skeleton_path, robot_vision)
    # safe_radius = robot_vision

    for i in range(1, len(skeleton_path) - 1):
        pre_pt = skeleton_path[i - 1]  # pre point
        post_pt = skeleton_path[i + 1]  # post point
        c_pt = skeleton_path[i]  # center point
        # query true sight for center node
        tsight = []
        for ts in traversal_sight:
            if c_pt == ts[0]:
                tsight = ts[1]
        # print ("true closed sight for current point:", tsight)
        local_ls = []
        base_edge = np.subtract(c_pt, pre_pt)
        for pair in tsight:
            pt0 = list(pair[0])
            pt1 = list(pair[1])
            in_status, _ = inside_angle_area(pt0, c_pt, (pre_pt, post_pt))
            if in_status:
                buff_ls0 = np.subtract(c_pt, pt0)
                buff_ls1 = np.subtract(c_pt, pt1)
                angle0 = abs(signed_angle(base_edge, buff_ls0))
                angle1 = abs(signed_angle(base_edge, buff_ls1))
                pt0 = get_disjoint_ls(c_pt, pt0, safe_radius)
                pt1 = get_disjoint_ls(c_pt, pt1, safe_radius)
                local_ls.append([angle0, pt0, c_pt])
                local_ls.append([angle1, pt1, c_pt])
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
            midpt = get_middle_direction(c_pt, robot_vision, (pre_pt, post_pt))
            local_ls.append([0, midpt, c_pt])

        critical_ls.extend(local_ls)
        print("critical_ls", critical_ls)
    return critical_ls


def approximately_sp_ls_B(critical_ls, spt, gpt):
    i = 0
    path = []  # list of points
    pre_total_dist = float('inf')
    total_dist = 0.0
    if len(critical_ls) > 0:
        # initialize the path by choosing mid points of critical line segment
        path.append(spt)  # start point
        for ls in critical_ls:
            pt = midpoint(ls[1], ls[2])
            path.append(pt)  # middle point of critical line segments
            i = i + 1
        path.append(gpt)  # end point

        for j in range(100):
            midpts = []
            for i in range(len(path) - 1):
                midpts.append(midpoint(path[i], path[i + 1]))

            # find all new points for the path
            for i in range(len(midpts) - 1):
                line1 = (midpts[i], midpts[i + 1])
                line2 = critical_ls[i][1:3]
                # find cross point of 2 line midpoints(i, i +1) and critical ls [i]
                pn_new = line_across(line1, line2)
                if pn_new is not None:
                    path[i + 1] = pn_new
                else:
                    dist1 = point_dist(path[i], critical_ls[i][1])
                    dist2 = point_dist(path[i], critical_ls[i][2])
                    if dist1 < dist2:
                        path[i + 1] = critical_ls[i][1]
                    else:
                        path[i + 1] = critical_ls[i][2]
    return path


def approximately_sp_ls(critical_ls, spt, gpt):
    i = 0
    path = []  # list of points
    pre_total_dist = float('inf')
    total_dist = 0.0
    if len(critical_ls) > 0:
        # initialize the path by choosing mid points of critical line segment
        path.append(spt)  # start point
        for ls in critical_ls:
            pt = midpoint(ls[1], ls[2])
            path.append(pt)  # middle point of critical line segments
            i = i + 1
        path.append(gpt)  # end point

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
    return path


'''
    get local open points
'''


def get_local_open_points(open_sights):
    local_open_pts = []
    if len(open_sights) > 0:
        open_sights = np.array(open_sights)
        local_open_pts = open_sights[:, 2]  # local_openPts
        #print("local_openPts,", local_open_pts)
        for i in range(len(local_open_pts)):
            local_open_pts[i][0] = approximately_num(local_open_pts[i][0])
            local_open_pts[i][1] = approximately_num(local_open_pts[i][1])
    return local_open_pts


'''
    check whether local open_points are active 
'''

def get_active_open_points(local_open_pts, traversal_sights, robot_vision, center, goal):
    local_active_open_pts = []
    if len(local_open_pts):  # new local found
        local_active_open_pts = local_open_pts

        # remove local_point which is inside explored area
        if len(traversal_sights) > 0:
            local_open_pts_status = [inside_global_true_sight(pt, robot_vision, traversal_sights) for pt in local_active_open_pts]
            local_active_open_pts = local_active_open_pts[np.logical_not(local_open_pts_status)]

    return local_active_open_pts

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

'''
    add local active and its ranking to global active points set
'''


def store_global_active_points(g_active_open_pts, l_active_open_pts, ranking_score):
    if len(l_active_open_pts) > 0:
        local_active_open_pts_info = np.concatenate((l_active_open_pts, ranking_score), axis=1)
        if len(g_active_open_pts) == 0:
            g_active_open_pts = np.array(local_active_open_pts_info)
        else:
            g_active_open_pts = np.concatenate((g_active_open_pts, local_active_open_pts_info), axis=0)
    return g_active_open_pts
