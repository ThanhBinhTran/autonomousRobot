import numpy as np
from Robot_lib import *
from collections import defaultdict

def motion(current_position, next_pt):
    '''
    motion model
    '''
    current_position = (approximately_num(next_pt[0]), approximately_num(next_pt[1]))
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
    #print("BFS_skeleton_path: Current {0}, Next {1}".format(start, goal))
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
                    #print("BFS_skeleton_path = ", new_path)
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
        #print("critical_ls ", critical_ls)
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
            pt = mid_point(ls[1], ls[2])
            path.append(pt)  # middle point of critical line segments
            i = i + 1
        path.append(gpt)  # end point

        for j in range(100):
            midpts = []
            for i in range(len(path) - 1):
                midpts.append(mid_point(path[i], path[i + 1]))

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
            pt = mid_point(ls[1], ls[2])
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

