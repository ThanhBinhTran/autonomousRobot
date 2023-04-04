from Robot_paths_lib import *
from Robot_math_lib import *
from Robot_sight_lib import *
from Robot_base import Robot_base
from Program_config import *
from Graph import Graph
from Sight import Sight
import platform
import ctypes
import glob

if platform.system() == 'Linux':
    # find the shared library, the path depends on the platform and Python version
    libfile = glob.glob('build/*/cgal_intersection_tri*.so')[0]

    # 1. open the shared library
    mylib = ctypes.CDLL(libfile)

    # 2. tell Python the argument and result types of function main
    mylib.main.restype = ctypes.c_int
    mylib.main.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64), np.ctypeslib.ndpointer(dtype=np.float64),
                np.ctypeslib.ndpointer(dtype=np.float64), np.ctypeslib.ndpointer(dtype=np.float64)]


#from RRTree_star import RRTree_star
class Robot(Robot_base):
    def __init__(self, start, goal, vision_range=20, robot_type= Robot_base.RobotType.circle, robot_radius= 0.2):

        super().__init__(vision_range=vision_range, robot_type=robot_type, robot_radius=robot_radius)

        self.start = start                  # start point
        self.goal = goal                    # goal point
        self.cost = 0                       # cost of visited path
        self.coordinate = tuple(start)      # hold current coordinate of robot
        self.next_coordinate = tuple(start) # hold next coordinate where robot moves to
        self.next_point = 0,0               # for display only
        self.reach_goal = False             # True if robot reach goal
        self.saw_goal = False               # True if robot saw goal
        self.no_way_to_goal = False         # True if there is no path to goal
        self.path_look_ahead_to_goal = []   # path look a head to goal (estimate)
        
        #self.local_open_pts = []            # local open point
        self.local_active_open_pts = []     # local active open point
        self.local_active_open_rank_pts = []     # local active open point and its ranking
        self.global_active_open_rank_pts = []    # global active open points and its ranking

        
        self.visited_paths = []              # path where robot visited
        self.visited_path_directions = []    # status of visited subpath, true = forward, false = backward

        # visibility Graph containing information of visited places
        self.visibility_graph = Graph()
        
        # visited sights
        self.visited_sights = Sight()          # hold traversal sights where robot visited

        # skeleton_path, line_segment, approximate shortest path
        self.skeleton_path = []
        self.ls = []
        self.asp = []

    ''' find working space boundaries'''
    def find_working_space_boundaries(self, obstacles):
        # find working space boundary
        x_min = min(obstacles.x_lim[0], obstacles.y_lim[0], self.start[0], self.goal[0]) - self.vision_range/2
        x_max = max(obstacles.x_lim[1], obstacles.y_lim[1], self.start[1], self.goal[1]) + self.vision_range/2
        y_min = min(obstacles.x_lim[0], obstacles.y_lim[0], self.start[0], self.goal[0]) - self.vision_range/2
        y_max = max(obstacles.x_lim[1], obstacles.y_lim[1], self.start[1], self.goal[1]) + self.vision_range/2
        return ([x_min, y_min], [x_max, y_max])

    def is_no_way_to_goal(self, noway):
        self.no_way_to_goal = noway

    def update_coordinate(self, coords):
        self.coordinate = coords

    def add_visited_sights(self, closed_sights, open_sights):
        self.visited_sights.add_sight(center=self.coordinate, closed_sights=closed_sights,
                                        open_sights= open_sights)
    
    ''' expand visited path and its cost also '''
    def expand_visited_path(self, path):
        '''
        set direction: true means robot goes forward (len (asp) ==2)
                       false means robot back to point in explored area
        
        '''
        if len(path)> 0:
            self.visited_paths.append(path)

            direction = len(path) == 2
            self.visited_path_directions.append(direction)
            self.cost += path_cost(path)
    
    ''' connect nearby visited nodes '''
    def bridge_visibility_graph(self, cur_coordinate, cur_open_sights):
        # get all visited node
        center_pts_x = []
        center_pts_y = []
        is_tri_pts_x = []
        is_tri_pts_y = []
        
        # get all visited nodes
        all_visited_nodes = np.array(self.visibility_graph.get_all_non_leaf())
        
        # if there are visited nodes
        if len(all_visited_nodes) > 0:
        
            # get nearby node (distance < 2 range) from all visited node
            node_distance = np.array([point_dist(pt, cur_coordinate) for pt in all_visited_nodes])
            # get all nodes in closed range (0, 2*r) 
            mask = np.logical_and(node_distance > 0 ,node_distance < self.vision_range*2 ) 
            neighbor_nodes = all_visited_nodes[mask]
            
            connected_neighbors = self.visibility_graph.get_neighbor_nodes(cur_coordinate)
            
            # find intersection between nearby node and new coorindate
            for neighbor_node in neighbor_nodes:
                # flag to skip if connecting point is found
            
                skip = False
                # get open sight of nearby_node
                neighbor_center = neighbor_node
                for pt in connected_neighbors:
                    pd = point_dist(pt, neighbor_center)
                    if math.isclose(pd, 0.0):
                        skip = True
                        break
                if skip:
                    continue

                open_sights = self.visited_sights.get_open_sights(center=neighbor_center)

                # continue if there are no open sights
                if open_sights is None:
                    continue
                    
                for osight in open_sights:
                    # get open sight of new node
                    for cur_osights in cur_open_sights:
                        #if not inside_status:
                        pts_triA = np.array([neighbor_center[0],neighbor_center[1],
                                                osight[0][0], osight[0][1], osight[1][0], osight[1][1]], np.double)
                        pts_triB = np.array([cur_coordinate[0],cur_coordinate[1],
                                                cur_osights[0][0], cur_osights[0][1], cur_osights[1][0],cur_osights[1][1]], np.double)

                        pts_data = np.array([0,0, 0,0, 0,0, 0,0, 0,0, 0,0], np.double) # maximun of 6 points of intersection of 2 triangles
                        pt_centre = np.array([0,0], np.double) # centre of mass of intersection polygon if found.
                        # pts_data is initialed of max of 12 values  0.0 (= max 6 intersection points)
                        # result will holded a length of intersection pts_data
                        result = mylib.main(pts_triA, pts_triB, pts_data, pt_centre)
                        if result > 0:
                            pt_centre = tuple(pt_centre)
                            #print ("result--------------", result)
                            #print ("pts_triA--------------", pts_triA)
                            #print ("pts_triB--------------", pts_triB)
                            #print ("pts_data--------------", pts_data)
                            #print ("pt_centre--------------", pt_centre)
                                
                            # for display only
                            center_pts_x.append(pt_centre[0])
                            center_pts_y.append(pt_centre[1])
                            for i in range (result):
                                is_tri_pts_x.append(pts_data[2*i])
                                is_tri_pts_y.append(pts_data[2*i+1])

                            
                            self.visibility_graph.graph_create_edge(pt_centre, neighbor_center)
                            self.visibility_graph.graph_create_edge(pt_centre, cur_coordinate)

                            csights = []
                            for i in range(result):
                                ptA = (pts_data[2*i],pts_data[2*i+1])
                                if i == (result-1):
                                    ptB =(pts_data[0],pts_data[1])
                                else:
                                    ptB = (pts_data[2*i+2],pts_data[2*i+3])
                                   
                                if line_across(line1=(ptA, ptB), line2=(pt_centre, neighbor_center)) is not None:
                                    continue
                                    
                                if line_across(line1=(ptA, ptB), line2=(pt_centre, cur_coordinate)) is not None:
                                    continue
                                csights.append([ptA,ptB])
                            closed_sights = sort_closed_sights(center=pt_centre, closed_sights=csights)
                                
                            #dictionary can not get ndarray as key cause unhashbale
                                
                            self.visited_sights.add_sight(center=pt_centre, closed_sights=closed_sights,
                                    open_sights= [])
                            skip = True
                            break
                    if skip:
                        break
        return center_pts_x, center_pts_y, is_tri_pts_x, is_tri_pts_y

    ''' Check if robot saw goal '''
    def is_saw_goal(self, goal, closed_sights):
        self.saw_goal = inside_local_sights(goal, self.coordinate, self.vision_range, closed_sights)

    ''' Check if robot reached goal '''
    def is_reach_goal(self, goal):
        self.reach_goal = point_dist(self.coordinate, goal) <= self.radius

    ''' Check if robot whether reached or saw goal '''
    def check_goal(self, goal, closed_sights):
        
        self.is_reach_goal(goal)
        if not self.reach_goal:
            self.is_saw_goal(goal, closed_sights)

    ''' print status and information of robot '''
    def print_infomation(self):
        if self.no_way_to_goal:
            print ("NO path to goal!")
        elif self.reach_goal:
            print ("Reached goal!")
        elif self.saw_goal:
            print ("Saw goal!")


    def finish(self):
        return self.no_way_to_goal or self.reach_goal

    ''' clear local information '''
    def clear_local(self):
        self.local_open_pts = []
        self.local_active_open_pts = []
        self.local_active_open_rank_pts = []
        self.next_point = []
        
    ''' get local open points '''
    def get_local_open_points_and_rank_by_RRTree_start(self, open_sights, nodes):
        self.local_open_pts = []
        if len(open_sights) > 0:
            open_sights = np.array(open_sights)
            self.local_open_pts = open_sights[:, 2]  # local_openPts
            for i in range(len(self.local_open_pts)):
                self.local_open_pts[i][0] = approximately_num(self.local_open_pts[i][0])
                self.local_open_pts[i][1] = approximately_num(self.local_open_pts[i][1])
        return self.local_open_pts

    ''' get local open points '''
    def get_local_open_points(self, open_sights):
        # open_sights is a list:

        self.local_open_pts = []
        if len(open_sights) > 0:
            for open_sight in open_sights:
                o_pt = approximately_num(open_sight[2][0]), approximately_num(open_sight[2][1])
                self.local_open_pts.append(o_pt)
        return self.local_open_pts
    
    ''' check whether local open_points are active '''
    def get_local_active_open_points(self):

        # get local active point for local points
        self.local_active_open_pts = []
        if len(self.local_open_pts):  # new local found
            self.local_active_open_pts = np.array(self.local_open_pts)

            # remove local_point which is inside explored area
            if self.visited_sights.size():
                self.local_open_pts_status = [inside_visited_sights(pt, self.vision_range, self.visited_sights) for pt in self.local_active_open_pts]
                self.local_active_open_pts = self.local_active_open_pts[np.logical_not(self.local_open_pts_status)]
        

    ''' check if a point is inside explored area '''
    def inside_explored_area(self, pt):
        if self.visited_sights.size() > 0:
            return inside_visited_sights(pt, self.vision_range, self.visited_sights)
        return False

    def ranking_active_open_point(self, ranker, goal):
        ranks_new = []
        self.local_active_open_rank_pts = []
        # Ranking new active openPts then stack to global set.
        if len(self.local_active_open_pts) > 0:
            ranks_new = np.array([ranker.rank(self.coordinate, pt, goal) for pt in self.local_active_open_pts])

            # store local open points and its ranking 
            self.local_active_open_rank_pts = np.concatenate((self.local_active_open_pts, ranks_new), axis=1)

    #def ranking_by_RRTree(self, open_sights, ranker, goal, RRT_star:RRTree_star):
    def ranking_by_RRTree(self, open_sights, ranker, goal, RRT_star):
        
        # clear old data
        self.local_active_open_pts  = []
        self.rank_score = []
        self.local_active_open_rank_pts = []

        # get all neighbour nodes in radius area
        neighbour_nodes = RRT_star.neighbour_nodes(node_coordinate=self.coordinate, radius=self.vision_range)
        if len(neighbour_nodes):
            current_node = RRT_star.get_node_by_coords(self.coordinate)

            ancentor, _ = RRT_star.find_next(start_node=current_node, nodes = neighbour_nodes)

            active_open_nodes = np.array(neighbour_nodes)
            # remove local_point which is inside explored area
            if self.visited_sights.size() > 0:
                inside_status = [inside_visited_sights(node.coords, self.vision_range, self.visited_sights) for node in neighbour_nodes]
                active_open_nodes = active_open_nodes[np.logical_not(inside_status)]


            for open_sight in open_sights:
                inside_status = [inside_angle_area(ao_node.coords, self.coordinate, open_sight)[0] for ao_node in active_open_nodes]
                active_arc_nodes = active_open_nodes[inside_status]
                if len (active_arc_nodes) > 0:
                    if ancentor in active_arc_nodes:
                        self.local_active_open_pts.append(ancentor.coords)
                        self.rank_score.append(float('inf'))
                    else:   # select active arc_nodes
                        dist_c_n = [point_dist(self.coordinate, n_node.coords) for n_node in active_arc_nodes]
                        node_idx = np.argmax(dist_c_n)
                        self.local_active_open_pts.append(active_arc_nodes[node_idx].coords)
                        self.rank_score.append(1/active_arc_nodes[node_idx].cost)
            self.local_active_open_pts = np.array(self.local_active_open_pts)
            self.rank_score = np.reshape(self.rank_score, newshape=(len(self.rank_score), 1))
            if len(self.local_active_open_pts):
                self.local_active_open_rank_pts = np.concatenate((self.local_active_open_pts, self.rank_score), axis=1)

    ''' check whether local open_points are active '''
    def get_local_active_open_ranking_points(self, open_sights, ranker, goal, RRT_star=None, \
                                             open_points_type= Robot_base.Open_points_type.Open_Arcs):
        if open_points_type == Robot_base.Open_points_type.Open_Arcs:
            # get all local points from open sights
            self.get_local_open_points(open_sights)

            # get only active open point
            self.get_local_active_open_points()

            # ranking and store it to local active open ranking points
            self.ranking_active_open_point(ranker=ranker, goal=goal)
        elif open_points_type == Robot_base.Open_points_type.RRTstar:
            self.ranking_by_RRTree(open_sights, ranker, goal, RRT_star)

    ''' the active_open_rank_pts has to not empty '''
    def pick_max_ranking(self, active_open_rank_pts):
        ranks = active_open_rank_pts[:, 2]
        next_pt_idx = np.argmax(ranks)
        next_point = active_open_rank_pts[next_pt_idx, 0:2]        
        return next_point, next_pt_idx

    ''' pick next point, where its ranking is heighest, in given list '''
    def pick_next_point(self, goal, picking_strategy = Robot_base.Picking_strategy.global_first):
        next_point, next_pt_idx = None, -1  # default (none, idx = -1)

        global_len = len(self.global_active_open_rank_pts)  # Note: global set already get current local
        local_len = len(self.local_active_open_rank_pts)

        # if already saw/reach goal, next point is goal
        if self.saw_goal or self.reach_goal:
            next_point = goal
            
        else:   # pick next one in waiting set.
            # pop the next point in global set first
            if picking_strategy == Robot_base.Picking_strategy.global_first:     # picking global first
                if global_len > 0:
                    next_point, next_pt_idx = self.pick_max_ranking(self.global_active_open_rank_pts)
            
            # pop the next point in local set first, if the next local point is not exist then pick
            # in global set.
            elif picking_strategy == Robot_base.Picking_strategy.neighbor_first:     # picking local first
                if local_len > 0:
                    next_point, next_pt_idx = self.pick_max_ranking(self.local_active_open_rank_pts)
                    # global index 
                    next_pt_idx = global_len - local_len + next_pt_idx
                elif global_len > 0:
                    next_point, next_pt_idx = self.pick_max_ranking(self.global_active_open_rank_pts)

            # then remove picked point from active global open point
            if global_len > 0:
                self.remove_global_active_pts_by_index(next_pt_idx)

        return next_point

    ''' remove active point form active list '''
    def remove_global_active_pts_by_index(self, point_idx):
        self.global_active_open_rank_pts = np.delete(self.global_active_open_rank_pts, point_idx, axis=0)

    ''' remove active point form active list '''
    def remove_local_active_pts_by_index(self, point_idx):
        self.local_active_open_rank_pts = np.delete(self.local_active_open_rank_pts, point_idx, axis=0)

    ''' add local active and its ranking to global active points set '''
    def expand_global_open_ranking_points(self, local_active_open_pts):
        if len(local_active_open_pts) > 0:
            if len(self.global_active_open_rank_pts) == 0:
                self.global_active_open_rank_pts = np.array(local_active_open_pts)
            else:
                self.global_active_open_rank_pts = np.concatenate((self.global_active_open_rank_pts, local_active_open_pts), axis=0)

    ''' record look ahead path '''
    def set_look_ahead_to_goal(self, path):
        self.path_look_ahead_to_goal = path
