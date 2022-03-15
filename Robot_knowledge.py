from Robot_paths_lib import *
from Robot_lib import *
from Robot_sight_lib import inside_local_true_sight, inside_global_true_sight

class Robot_knowledge:
    def __init__(self, start, vision_range, robot_radius= 0.2): 
        self.coordinate = tuple(start)      # hold current coordinate of robot
        self.next_coordinate = tuple(start) # hold next coordinate where robot moves to
        self.vision_range = vision_range
        self.robot_radius = robot_radius

        self.reach_goal = False             # True if robot reach goal
        self.saw_goal = False               # True if robot saw goal
        self.no_way_to_goal = False         # True if there is no path to goal
        
        self.local_open_pts = []            # local open point
        self.local_active_open_pts = []     # local active open point
        self.global_active_open_pts = []    # global active open points
        self.next_point = []                # next point where robot move to

        self.traversal_sights = []          # hold traversal sights that robot visited
        self.visited_path = []              # path that robot visited
        
        # visibility Graph which contains information of visited places
        self.visibility_graph = graph_intiailze()
    def is_no_way_to_goal(self, noway):
        self.no_way_to_goal = noway

    def update_coordinate(self, coords):
        self.coordinate = coords

    def clear_next_point(self):
        self.next_point = []

    def update_coordinate(self, point):
        self.coordinate = point

    def expand_traversal_sights(self, closed_sights, open_sights):
        self.traversal_sights.append([self.coordinate, closed_sights, open_sights])

    def print_traversal_sights(self):
        print("traversal_sights:", self.traversal_sights)

    def expand_visited_path(self, path):
        self.visited_path.append(path)

    def print_visited_path(self):
        print("visited path:", self.visited_path)

    ''' Check if robot saw goal '''
    def is_saw_goal(self, goal, true_sight):
        self.saw_goal = inside_local_true_sight(goal, self.coordinate, self.vision_range, true_sight)

    ''' Check if robot reached goal '''
    def is_reach_goal(self, goal):
        self.reach_goal = point_dist(self.coordinate, goal) <= self.robot_radius

    ''' Check if robot whether reached or saw goal '''
    def check_goal(self, goal, true_sight):
        
        self.is_reach_goal(goal)
        if not self.reach_goal:
            self.is_saw_goal(goal, true_sight)
    ''' print status of robot to goal '''
    def show_status(self):
        print ("_robot reached goal: {0}, saw goal: {1}".format(self.reach_goal, self.saw_goal))

    ''' get local open points '''
    def get_local_open_points(self, open_sights):
        self.local_open_pts = []
        if len(open_sights) > 0:
            open_sights = np.array(open_sights)
            self.local_open_pts = open_sights[:, 2]  # local_openPts
            #print("local_openPts,", local_open_pts)
            for i in range(len(self.local_open_pts)):
                self.local_open_pts[i][0] = approximately_num(self.local_open_pts[i][0])
                self.local_open_pts[i][1] = approximately_num(self.local_open_pts[i][1])
        return self.local_open_pts
    
    ''' check whether local open_points are active '''
    def get_local_active_open_points(self):
        self.local_active_open_pts = []
        if len(self.local_open_pts):  # new local found
            self.local_active_open_pts = self.local_open_pts

            # remove local_point which is inside explored area
            if len(self.traversal_sights) > 0:
                local_open_pts_status = [inside_global_true_sight(pt, self.vision_range, self.traversal_sights) for pt in self.local_active_open_pts]
                self.local_active_open_pts = self.local_active_open_pts[np.logical_not(local_open_pts_status)]

        return self.local_active_open_pts

    ''' pick next point where its ranking is heighest '''
    def pick_next_point(self):
        next_point = None
        next_pt_idx = -1
        if len(self.global_active_open_pts) > 0:
            ranks = self.global_active_open_pts[:, 2]
            next_pt_idx = np.argmax(ranks)
            next_point = self.global_active_open_pts[next_pt_idx, 0:2]

        return next_point, next_pt_idx

    ''' remove active point form active list '''
    def global_list_remove(self, point_idx):
        self.global_active_open_pts = np.delete(self.global_active_open_pts, point_idx, axis=0)