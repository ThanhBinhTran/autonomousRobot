import numpy as np

from Tree import Tree, Node
from RRT_draw_lib import Plot_RRT
from Robot_lib import *

from RRT_user_input import menu_RRT
from Program_config import *
from Robot_csv_lib import Obstacles

class RRTree(Tree):

    """ RRTree class from Tree class """
    def __init__(self, root, step_size = 5, radius=5, random_area=(0, 100), sample_size = 100):
        super().__init__(root)
        self.step_size = step_size
        self.radius = radius
        self.sampling_area = random_area
        self.sampling_size = sample_size

    def set_step_size(self, x): self.step_size = x
    def set_radius(self, x): self.radius = x

    ''' add node to RRTree , return new_node and its neighbour_node(s)'''
    def add_node(self, rand_coordinate, i):
        # get array of all RRTree's nodes
        all_nodes = self.all_nodes()

        # find the neareset node (in distance) to random coordinate
        nearest_dist, nearest_idx = self.nearest(rand_coordinate)
        # get nearest node
        nearest_node = all_nodes[nearest_idx]

        # bring random coordinate closer to nearest node
        picked_coordinate = nearest_node.bring_closer(rand_coordinate, nearest_dist, self.step_size)

        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(picked_coordinate, self.radius)
        # pick nearest neighbour as new_node's parent
        nearest_neighbour_node = self.nearest_in_list_neighbour(picked_coordinate, neighbour_nodes)

        if nearest_neighbour_node is not None: # found a neighbour to link to 
            # allocate new node then link to RRTree
            rand_node = Node(picked_coordinate)
            self.add_edge(nearest_neighbour_node, rand_node)
            return rand_node, neighbour_nodes
        return None, None

    def build(self,  goal_node, plotter, obstacles):
        first_saw_goal = False
        reach_goal = False
        path_to_goal = []
        cost = float('inf')

        for i in range(self.sampling_size):
            # generate random coordinate in sampling area = [min, max]
            rand_coordinate = np.random.random(2)*self.sampling_area[1] + self.sampling_area[0]
            if i %100 == 0 and not reach_goal:
                rand_coordinate = np.array(goal_node.coords)

            self.add_node(rand_coordinate, i)

                
            if not first_saw_goal:
                nn_goal = self.saw_goal(goal_node)    # return nearst neighbour node of goal
                if nn_goal is not None: # existing a node nearby goal
                    self.add_edge(nn_goal, goal_node)
                    first_saw_goal = True
                    reach_goal = True
            #reach_goal = self.reach_goal(goal_node)

            if reach_goal:
                cost, path_to_goal = self.path_to_root(goal_node)
            
            ''' for display '''
            if show_animation:
                plotter.animation(i, cost, path_to_goal, self, obstacles, self.root.coords, goal_node.coords)

    ''' 
        check if the RRT contain nodes that are inside goal radius
        return None if do not see
        return nearest Node if existing a node (nearest) in goal radius 
    ''' 
    def saw_goal(self, goal_node):
        # find nearest neighbours tree's nodes where are in circle of radius
        return self.nearest_neighbour(goal_node.coords, self.radius)

    '''check if the tree contains goal node'''
    def reach_goal(self, goal_node):
        return goal_node in self
        

    def path_to_root(self, node):
        cost = self.cost(node)
        path = []
        path.append(node)
        while (node.parent): # loop whenever parent existed
            point_dist(node.coords, node.parent.coords)
            node = node.parent
            path.append(node)
        return cost, path

if __name__ == '__main__':
    ''' initial parameters '''
    # get user input
    menu_result = menu_RRT()
    # get start_cooridinate and goal_coordinate
    start_cooridinate = menu_result.sx, menu_result.sy
    goal_coordinate = menu_result.gx, menu_result.gy

    step_size = menu_result.step_size
    radius = menu_result.radius
    sample_size = menu_result.ss
    random_area = (menu_result.rx, menu_result.ry)
    map_name = menu_result.m
    world_name = None

    ''' Running '''
    # set same window size to capture pictures
    plotter = Plot_RRT(title="Rapidly-exploring Random Tree (RRT)")
    plotter.set_equal()

    start_node = Node(start_cooridinate)
    goal_node = Node(goal_coordinate)
    
    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(world_name, map_name)

    RRT = RRTree(start_node, step_size, radius, random_area, sample_size)
    RRT.build(goal_node, plotter, obstacles)
    
    fixed_point = 40,40
    neighbour_nodes = RRT.neighbour_nodes(node_coordinate=fixed_point, radius=radius)
    if neighbour_nodes is not None:
        [n_node.set_inactive() for n_node in neighbour_nodes]
    plotter.clear()
    plotter.tree(RRT)
    plotter.show_map(obstacles=obstacles)
    plotter.vision_area(fixed_point, radius)
    print (RRT.dict)
    RRT.printTree(RRT.root)
    plotter.show()
