from re import S
import numpy as np

from Tree import Node
from RRTree import RRTree
from RRT_draw_lib import Plot_RRT
from Robot_lib import *

from RRT_user_input import menu_RRT
from Program_config import *
from Obstacles import Obstacles

class RRTree_x(RRTree):

    """ RRTree class from Tree class """
    def __init__(self, root:Node, step_size = 5, radius=5, random_area=(0, 100), sample_size = 100):
        super().__init__(root, step_size, radius, random_area, sample_size)

    def build(self,  goal_node = None, plotter: Plot_RRT=None, obstacle=None):
        first_saw_goal = False
        reach_goal = False
        path_to_goal = []
        cost = float('inf')

        for i in range(self.sampling_size):
            # generate random coordinate in sampling area = [min, max]
            rand_coordinate = np.random.random(2)*self.sampling_area[1] + self.sampling_area[0]
            if i %100 == 0 and not reach_goal:
                rand_coordinate = np.array(goal_node.coords)

            new_node, neighbour_nodes  = self.add_node(rand_coordinate, i)

            ''' rewire for RRT* '''
            self.rewire(new_node, neighbour_nodes)
                
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
                plotter.animation(i, cost, path_to_goal, self, obstacle, self.root.coords, goal_node.coords)


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
    plotter = Plot_RRT(title="Rapidly-exploring Random Tree Star (RRT*)")
    plotter.set_equal()

    start_node = Node(start_cooridinate)
    goal_node = Node(goal_coordinate)

    obstacles = Obstacles()
    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles.read(world_name, map_name)

    RRTx = RRTree_x(root=goal_node, step_size=step_size, radius=radius, 
                    random_area=random_area, sample_size=sample_size)
    #start_node, step_size, radius, random_area, sample_size)
    RRTx.build(goal_node=start_node,plotter=plotter,obstacle=obstacles)

    fixed_point = 40,40
    neighbour_nodes = RRTx.neighbour_nodes(node_coordinate=fixed_point, radius=radius)
    if neighbour_nodes is not None:
        [n_node.set_inactive() for n_node in neighbour_nodes]
    plotter.clear()
    plotter.tree(RRTx)
    plotter.point_text(goal_coordinate, "*r", "goal")
    plotter.point_text(start_cooridinate, "Hr", "start")
    curent_robot_node = RRTx.find_root(start_node)
    plotter.point_text(curent_robot_node.coords, "Hg", "im here")
    
    #plotter.show_map(obstacles=obstacles)
    plotter.vision_area(fixed_point, radius)
    plotter.show()