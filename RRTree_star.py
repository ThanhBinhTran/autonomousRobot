from Obstacles import Obstacles
from Program_config import *
from RRT_user_input import RRTree_user_input
from RRTree import RRTree
from Robot_class import Robot
from Robot_math_lib import *
from Tree import Node

''' plotter lib '''
from Plotter import Plotter


class RRTree_star(RRTree):
    """ RRTree class from Tree class """

    def __init__(self, root, step_size=5, radius=5, random_area=(0, 100), sample_size=100):
        super().__init__(root, step_size, radius, random_area, sample_size)

    ''' add node to RRTree , return new_node and its neighbour_node(s)'''

    def add_node_RRTstar(self, accepted_coordinate):
        # check if given accepted_coordinate is exist
        n = self.get_node_by_coords(accepted_coordinate)
        if n is not None:
            return n, None, None

        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(accepted_coordinate, self.radius)
        # pick nearest neighbour as new_node's parent
        nearest_neighbour_node = self.neighbours_smallest_cost(accepted_coordinate, neighbour_nodes)

        if nearest_neighbour_node is not None:  # found a neighbour to link to
            # allocate new node then link to RRTree
            new_node = Node(accepted_coordinate)
            new_node.add_neighbours(neighbours=neighbour_nodes)
            self.add_node(new_node=new_node)
            self.add_edge(parent_node=nearest_neighbour_node, node=new_node)
            return new_node, neighbour_nodes, nearest_neighbour_node
        return None, None, None

    def add_coordinate_to_RRTstar(self, robot, coordinate, obstacles, goal_coordinate, ignore_obstacles=False):

        # bring closer random coordinate to tree 
        accepted_coordinate = self.bring_closer(rand_coordinate=coordinate, obstacles=obstacles,
                                                ignore_obstacles=ignore_obstacles)

        if not ignore_obstacles:
            if accepted_coordinate is None or not robot.inside_explored_area(pt=accepted_coordinate):
                return None, None, None

                # if tree first saw given goal , instead of adding new random , add goal
        if not self.reach_goal:
            nn_goal = self.saw_goal(goal_coordinate)  # return nearst neighbour node of goal
            if nn_goal is not None:  # existing a node nearby goal
                self.reach_goal = True
                accepted_coordinate = goal_coordinate

        # add and link node to tree
        new_node, neighbour_nodes, nearest_neighbour_node = self.add_node_RRTstar(accepted_coordinate)
        if new_node is None:
            return None, None, None

        ''' rewire for RRT* '''
        self.rewire(node=new_node, neighbour_nodes=neighbour_nodes)

        return new_node, neighbour_nodes, nearest_neighbour_node

    def build(self, goal_coordinate, plotter: Plotter = None, obstacles=None, robot: Robot = None,
              ignore_obstacles=False):
        for i in range(1, self.sampling_size):
            # orient to goal sometime :))
            if i % 50 == 0 and not self.reach_goal:  # bias to goal sometime
                rand_coordinate = np.array(goal_coordinate)
            else:
                # generate random coordinate in sampling area = [min, max]
                rand_coordinate = self.random_coordinate()
            new_node, neighbour_nodes, nearest_neighbour_node = \
                self.add_coordinate_to_RRTstar(coordinate=rand_coordinate, obstacles=obstacles,
                                               ignore_obstacles=ignore_obstacles, robot=robot,
                                               goal_coordinate=goal_coordinate)
            ''' for display '''

            # update path to goal
            if self.reach_goal:
                goal_node = self.get_node_by_coords(goal_coordinate)
                self.path_to_goal = self.path_to_root(goal_node)
                self.total_goal_cost = goal_node.cost

            if show_animation:
                plotter.build_tree_animation(num_iter=i, Tree=self, obstacles=obstacles, goal_coords=goal_coordinate,
                                             start_coords=self.root.coords, rand_coordinate=rand_coordinate,
                                             rand_node=new_node,
                                             neighbour_nodes=neighbour_nodes,
                                             nearest_neighbour_node=nearest_neighbour_node,
                                             color_tree=TreeColor.by_cost)


if __name__ == '__main__':
    ''' initial parameters '''
    # get user input
    menu_result = RRTree_user_input()
    # get start_coordinate and goal_coordinate
    start_coordinate = menu_result.s
    goal_coordinate = menu_result.g
    step_size = menu_result.step_size
    radius = menu_result.radius
    node_density = menu_result.d
    map_name = menu_result.m
    vision_range = menu_result.r

    print (start_coordinate)
    print (goal_coordinate)
    ''' Running '''
    # set same window size to capture pictures
    plotter = Plotter(title="Rapidly-exploring Random Tree Star (RRT*)")
    plotter.set_equal()

    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(map_name=map_name)
    obstacles.line_segments()

    # find working space boundary
    robot = Robot(start=start_coordinate, goal=goal_coordinate, vision_range=vision_range)
    # find working space boundary
    boundary_area = robot.find_working_space_boundaries(obstacles=obstacles)
    RRT_sample_size = int( (boundary_area[0][1]-boundary_area[0][0]) * (boundary_area[1][1]-boundary_area[1][0])/node_density)


    ''' build tree '''
    start_node = Node(start_coordinate, cost=0)  # initial root node, cost to root = 0
    RRT_star = RRTree_star(root=start_node, step_size=step_size, radius=radius,
                           random_area=boundary_area, sample_size=RRT_sample_size)
    RRT_star.build(goal_coordinate=goal_coordinate, plotter=plotter, obstacles=obstacles, robot=robot)

    plotter.show_map(obstacles=obstacles)
    plotter.tree(RRT_star, color_mode=TreeColor.by_cost)
    plotter.show()
