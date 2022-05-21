'''
autonomousRobot
reimplementing an algorithm which has been written from the following paper:
https://pdfs.semanticscholar.org/0cac/84962d0f1176c43b3319d379a6bf478d50fd.pdf
author: Binh Tran Thanh / email:thanhbinh@hcmut.edu.vn or thanhbinh.hcmut@gmail.com
'''

import numpy as np

from Tree import Node
from RRTree import RRTree
from RRT_draw_lib import Plot_RRT
from Robot_math_lib import *

from RRT_user_input import menu_RRT
from Program_config import *
from Obstacles import Obstacles
from Queue_class import Priority_queue
from Robot_class import Robot

class RRTree_x(RRTree):

    ''' RRTree class from Tree class '''
    def __init__(self, root:Node, step_size = 5, radius=5, random_area=([0, 100],[0,100]), sample_size = 100):
        super().__init__(root, step_size, radius, random_area, sample_size)

    ''' add node to RRTreeX , return new_node and its neighbour_node(s), update rhs, weight'''
    def add_node_RRTx(self, accepted_coordinate):
        
        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(accepted_coordinate, self.radius)
        # pick nearest neighbour as new_node's parent
        nearest_neighbour_node = self.neighbours_smallest_lmc(accepted_coordinate, neighbour_nodes)


        if nearest_neighbour_node is not None: # found a neighbour to link to 
            # allocate new node then link to RRTree
            new_node = Node(accepted_coordinate)
            new_node.add_neighbours(neighbours=neighbour_nodes)
            self.add_node(new_node=new_node)
            self.add_edge_RRTx(parent_node=nearest_neighbour_node, node=new_node)
            return new_node, neighbour_nodes, nearest_neighbour_node
        return None, None, None

    def build(self,  goal_coordinate, plotter: Plot_RRT=None, obstacles=None, rrt_queue=Priority_queue):
        first_saw_goal = False

        for i in range(1, self.sampling_size):

            # generate random coordinate in sampling area = [min, max]
            rand_coordinate = self.random_coordinate()
            
            # orient to goal sometime :))
            if i %100 == 0 and not self.reach_goal: # bias to goal sometime
                rand_coordinate = np.array(goal_coordinate)

            # bring closer random coordinate to tree 
            accepted_coordinate = self.bring_closer(rand_coordinate=rand_coordinate)

            # if tree first saw given goal , instead of adding new random , add goal
            if not first_saw_goal:
                nn_goal = self.saw_goal(goal_coordinate)    # return nearst neighbour node of goal
                if nn_goal is not None: # existing a node nearby goal
                    first_saw_goal = True
                    self.reach_goal = True
                    accepted_coordinate = goal_coordinate

            # add and link node to tree
            new_node, neighbour_nodes, nearest_neighbour_node  = self.add_node_RRTx(accepted_coordinate)
            
            # rewire for RRTX
            self.rewire_RRTx(node=new_node, neighbour_nodes=neighbour_nodes, rrt_queue=rrt_queue)
            # reduce inconsistency for RRTX
            self.reduce_inconsistency(rrt_queue=rrt_queue)

            if self.reach_goal:
                goal_node = self.at_node(goal_coordinate)
                self.path_to_goal = self.path_to_root(goal_node)
                self.total_goal_cost = goal_node.cost
               
            
            show_animation = False
            ''' for display '''
            if show_animation:
                plotter.build_tree_animation(num_iter= i, Tree= self, obstacles=None,  goal_coords=goal_coordinate, \
                    rand_coordinate= rand_coordinate, rand_node=new_node, neighbour_nodes=neighbour_nodes,\
                        nearest_neighbour_node=nearest_neighbour_node)


    def local_obstacle_nodes(self, nodes, obstacles: Obstacles):
        obstacle_nodes = []
        for node in nodes:
            conllision = obstacles.check_point_collision(point=node.coords,\
                            obstacles_line_segments=obstacles.obstacles_line_segments)
            if conllision:
                obstacle_nodes.append(node)
                node.set_inactive()
            else:
                node.set_visited()
        return obstacle_nodes

    def propogate_descendants(self, discovered_obstacle_nodes, obstacle_nodes):
        discovered_obstacle_nodes_children = self.all_subtree_nodes_at_node(discovered_obstacle_nodes)
        all_children = discovered_obstacle_nodes.copy()
        all_children.extend(discovered_obstacle_nodes_children)

        for orphan_node in all_children:
            nb_nodes = orphan_node.neighbours
            
            orphan_node.cost = float("inf")
            orphan_node.rhs = float("inf")
            for nb_node in nb_nodes:
                if nb_node not in all_children and nb_node not in obstacle_nodes:
                    nb_node.cost = float("inf")
                    rrt_queue.verify_queue(nb_node)

        for orphan_node in all_children:
            orphan_node_parent = orphan_node.parent
            if orphan_node_parent is not None:
                self.remove_edge(parent_node=orphan_node_parent, node=orphan_node)
        return all_children

    def update_obstacles(self, neighbour_nodes, currnode, obstacles=Obstacles, rrt_queue=Priority_queue):
        # obstacle_nodes which are nodes and belong to obstacles ares at local circle range
        # discovered_obstacle_nodes (aka orphan nodes) whose parent are dead
        # all_children are nodes, containing all discovered_obstacle_nodes and theirs discovered_obstacle_nodes's chilrend

        obstacle_nodes = self.local_obstacle_nodes(nodes=neighbour_nodes, obstacles=obstacles)

        if len(obstacle_nodes) > 0: # found obstacle ahead, then rerouting tree

            # get discovered_obstacle_nodes witch are orphan nodes and theirs parent are obstacle_nodes
            discovered_obstacle_nodes = self.add_new_obstacle(obstacle_nodes, rrtx_queue=rrt_queue)
            all_children = self.propogate_descendants(discovered_obstacle_nodes, obstacle_nodes)
            rrt_queue.verify_queue(node=currnode)
            self.reduce_inconsistency_v2(rrt_queue=rrt_queue, currnode=curr_node)
            return obstacle_nodes, discovered_obstacle_nodes, all_children
        return None, None, None

    def add_new_obstacle(self, obstacle_nodes, rrtx_queue:Priority_queue):
        discovered_obstacle_nodes = []
        orphan_nodes = []
        for node in obstacle_nodes:
            neighbour_b_nodes = node.neighbours
            for weight in node.neighbours_weight:
                weight += float("inf")
            for nb_node in neighbour_b_nodes:
                node_idx = nb_node.neighbours.index(node)
                nb_node.neighbours_weight[node_idx] += float("inf")
                if nb_node.parent == node and nb_node not in obstacle_nodes:
                    rrtx_queue.verify_orphan(nb_node)
                    discovered_obstacle_nodes.append(nb_node)
                    orphan_nodes.append(nb_node)
        return discovered_obstacle_nodes

if __name__ == '__main__':
    # get user input
    menu_result = menu_RRT()
    # get start_cooridinate and goal_coordinate
    start_cooridinate = menu_result.sx, menu_result.sy
    goal_coordinate = menu_result.gx, menu_result.gy

    step_size = menu_result.step_size
    radius = menu_result.radius
    sample_size = menu_result.ss
    map_name = menu_result.m
    num_iter = menu_result.n
    world_name = menu_result.w

    ''' variable declaration '''
    robot = Robot(start=start_cooridinate, goal=goal_coordinate, vision_range=radius)
    # set same window size to capture pictures
    plotter = Plot_RRT(title="Rapidly-exploring Random Tree X (RRT_X)")
    plotter.set_equal()


    ''' get obstacles data whether from world (if indicated) or map (by default)'''
    obstacles = Obstacles()
    obstacles.read(world_name, map_name)
    obstacles.line_segments()
    rrt_queue = Priority_queue()

    # find working space boundary
    x_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    x_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    y_min = min(obstacles.x_lim[0], obstacles.y_lim[0], start_cooridinate[0], goal_coordinate[0])
    y_max = max(obstacles.x_lim[1], obstacles.y_lim[1], start_cooridinate[1], goal_coordinate[1])
    random_area = ([x_min, y_min], [x_max, y_max])

    ''' build tree '''
    goal_node = Node(goal_coordinate, lmc=0, cost=0)    # initial goal node, rsh to goal =0, cost to goal = 0
    RRTx = RRTree_x(root=goal_node, step_size=step_size, radius=radius, 
                    random_area=random_area, sample_size=sample_size)
    RRTx.build(goal_coordinate=start_cooridinate, plotter=plotter,obstacles=obstacles, rrt_queue=rrt_queue)

    #############################################################
    # initial variables
    #############################################################
    obstacle_nodes = None
    iter_count = 0
    if RRTx.reach_goal:     # generated tree reached to start from goal
        start_node = RRTx.dict[start_cooridinate]
        curr_node = start_node
        
        while curr_node is not goal_node:
            iter_count += 1
            # update new coordinate for robot
            robot.coordinate = curr_node.coords
            neighbour_nodes = curr_node.neighbours
            if neighbour_nodes is not None:
                obstacle_nodes, discovered_obstacle_nodes, all_children = \
                    RRTx.update_obstacles(neighbour_nodes=neighbour_nodes, currnode=curr_node,\
                    obstacles=obstacles, rrt_queue=rrt_queue)

            path_look_ahead_to_goal = RRTx.path_to_root(curr_node)
            robot.set_look_ahead_to_goal(path=path_look_ahead_to_goal)
            curr_node, path = RRTx.find_next(curr_node, neighbour_nodes)
            
            
            
            if len(path) > 1:
                robot.expand_visited_path(path=path)
            
            if show_animation:
                plotter.RRT_animation(Tree=RRTx, obstacles=obstacles, robot=robot)

            # Run n times for debugging
            if  iter_count == num_iter:
                break
    else: # need rerun the tree
        print ("No path from goal to start belongs to generated tree")
        plotter.tree(RRTx, color_mode=TreeColor.by_lmc)

    plotter.show()
    print ("Done!")
    