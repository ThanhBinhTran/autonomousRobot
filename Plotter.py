from Obstacles import Obstacles

from Robot_math_lib import *
from Program_config import *
from matplotlib import patches
import matplotlib.image as mpimg
from matplotlib.collections import PatchCollection
from Queue_class import Priority_queue
from Robot_class import Robot
from Tree import Node, Tree
import numpy as np
from Plot_base_lib import Plot_base
from Graph import Graph
from Sight import Sight

from Obstacles import Obstacles
from Plot_base_lib import Plot_base
from Program_config import *
from Queue_class import Priority_queue
from Robot_class import Robot
from Tree import Node, Tree
import numpy as np

class Plotter(Plot_base):
    def __init__(self, size=(7,7), title="Path Planning Problem for an Autonomous Robot"):
        super().__init__(size, title)

    def sight(self, center, pair, cl="g", alpha=transparent, linestyle=":"):
        triangle = [center, pair[0], pair[1]]
        self.plot_triangle_fill(triangle, cl, alpha, linestyle)


    def closed_sights(self, center, closed_sights, cl="g", ls_ts="-"):
        for idx, pair in enumerate(closed_sights):
            self.sight(center, pair, cl, transparent, ls_ts)
            #self.plt.text(pair[0][0],pair[0][1], "{0}".format(idx))
    def open_sights_arc(self, center, open_sight, radius, cl="g", ls_ts="-"):
        arc_patches = []

        # center_ox: a point starts from center and follows X-axis direction 
        center_ox = np.add(center, [1,0] )
        for arc in open_sight:
            theta1radian = unsigned_angle(center, center_ox, arc[0])
            theta2radian = unsigned_angle(center, center_ox, arc[1])
            theta1 = math.degrees(theta1radian)
            theta2 = math.degrees(theta2radian)
            wedge = patches.Wedge(center, radius, theta1=theta1, theta2=theta2)
            arc_patches.append(wedge)
        collection = PatchCollection(arc_patches, facecolor=cl, linestyle='solid', edgecolor='r', alpha=transparent)
        self.ax.add_collection(collection)

    def vision_area(self, center, radius, ls=":", color="red"):
        """ draw a circle that limits the vision of robot """
        vision = self.plt.Circle(center, radius, color=color, linestyle=ls, fill=False)
        self.plt.gcf().gca().add_artist(vision)

    def vision(self, center, radius, csights, osights):
        if show_circleRange:
            self.vision_area(center, radius)

        if show_closedSight:
            self.closed_sights(center, csights, cl_ts, ls_ts)

        if show_openSight:
            self.open_sights_arc(center, osights, radius)

    def paths(self, paths, ls="-r", ls_next="-b"):
        for i in range(len(paths)):
            path = paths[i]
            if i == len(paths) - 1:
                self.RRT_path(path, ls_next)
            else:
                self.RRT_path(path, ls)

    def show_configuration_space(self, config_space: list):
        self.polygons(config_space, ls = ls_cspace)
        
    def visibility_graph(self, graph: Graph , ls_vg):
        visibility_graph = graph.graph
        for pnode in visibility_graph:
            for verteces in visibility_graph[pnode]:
                self.line_segment([pnode, verteces], ls_vg)

    def critical_line_segments(self, critical_ls, ls_cls):
        for idx, ls in enumerate(critical_ls):
            if len(ls)==3:
                pta, ptb = ls[1], ls[2]
            else:
                pta, ptb = ls
            self.line_segment((pta,ptb), ls_cls)
            if show_cls_orderednumber:
                self.plt.text(pta[0], pta[1], f"{idx}")

    def show_visited_sights(self, visited_sights:Sight, vision_range):
        for center  in visited_sights.closed_sights:
            csights=visited_sights.closed_sights[center]
            osights=visited_sights.open_sights[center]
            self.vision(center, vision_range, csights, osights)

    def animation(self, Robot:Robot, world_name, iter_count, obstacles:Obstacles, easy_experiment= False):
        # clear plot
        self.clear()

        # for stopping simulation with the esc key.
        self.plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
                  
        ''' draw map obstacles/world '''            
        # prepare title
        status_title = self.prepare_title(iter_count, Robot.cost)
        self.show_map(world_name=world_name, obstacles=obstacles, plot_title=status_title)
        if obstacles.enable_config_space:
            self.show_configuration_space(obstacles.config_space)

        # show_visitedSights
        if show_visitedSights:
            self.show_visited_sights(Robot.visited_sights, Robot.vision_range)
        
        if show_robot:
            self.robot(Robot,yaw=0)
        
        if show_goal:
            self.goal(Robot.goal, Robot.reach_goal, Robot.saw_goal)
        
        # plot the last robot's vision at local (center)
        if not show_visitedSights:
            closed_sights = Robot.visited_sights.get_closed_sights(Robot.coordinate)
            open_sights = Robot.visited_sights.get_open_sights(Robot.coordinate)
            self.vision(center=Robot.coordinate, radius=Robot.vision_range, csights=closed_sights, osights=open_sights)
        
        if show_local_openpt and len(Robot.local_open_pts) > 0:
            lo_points  = np.array(Robot.local_open_pts)
            self.points(lo_points, ls_lopt)
        
        # ranking points
        if show_active_openpt and len(Robot.global_active_open_rank_pts) > 0:
            self.point_colors(Robot.global_active_open_rank_pts, Robot.global_active_open_rank_pts[:,2])
        
        if show_visibilityGraph:
            self.visibility_graph(Robot.visibility_graph, ls_vg)
        
        if show_visitedPath:
            #self.paths(Robot.visited_path, ls_vp, ls_goingp)
            print ("Robot.visited_paths", Robot.visited_paths)
            self.paths_color(Robot.visited_paths, Robot.visited_path_directions)
        
        if show_sketelonPath and len(Robot.skeleton_path) > 0:
            self.path(Robot.skeleton_path, ls_sp)
        
        if show_approximately_shortest_path and len(Robot.asp)>0:
            self.path(Robot.asp, ls_asp)
        
        if show_critical_line_segments:
            self.critical_line_segments(Robot.ls, ls_cls)
        
            # display next point if existing
        if show_next_point:
            if Robot.next_point is not None:
                self.point(Robot.next_point, ls_nextpt)
        
        # to set equal make sure x y axises are same resolution 
        self.set_equal()
        self.show_grid()
        if not easy_experiment:
            self.plt.pause(0.001)

    ''' plot connection between 2 nodes'''
    def connection(self, nodeA, nodeB, ls="-b"):
        self.line_segment( (nodeA.coords, nodeB.coords), ls=ls)
    
    ''' plot edges from node to its children '''
    def tree_edges(self, node, ls=ls_tree_edge):
        for node_children in node.children:
            self.line_segment( (node.coords, node_children.coords), ls=ls, lw=0.3)
    
    ''' plot a tree's node '''
    def tree_node(self, node, ls_active=ls_tree_node_active, ls_inactive= ls_tree_node_inactive,\
            ls_visited = ls_tree_node_visited):

        if node.visited:
            self.point(node.coords, ls=ls_visited)
        elif node.active:
            self.point(node.coords, ls=ls_active)
        else:
            self.point(node.coords, ls=ls_inactive)

    ''' plot tree (all edges and vertices) from given node as tree's root '''
    def tree(self, tree, node_en=True, edge_en=True, color_mode=TreeColor.no):
        nodes_coords  = []
        nodes_lmc = []
        nodes_cost = []
        for node in tree.all_nodes():   # get all nodes
            nodes_coords.append(node.coords)
            nodes_lmc.append(node.lmc)
            nodes_cost.append(node.cost)

            # draw edge
            if edge_en:
                self.tree_edges(node)      # plot all edges between node and its children            
            if node_en and color_mode==TreeColor.no:
                self.tree_node(node)   # plot nodes

        nodes_coords = np.array(nodes_coords)
        if node_en:
            if color_mode == TreeColor.by_lmc:
                self.point_colors(nodes_coords, nodes_lmc, colormap="Dark2")
            elif color_mode == TreeColor.by_cost:
                self.point_colors(nodes_coords, nodes_cost, colormap="Dark2")

    ''' plot all trees'node (ertices) from given node as tree's root '''
    def tree_all_nodes(self, tree):
        for node in tree.all_nodes():   # get all nodes
            self.tree_node(node)   # plot nodes
            #self.text(node.coords, "{0:.2f}".format(node.rhs))

    ''' plot connection between 2 nodes'''
    def connection(self, nodeA, nodeB, ls="-b", lw=1):
        self.line_segment( (nodeA.coords, nodeB.coords), ls=ls, lw=lw)
    
    ''' plot edges from node to its children '''
    def RRT_edges(self, node, ls=ls_tree_edge):
        for node_children in node.children:
            self.line_segment( (node.coords, node_children.coords), ls, lw=0.2)

    ''' plot edges from node to its children '''
    def RRT_neighbour_edges(self, node, ls=ls_tree_edge):
        children_node = node.children
        neighbour_nodes = node.neighbours
        for n_node in neighbour_nodes:
            if n_node in children_node:
                lw = 0.7
                ls = "-b"
            else:
                lw = 0.2
                ls = ":"
            self.line_segment( (node.coords, n_node.coords), ls=ls, lw=lw)

    ''' plot a tree's node '''
    def RRT_node(self, node:Node, ls_active=ls_tree_node_active, ls_inactive= ls_tree_node_inactive,\
            ls_visited = ls_tree_node_visited):
        
        if node.visited:
            self.point(node.coords, ls=ls_visited)
        elif node.active:
            self.point(node.coords, ls=ls_active)
        else:
            self.point(node.coords, ls=ls_inactive)

    ''' plot tree (all edges and vertices) from given node as tree's root '''

    def RRTree(self, tree, node_en=True, edge_en=True, neighbour_en = False, color_mode=TreeColor.no):
        nodes_coords  = []
        nodes_lmc = []
        nodes_cost = []
        MAX_RANGE = max(tree.sampling_area[0][1],tree.sampling_area[1][1])*1.5
        for node in tree.all_nodes():   # get all nodes
            nodes_coords.append(node.coords)
            if node.lmc == float("inf"):
                nodes_lmc.append(MAX_RANGE)    
            else:
                nodes_lmc.append(node.lmc)
            if node.cost == float("inf"):
                nodes_cost.append(MAX_RANGE)   
            else:
                nodes_cost.append(node.cost)

            # draw edge
            if neighbour_en: # plot all edges between nodes and their neighbours
                self.RRT_neighbour_edges(node)
            elif edge_en:
                self.RRT_edges(node)      # plot all edges between nodes and their children            

            if node_en and color_mode==TreeColor.no:
                self.RRT_node(node)   # plot nodes

        nodes_coords = np.array(nodes_coords)
        if node_en:
            if color_mode == TreeColor.by_lmc:
                self.point_colors(nodes_coords, nodes_lmc, colormap="Dark2")
            elif color_mode == TreeColor.by_cost:
                self.point_colors(nodes_coords, nodes_cost, colormap="Dark2")
            

    ''' plot path_coords that is sequence of nodes coordinate'''
    def path_coords(self, path, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.point(nodeA, ls_node)
            self.line_segment( (nodeA, nodeB), ls_edge, lw=lw)

    ''' plot paths which contains many paths_coords'''
    def paths_coords(self, paths, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for path in paths:
            self.path_coords(path, ls_node = ls_node, ls_edge=ls_edge, lw=lw)

    ''' plot path that is sequence of nodes'''
    def RRT_path(self, path, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.RRT_node(nodeA, ls_node)
            self.connection(nodeA, nodeB, ls_edge, lw=lw)

    ''' plot paths which contains many paths'''
    def RRT_paths(self, paths, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for path in paths:
            self.RRT_path(path, ls_node = ls_node, ls_edge=ls_edge, lw=lw)

    ''' plot path that is sequence of nodes'''
    def path_cost(self, path, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.RRT_node(nodeA, ls_node)
            self.connection(nodeA=nodeA, nodeB=nodeB, ls=ls_edge, lw=lw)
            self.text(nodeA.coords, "{0:.2f}".format(nodeA.cost))   # cost
            self.text((nodeA.coords[0],nodeA.coords[1]-1), "{0:.2f}".format(nodeA.lmc))   # cost



    def paths_cost(self, paths, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge, lw=1):
        for path in paths:
            self.path_cost(path, ls_node = ls_node, ls_edge=ls_edge, lw=lw)

    ''' plot all info ( number of iteration, cost, path, tree) '''
    def build_tree_animation(self, num_iter, Tree, obstacles,  goal_coords, start_coords, rand_coordinate=None, rand_node=None,\
            neighbour_nodes=[], nearest_neighbour_node= None, color_tree=TreeColor.no):
        
        # clear plot
        self.clear()

        ''' draw map obstacles/world '''            
        # prepare title
        status_title = self.prepare_title(num_iter, Tree.total_goal_cost)

        # plot map
        if obstacles is None:
            obstacles = Obstacles() # replace with empty obstacle

        if show_map:
            self.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
        
        # draw current tree
        self.RRTree(Tree, color_mode=color_tree, neighbour_en=True)

        # draw goal
        self.goal(goal_coords, Tree.reach_goal, None)

        # draw start 
        self.start(start_coords)

        ''' animation new node '''
        if rand_coordinate is not None and rand_node is not None:
            self.point(rand_coordinate, ls=ls_rand_coordinates)
            self.line_segment((rand_coordinate, rand_node.coords), ls=ls_ls_to_nn)
            
        if rand_node is not None:
            self.vision_area(rand_node.coords, Tree.radius)
            self.point(rand_node.coords, ls=ls_random_node)
        if nearest_neighbour_node is not None:
            self.point(nearest_neighbour_node.coords, ls=ls_nearest_n_node)
        if neighbour_nodes is not None:
            for neighbour_node in neighbour_nodes:
                self.point(neighbour_node.coords, ls=ls_neighbour_node)
        
        # path from root to goal
        self.RRT_path(Tree.path_to_goal, ls_edge=ls_ahead_path, ls_node=ls_path_node, lw=lw_path)
        self.pause(0.001)
    
    def RRTX_animation(self, Tree=Tree, obstacles=Obstacles, robot=Robot, obstacle_nodes=[],\
                    discovered_obstacle_nodes = [], all_children= [], rrt_queue=Priority_queue,\
                    sql_nodes= [], easy_experiment=False):
        
        # clear plot
        self.clear()

        ''' draw map obstacles/world '''            
        # prepare title
        #status_title = self.prepare_title(num_iter, Tree.total_goal_cost)
        status_title = "range {0}, path cost {1:0.2f}".format(robot.vision_range, robot.cost)
        if robot.reach_goal:
            status_title += ", reached goal."
        # plot map
        if show_map:
            self.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
        
        # draw tree
        self.RRTree(Tree, color_mode=TreeColor.by_lmc)
        
        # draw goal
        self.goal(robot.goal, robot.reach_goal, None)

        # draw start 
        self.start(robot.start)
        self.robot(robot=robot)
        self.vision_area(robot.coordinate, robot.vision_range)
        self.point_text(robot.coordinate, "1r", "bot")

        self.RRT_path(path=robot.path_look_ahead_to_goal, ls_edge=ls_ahead_path, ls_node=ls_path_node, lw=lw_path)
        self.paths_coords(paths=robot.visited_paths,ls_edge=ls_visited_path, ls_node=ls_path_node, lw=lw_path)
        
        # debug 
        debug = False
        if debug:
            for pt in obstacle_nodes:
                self.point(pt.coords, "ok")
                #self.point_text(pt.coords, "ok", "o")
            for pt in discovered_obstacle_nodes:
                self.point(pt.coords, "og")
                #self.point_text(pt.coords, "og", '_')
            for pt in all_children:
                self.point(pt.coords, ".b")
                #self.point_text(pt.coords, "^b", 'c')

            for pt in sql_nodes:
                #self.point(pt.coords, "^b")
                self.point_text(pt.coords, ".m", '_')
            
            queue_node = rrt_queue.get_all_values()
            for pt in queue_node:
                #self.point(pt.coords, "Pr")
                self.point_text(pt.coords, "1r", 'q')
        if not easy_experiment:
            self.pause(0.001)

########################################################################################################
# display csv results 
# Plotter -r file name
########################################################################################################
import pandas as pd
import argparse
import matplotlib.pyplot as plt
import os
import glob

path = '/path/to/directory'
for file in glob.glob(os.path.join(path, '*.py')):
    print(file)
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-r', metavar="result file",  help='result file', default="result.csv")
    menu_result = parser.parse_args()
    # get user input
    result_files = menu_result.r
    path = ''
    print (result_files)
    for file in glob.glob(os.path.join(path, result_files)):
        print(file)
        result_data = pd.read_csv(file)
        result_data.plot(kind="bar")
        plt.show()