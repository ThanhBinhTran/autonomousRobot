from Plot_base_lib import Plot_base
from Program_config import *
from Tree import Node

class Plot_RRT(Plot_base):
    def __init__(self, size=(7,7), title="Robotic Plotter"):
        super().__init__(size, title)

    ''' plot connection between 2 nodes'''
    def connection(self, nodeA, nodeB, ls="-b"):
        self.line_segment( (nodeA.coords, nodeB.coords), ls=ls)
    
    ''' plot edges from node to its children '''
    def tree_edges(self, node, ls=ls_tree_edge):
        for node_children in node.children:
            self.line_segment( (node.coords, node_children.coords), ls)
    
    ''' plot a tree's node '''
    def tree_node(self, node:Node, ls_active=ls_tree_node_active, ls_inactive= ls_tree_node_inactive,\
            ls_visited = ls_tree_node_visited):

        if node.visited:
            self.point(node.coords, ls=ls_visited)
        elif node.active:
            self.point(node.coords, ls=ls_active)
        else:
            self.point(node.coords, ls=ls_inactive)

    ''' plot tree (all edges and vertices) from given node as tree's root '''
    def tree(self, tree):
        for node in tree.all_nodes():   # get all nodes
            self.tree_edges(node)      # plot all edges between node and its children
            self.tree_node(node)   # plot nodes

    ''' plot path that is sequence of nodes'''
    def path(self, path, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.tree_node(nodeA, ls_node)
            self.connection(nodeA, nodeB, ls_edge)

    ''' plot paths which contains many paths'''
    def paths(self, paths, ls_node = ls_goal_path_node, ls_edge=ls_goal_path_edge):
        for path in paths:
            self.path(path, ls_node = ls_node, ls_edge=ls_edge)

    ''' plot all info ( number of iteration, cost, path, tree) '''
    def animation(self, num_iter, cost, path, Tree, obstacles,  start_coords, goal_coords):
        # prepare title
        reach_goal = (cost > 0 and cost != float("inf") )
        status_title = self.prepare_title(num_iter,cost)

        # clear old one
        self.clear()

        # plot new one
        self.show_map(world_name=None, obstacles=obstacles, plot_title=status_title)
        self.tree(Tree)
        self.goal(goal_coords, reach_goal, None)
        self.start(start_coords)
        self.path(path)
        self.pause(0.1)
