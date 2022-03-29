from Robot_lib import point_dist
import numpy as np

''' Node class '''
class Node:
    def __init__(self, coords=None):
        self.coords   = tuple(coords)
        self.children = []
        self.parent  = None
        self.active = True  # if inactive, it is eliminated from tree. 
        self.cost = 0.0

    ''' set cost '''
    def set_cost(self, x):
        self.cost = x
    
    ''' set inactive node, delete its parent and children as well '''
    def set_inactive(self):
        self.active = False
        # remove parents connections
        if self.parent is not None:
            self.parent.remove_child(self)
        self.parent = None
        
        # remove all childrens connections
        for child_node in self.children:
            child_node.parent = None
        
        self.children.clear()


    ''' update self cost and all its children costs (for rewiring) '''
    def update_costs(self):
        self.cost = self.parent.cost + point_dist(self.parent.coords, self.coords)
        [children_node.update_costs() for children_node in self.children]

    def all_children(self):
        allChildren = []
        for child in self.children:
            allChildren.append(child.coords)
        return allChildren
    
    def bring_closer(self, random_node_coords, nearest_dist, step_size):
        min_dist = min(nearest_dist, step_size); # If the node is closer than epsilon, we have the same distance
        closer_coordinate = self.coords + (random_node_coords - self.coords)*min_dist/nearest_dist; 
        return closer_coordinate

    add_child   = lambda self, x: self.children.append(x)
    remove_child   = lambda self, x: self.children.remove(x)

    def add_parents(self, parents):
        self.parent = parents
    def remove_parents(self):
        self.parent = None

''' Tree class '''
class Tree:
    """ Tree class for generating final path """
    def __init__(self, root: Node):
        self.root   = root
        self.dict   = {root.coords: self.root}

    def add_edge(self, parent_node, new_node):
        # calculate and set cost for new nodes.
        cost = parent_node.cost + point_dist(parent_node.coords, new_node.coords)
        new_node.set_cost(cost)

        # link to parent and declare node in dicitonary
        new_node.parent = parent_node
        parent_node.add_child(new_node)
        self.dict[new_node.coords]  = new_node
    
    def remove_edge(self, parent_node, node):
        self.dict[parent_node.coords].remove_child(node)

    __getitem__  = lambda self, x: self.dict[x]
    __contains__ = lambda self, x: x in self.dict

    def all_nodes(self):
        ''' return a list of all tree's nodes'''
        allNodes = list(self.dict.values())
        #print ("allNodes: ", allNodes)
        return allNodes

    def all_nodes_coordinate(self):
        ''' return a list of all tree's nodes coordinate'''
        all_Nodes_Coordinate = list(self.dict.keys())
        #print ("all_Nodes_Coordinate: ", all_Nodes_Coordinate)
        return all_Nodes_Coordinate
    
    ''' calculate cost from node to root'''
    def cost(self, node):
        return node.cost

    ''' calculate costs from nodes to tree's root '''
    def costs(self, nodes):
        return [node.cost for node in nodes]

    ''' calculate all costs from all tree's nodes to root'''
    def all_nodes_cost(self):
        return [self.cost(node) for node in self.all_nodes()]
    
    ''' calcualte distances among node and given tree's nodes '''
    def distances(self, node_coords, tree_nodes):
        return [point_dist(node_coords, node.coords) for node in tree_nodes]

    ''' calculate all distances among node and all tree nodes '''
    def all_distances(self, node_coords):
        return [point_dist(node_coords, n_coords) for n_coords in self.all_nodes_coordinate()]
    
    ''' find the nearest node to given node, return its distance and index '''
    def nearest(self, node_coords):
        all_dist = self.all_distances(node_coords)
        nearest_idx = np.argmin(all_dist)
        return all_dist[nearest_idx], nearest_idx
    
    '''  find nearest neighbour node from list of nodes'''
    def nearest_in_list_neighbour(self, node_coordinate, neighbour_nodes):
        # check if there is neighbour nearby
        if neighbour_nodes is None:
            return None
            
        # calculate all cost from random's neighbours tree's node to tree's root
        n_costs = np.array(self.costs(neighbour_nodes))
        # get distances from random node to all its neighbours
        n_dist = np.array(self.distances(node_coordinate, neighbour_nodes))
        # pick the closest neighbour
        n_idx = np.argmin(n_costs +  n_dist)
        nearest_neighbour_node = neighbour_nodes[n_idx]

        return nearest_neighbour_node

    '''  find nearest neighbour node to node_coordinate in circle of radius '''
    def nearest_neighbour(self, node_coordinate, radius):
        # find all neighbours of node_coordinate
        neighbour_nodes = self.neighbour_nodes(node_coordinate, radius)
        # pick nearest neighbour as new_node's parent
        return self.nearest_in_list_neighbour(node_coordinate, neighbour_nodes)

    def neighbour_nodes(self, node_coordinate, radius):
        # get array of all RRTree's node
        all_nodes = np.array(self.all_nodes())

        # calcualte all distances from random to tree nodes
        all_distances = np.array(self.all_distances(node_coordinate))

        # get all random's neighbours (represent n_) nodes which are inside radius
        n_node_indices = all_distances < radius

        if np.sum(n_node_indices) == 0: # no-one lives nearby :)
            return None
        # else return neighbour(s)
        return all_nodes[n_node_indices]   
        
    def printTree(self, node, depth=0):
        if node is None: return
        print(" | "*depth, node)

        for child in node.children:
            self.printTree(child, depth+1)
    
    # return the root of subtree from node
    def find_root(self, node: Node):
        while(node.parent is not None):
            node = node.parent
        return node

    def rewire(self, new_node, neighbour_nodes):
        if new_node is None:
            return None
        new_node_cost = self.cost(new_node)
        old_costs = self.costs(neighbour_nodes)
        neighbour_costs = self.distances(new_node.coords, neighbour_nodes)
        new_cost = np.array([new_node_cost]* len(neighbour_nodes)) + neighbour_costs

        #print ("______________newNode_to_root ",new_node_cost)
        #print ("______________old_costs ",old_costs)
        #print ("______________neighbour_costs ",neighbour_costs)
        #print ("______________new_cost ",new_cost)
        is_rewire = new_cost < old_costs
        for node in neighbour_nodes[is_rewire]:
            self.remove_edge(node.parent, node) # remove old parent, ALWAYS COME FIRST
            self.add_edge(new_node, node)   # connect to new parent
            node.update_costs()
            
        for node in neighbour_nodes[is_rewire]:
            self.remove_edge(node.parent, node) # remove old parent, ALWAYS COME FIRST
            self.add_edge(new_node, node)   # connect to new parent
