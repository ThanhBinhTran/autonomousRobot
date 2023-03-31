from collections import defaultdict
from queue import PriorityQueue
from Robot_math_lib import point_dist
class Graph:
    def __init__(self):
        self.graph = defaultdict(list)

    # Function to add local active point to graph
    def add_local_open_points(self, center, lActive_OpenPts):
        if len(lActive_OpenPts) > 0:
            self.graph_insert(center, lActive_OpenPts)
    
    def graph_create_edge(self, nodeA, nodeB):
        self.graph[tuple(nodeA)].append(tuple(nodeB))
        self.graph[tuple(nodeB)].append(tuple(nodeA))

    # Function to insert edges into graph
    def graph_insert(self, pnode, leafs):
        #print ("__pnode: {0}, ___leafs{1}".format (pnode, leafs))
        if len(leafs) > 0:
            for leaf in leafs:
                self.graph_create_edge(pnode, leaf)

    def get_all_non_leaf(self):
        non_leaves = []
        for pnode in self.graph:
            if len(self.graph[pnode]) > 1:
                non_leaves.append(pnode)
        return non_leaves
    
    def get_neighbor_nodes(self, node):
        return self.graph[tuple(node)]
    
    # path between two nodes of a graph
    def BFS_skeleton_path_old(self, start, goal):

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
                neighbours = self.graph[node]
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

    def BFS_skeleton_path(self, start, goal):
        visited = set()
        distances = {node: float('inf') for node in self.graph}
        distances[start] = 0
        previous_nodes = {node: None for node in self.graph}
        queue = PriorityQueue()
        queue.put((0, start))
        while not queue.empty():
            current_distance, current_node = queue.get()
            if current_node == goal:
                break
            if current_distance > distances[current_node]:
                continue
            for neighbor, weight in self.graph[current_node]:
                weight = point_dist(current_node, neighbor)
                weight_ng = point_dist(neighbor, goal)
                weight_cg = point_dist(current_node, goal)
                distance = current_distance + weight + weight_ng - weight_cg
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    queue.put((distance, neighbor))
        path = []
        current_node = goal
        while current_node is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]
        path.reverse()
        return path