from collections import defaultdict
import heapq
from Robot_math_lib import point_dist
from queue import PriorityQueue


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
        # print ("__pnode: {0}, ___leafs{1}".format (pnode, leafs))
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
    def BFS_skeleton_path(self, start, goal):
        visited = set()
        queue = PriorityQueue()
        queue.put((0, start, [start]))

        while not queue.empty():
            cost, current, path = queue.get()
            if current == goal:
                return path
            if current not in visited:
                visited.add(current)
                for neighbor in self.graph[current]:
                    weight = point_dist(current, neighbor)
                    if neighbor not in visited:
                        new_path = path + [neighbor]
                        queue.put((cost + weight, neighbor, new_path))
        return []
