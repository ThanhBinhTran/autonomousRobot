import matplotlib.pyplot as plt

class plotter:
    def __init__(self, size, title):
        self.plt   = plt
        self.fig, self.ax = plt.subplots(figsize=size)
        self.fig.canvas.set_window_title(title)
        self.plt.xlim(0, 100)
        self.plt.ylim(0, 100)
    show = lambda self: self.plt.show()
    pause = lambda self, x: self.plt.pause(x)
    clear = lambda self: self.plt.cla()
    set_equal = lambda self: self.plt.axis("equal")
    title = lambda self, title: self.plt.title(title)

    ''' plot line connecting 2 points'''
    def line_points(self, ptA, ptB, ls = "-b"):
        self.plt.plot((ptA[0], ptB[0]), (ptA[1], ptB[1]), ls)
    
    ''' plot connection between 2 nodes'''
    def connection(self, nodeA, nodeB, ls="-b"):
        self.line_points(nodeA.coords, nodeB.coords, ls=ls)
    
    ''' plot edges from node to its children '''
    def tree_edges(self, node, ls="-k"):
        for node_children in node.children:
            self.line_points( node.coords, node_children.coords, ls)
    
    ''' plot tree's node '''
    def tree_node(self, node, radius, color="b"):
        circle = self.plt.Circle(node.coords, radius, color=color)
        self.plt.gcf().gca().add_artist(circle)

    ''' plot tree from given node as tree's root '''
    def tree(self, tree):
        for node in tree.all_nodes():   # get all nodes
            self.tree_edges( node)      # plot all edges between node and its children
            self.tree_node(node, 0.3, "r")   # plot nodes

    ''' plot path that is sequence of nodes'''
    def path(self, path, ls_node = "r", ls_edge="-g"):
        for i in range(len(path)-1):
            nodeA = path[i]
            nodeB = path[i+1]
            self.tree_node(nodeA, 0.5, ls_node)
            self.connection(nodeA, nodeB, ls_edge)
    
    ''' plot all info ( number of iteration, cost, path, tree) '''
    def display(self, num_iter, cost, path, Tree):
        # prepare title
        title = "number of iteration: {0}".format(num_iter+1)
        if len(path) > 0:
            title += ", cost: {:.2f}".format(cost)
        else:
            title += "; not saw goal yet."

        # clear old one
        self.clear()

        # plot new one
        self.title(title)
        self.tree(Tree)
        self.path(path)
        self.pause(0.0000001)
