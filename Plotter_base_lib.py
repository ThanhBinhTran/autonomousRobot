from Program_config import *

import matplotlib.pyplot as plt
import numpy as np

class Plotter_base:
    def __init__(self, size, title):
        self.plt   = plt
        self.fig, self.ax = plt.subplots(figsize=size)
        self.fig.canvas.set_window_title(title)
        self.plt.xlim(0, 100)
        self.plt.ylim(0, 100)
        self.plot_title = ""
    show = lambda self: self.plt.show()
    pause = lambda self, x: self.plt.pause(x)
    clear = lambda self: self.plt.cla()
    set_equal = lambda self: self.plt.axis("equal")
    show_grid = lambda self: self.plt.grid(True)

    ''' plot point(s)'''
    point  = lambda self, point,  ls="xr": self.plt.plot(point[0], point[1], ls)
    points = lambda self, points, ls="xr": self.plt.plot(points[:, 0], points[:, 1], ls)

    ''' plot text at near point coordinate '''
    text = lambda self, point, str: self.plt.text(point[0], point[1] + 2, str)

    def point_text(self, point, ls, text):
        self.point(point, ls)
        self.text(point, text)

    ''' plot line segment '''
    line_segment = lambda self, line, ls="-xr": self.plt.plot((line[0][0], line[1][0]), (line[0][1], line[1][1]), ls)
    
    ''' plot line segment(s) '''
    def line_segments(self, lines, ls="-xr"):
        xs = [i[0] for i in lines]
        ys = [i[1] for i in lines]
        self.plt.plot(xs, ys, ls)


    ''' plot triangle (out fill) '''
    def plot_triangle(self, triangle, ls=":c"):
        self.line_segment((triangle[0], triangle[1]), ls)
        self.line_segment((triangle[1], triangle[2]), ls)
        self.line_segment((triangle[2], triangle[0]), ls)

    ''' plot triangle ( within fill) '''
    def plot_triangle_fill(self, triangle, cl="g", alpha=transparent, linestyle=":"):
        new_triangle = np.array(triangle)
        self.plt.fill(new_triangle[:, 0], new_triangle[:, 1], color=cl, alpha=alpha, linestyle=linestyle)
