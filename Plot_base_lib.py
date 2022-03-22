import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import math

from Robot_map_lib import Map
from Robot_world_lib import World
from Robot_base import RobotType
from Program_config import *


class Plot_base:
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
    show_grid = lambda self: self.plt.grid(True)
    
    ''' plot point(s)'''
    point  = lambda self, point,  ls=".r": self.plt.plot(point[0], point[1], ls)
    points = lambda self, points, ls=".r": self.plt.plot(points[:, 0], points[:, 1], ls)

    ''' plot text at near point coordinate '''
    text = lambda self, point, str: self.plt.text(point[0], point[1] + 2, str)

    ''' plot text and point'''
    def point_text(self, point, ls, text):
        self.point(point, ls)
        self.text(point, text)

    ''' plot line segment connecting 2 points'''
    line_segment = lambda self, line, ls="-.k": self.plt.plot((line[0][0], line[1][0]), (line[0][1], line[1][1]), ls)
    
    ''' plot path containing list of points '''
    def path(self, points, ls="-xr"):
        new_point = np.array(points)
        self.points(points=new_point, ls=ls)

    def polygon(self, polygon: list, ls= "-r"):
        new_points = polygon.copy()
        new_points.append(new_points[0])
        self.path(points= new_points, ls=ls)
    
    def polygons(self, polygons: list, ls= "-r"):   # polygons is list of polygon
        for polygon in polygons:
            self.polygon(polygon=polygon, ls = ls)

    ''' plot triangle (out fill) '''
    def plot_triangle(self, triangle, ls=":c"):
        self.line_segment((triangle[0], triangle[1]), ls)
        self.line_segment((triangle[1], triangle[2]), ls)
        self.line_segment((triangle[2], triangle[0]), ls)

    ''' plot triangle ( within fill) '''
    def plot_triangle_fill(self, triangle, cl="g", alpha=transparent, linestyle=":"):
        new_triangle = np.array(triangle)
        self.plt.fill(new_triangle[:, 0], new_triangle[:, 1], color=cl, alpha=alpha, linestyle=linestyle)
    
    ''' Goal'''
    def goal(self, goal, r_goal=False, s_goal=False):
        self.point(goal, ls_goal)
        if show_text_goal:
            if r_goal: 
                self.text(goal, "reached goal!")
            elif s_goal:
                self.text(goal, "saw goal!")
            else:
                self.text(goal, "goal")
    ''' Start '''
    def start(self, start):
        self.point_text(start, ls_start, "start!")

    ''' Obstacles'''
    def show_map(self, world_name=None, obstacles=None, plot_title=None):
        # draw world and map
        if show_world and world_name is not None:
            World().display(self.plt, mpimg, world_name)
            
        # draw map obstacles 
        if show_map:
            Map().display(self.plt, plot_title, obstacles.obstacles)
    
    ''' set plot's title'''
    title = lambda self, x: self.plt.title(x)
    
    ''' prepare title for display'''
    def prepare_title(self, iter_count, path_cost):
        plot_title = "Number of iteration: {0}".format(iter_count+1)
        if path_cost > 0 and path_cost != float('inf'):
            plot_title += ", path len: {:.2f}".format (path_cost)
        else:
            plot_title += ", not reached goal yet."
        return plot_title


    ''' vision libs'''

    ''' vision circle'''
    def vision_area(self, center, vision_radius, ls=":"):
        """ draw a circle that limits the vision of robot """
        vision = plt.Circle(center, vision_radius, color="red", linestyle=ls, fill=False)
        plt.gcf().gca().add_artist(vision)

    ''' Robot '''
    def robot(self, robot, yaw=0):  # pragma: no cover
        x,y = robot.coordinate
        if robot.robot_type == RobotType.rectangle:
            outline = np.array([[-robot.length / 2, robot.length / 2,
                                (robot.length / 2), -robot.length / 2,
                                -robot.length / 2],
                                [robot.width / 2, robot.width / 2,
                                - robot.width / 2, -robot.width / 2,
                                robot.width / 2]])
            Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                            [-math.sin(yaw), math.cos(yaw)]])
            outline = (outline.T.dot(Rot1)).T
            outline[0, :] += x
            outline[1, :] += y
            plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), "-k")
        elif robot.robot_type == RobotType.circle:
            circle = plt.Circle((x, y), robot.radius, color="b")
            plt.gcf().gca().add_artist(circle)
            out_x, out_y = (np.array([x, y]) +
                            np.array([np.cos(yaw), np.sin(yaw)]) * robot.radius)
            plt.plot([x, out_x], [y, out_y], "-k")