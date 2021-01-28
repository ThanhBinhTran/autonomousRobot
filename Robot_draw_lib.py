from Robot_lib import *
from Program_config import *
from matplotlib import patches

rel_tol = 0.0000001
#import matplotlib.pyplot as plt

def plot_sight(plt, x, y, pair, cl = "g", alpha = 0.3, linestyle = ":"):
    ptA, ptB = pair
    plt.fill([x, ptA[0], ptB[0]], [y, ptA[1], ptB[1]], color = cl, alpha = alpha, linestyle = linestyle)
    
def draw_true_sight(plt, x, y, true_sight, cl="g", ls_ts="-"):
    for pair in true_sight:
        plot_sight(plt, x, y, pair, cl, 0.3, ls_ts)

def draw_open_sight(plt, x, y, open_sight, cl="g", ls_ts="-"):
    for data in open_sight:
        pair = data[0], data[1]
        plot_sight(plt, x, y, pair, cl, 0.3, ls_ts)
            
def draw_vision_area(plt, x, y, radius):
    ''' draw a circle that limits the vision of robot ''' 
    vision = plt.Circle((x, y), radius, color="red", linestyle  = ":", fill=False)
    plt.gcf().gca().add_artist(vision)

def draw_vision_area(plt, x, y, radius, ls = ":"):
    ''' draw a circle that limits the vision of robot ''' 
    vision = plt.Circle((x, y), radius, color="red", linestyle  = ls, fill=False)
    plt.gcf().gca().add_artist(vision)
    
def draw_arc_area(plt, x, y, radius):

    # Draw arc with arrow.
    aarc = patches.Arc((x, y),
              radius*2, radius*2,  # ellipse width and height
              theta1=0, theta2=1, linestyle="-", color = "b")
    #plt.gcf().gca().add_patch(arc)
    plt.axes().add_patch(aarc)
       
def plot_vision(plt, x, y, radius, true_sight, osight, csight):
    if show_circle_range:
        draw_vision_area(plt, x, y, radius)
    #draw_arc_area(plt, x, y, radius)
    
    if show_true_sight:
        draw_true_sight(plt, x, y, true_sight, cl_ts, ls_ts) 
            
    if show_open_sight:
        draw_open_sight(plt, x, y, osight)
        
    if show_close_sight:
        plot_pairs(plt, csight, ls_cs)
        
def plot_explored_map(plt, explored_map, ls_em):
        plot_pairs(plt, explored_map, ls_em)

def plot_goal(plt, goal, r_goal, s_goal):
    plt.plot(goal[0], goal[1], ls_goal)
    if show_text_goal:
        if r_goal:
            plt.text(goal[0], goal[1] + 2, "reached goal!")
        elif s_goal:
            plt.text(goal[0], goal[1] + 2, "saw goal!")
        else:
            plt.text(goal[0], goal[1] + 2, "goal")
            
def plot_point(plt, point, ls="xr"):
    plt.plot(point[0], point[1], ls)

def plot_point_text(plt, point, ls, text):
    plt.plot(point[0], point[1], ls)
    plt.text(point[0], point[1] + 2, text)
   
def plot_line(plt, line, ls="-xr"):
    plt.plot((line[0][0],line[1][0]), (line[0][1],line[1][1]), ls)
    
def plot_lines(plt, lines, ls="-xr"):
    xs = [i[0] for i in lines]
    ys = [i[1] for i in lines]
    plt.plot(xs, ys, ls)

def plot_pairs(plt, pairs, ls="-xr"):
    for pair in pairs:
        plot_line(plt, pair, ls)
    
def plot_points(plt, pts, ls="xr"):
    plt.plot(pts[:, 0], pts[:, 1], ls)
    
def plot_edge(plt, center_pts, edges):
    for pairs in edges :
        plot_line(plt, [center_pts[pairs[0]],center_pts[pairs[1]]], ls="-k")
 
def plot_triangles(plt,triangles, ls = ":c"):
    ''' 
    plot list of triangles
    '''
    for triangle in triangles:
        plot_line(plt, [triangle[0],triangle[1]], ls)
        plot_line(plt, [triangle[1],triangle[2]], ls)
        plot_line(plt, [triangle[2],triangle[0]], ls)

def plot_center(plt, center_pts): 
    '''
    plot a center of triangles
    '''
    for i in range(len(center_pts)):
        plt.plot(center_pts[i][0], center_pts[i][1], ".b" )
        plt.text(center_pts[i][0], center_pts[i][1], "{0}".format(i) )  
