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
    """ draw a circle that limits the vision of robot """ 
    vision = plt.Circle((x, y), radius, color="red", linestyle  = ":", fill=False)
    plt.gcf().gca().add_artist(vision)
    
def draw_arc_area(plt, x, y, radius):

    # Draw arc with arrow.
    aarc = patches.Arc((x, y),
              radius*2, radius*2,  # ellipse width and height
              theta1=0, theta2=1, linestyle="-", color = "b")
    #plt.gcf().gca().add_patch(arc)
    plt.axes().add_patch(aarc)
       
def plot_vision(plt, x, y, radius, true_sight, osight, csight):
    draw_vision_area(plt, x, y, radius)
    #draw_arc_area(plt, x, y, radius)
    
    if show_true_sight:
        draw_true_sight(plt, x, y, true_sight, cl_ts, ls_ts) 

    if show_open_sight:
        draw_open_sight(plt, x, y, osight)
        
    if show_close_sight:
        plot_pairs(plt, csight, ls_cs)
        
def plot_point(plt, point, ls="xr"):
    plt.plot(point[0], point[1], ls)
    
def plot_line(plt, line, ls="-xr"):
    plt.plot((line[0][0],line[1][0]), (line[0][1],line[1][1]), ls)
    
def plot_lines(plt, lines, ls="-xr"):
    xs = [i[0] for i in lines]
    ys = [i[1] for i in lines]
    plt.plot(xs, ys, ls)

def plot_pairs(plt, pairs, ls="-xr"):
    for pair in pairs:
        plot_line(plt, pair, ls)
    
def plot_points(plt, ls="xr"):
    plt.plot(points[0,:], points[:,1], ls)