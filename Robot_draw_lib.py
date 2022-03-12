from Robot_lib import *
from Program_config import *
from matplotlib import patches
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

# import matplotlib.pyplot as plt

def plot_sight(plt, x, y, pair, cl="g", alpha=transparent_alpha, linestyle=":"):
    ptA, ptB = pair
    plt.fill([x, ptA[0], ptB[0]], [y, ptA[1], ptB[1]], color=cl, alpha=alpha, linestyle=linestyle)


def draw_true_sight(plt, x, y, true_sight, cl="g", ls_ts="-"):
    for pair in true_sight:
        plot_sight(plt, x, y, pair, cl, transparent_alpha, ls_ts)


def draw_open_sight_triangle(plt, x, y, open_sight, cl="g", ls_ts="-"):
    for data in open_sight:
        pair = data[0], data[1]
        plot_sight(plt, x, y, pair, cl, transparent_alpha, ls_ts)

def draw_open_sight_arc(ax, x, y, open_sight, radius, cl="g", ls_ts="-"):
    center = x, y
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
    collection = PatchCollection(arc_patches, facecolor='g', linestyle='solid', edgecolor='r', alpha=transparent_alpha)
    ax.add_collection(collection)

def draw_vision_area(plt, x, y, radius, ls=":"):
    """ draw a circle that limits the vision of robot """
    vision = plt.Circle((x, y), radius, color="red", linestyle=ls, fill=False)
    plt.gcf().gca().add_artist(vision)


def draw_arc_area(plt, x, y, radius):
    # Draw arc with arrow.
    aarc = patches.Arc((x, y),
                       radius * 2, radius * 2,  # ellipse width and height
                       theta1=0, theta2=1, linestyle="-", color="b")
    # plt.gcf().gca().add_patch(arc)
    plt.axes().add_patch(aarc)


def plot_vision(plt, ax, x, y, radius, csight, osight):
    if show_circleRange:
        draw_vision_area(plt, x, y, radius)


    if show_closedSight:
        draw_true_sight(plt, x, y, csight, cl_ts, ls_ts)

    if show_openSight:
        #draw_open_sight_triangle(plt, x, y, osight)
        draw_open_sight_arc(ax, x, y, osight, radius)


def plot_goal(plt, goal, r_goal, s_goal):
    plot_point(plt, goal, ls_goal)
    if show_text_goal:
        if r_goal:
            plt.text(goal[0], goal[1] + 2, "reached goal!")
        elif s_goal:
            plt.text(goal[0], goal[1] + 2, "saw goal!")
        else:
            plt.text(goal[0], goal[1] + 2, "goal")


def plot_start(plt, start):
    plot_point(plt, start, ls_start)
    plt.text(start[0], start[1] + 2, "start!")


def plot_point(plt, point, ls="xr"):
    plt.plot(point[0], point[1], ls)


def plot_points(plt, pts, ls="xr"):
    plt.plot(pts[:, 0], pts[:, 1], ls)


def plot_text(plt, point, text):
    plt.text(point[0], point[1] + 2, text)


def plot_point_text(plt, point, ls, text):
    plot_point(plt, point, ls)
    plt.text(point[0], point[1] + 2, text)


def plot_line(plt, line, ls="-xr"):
    plt.plot((line[0][0], line[1][0]), (line[0][1], line[1][1]), ls)


def plot_lines(plt, lines, ls="-xr"):
    xs = [i[0] for i in lines]
    ys = [i[1] for i in lines]
    plt.plot(xs, ys, ls)


def plot_paths(plt, paths, ls="-r", ls_next="-b"):
    for i in range(len(paths)):
        path = paths[i]
        if i == len(paths) - 1:
            plot_lines(plt, path, ls_next)
        else:
            plot_lines(plt, path, ls)


def plot_pairs(plt, pairs, ls="-xr"):
    for pair in pairs:
        plot_line(plt, pair, ls)


def plot_edge(plt, center_pts, edges):
    for pairs in edges:
        plot_line(plt, [center_pts[pairs[0]], center_pts[pairs[1]]], ls="-k")


def plot_visibilityGraph(plt, visibility_graph, ls_vg):
    for pnode in visibility_graph:
        for verteces in visibility_graph[pnode]:
            plot_line(plt, [pnode, verteces], ls_vg)


def plot_triangles(plt, triangles, ls=":c"):
    """ 
    plot list of triangles
    """
    for triangle in triangles:
        plot_line(plt, [triangle[0], triangle[1]], ls)
        plot_line(plt, [triangle[1], triangle[2]], ls)
        plot_line(plt, [triangle[2], triangle[0]], ls)


def plot_critical_line_segments(plt, critical_ls, ls_cls):
    i = 0
    for ls in critical_ls:
        plot_line(plt, ls[1:3], ls_cls)
        if show_cls_orderednumber:
            plot_text(plt, ls[1], i)
            i += 1


def plot_center(plt, center_pts):
    """
    plot a center of triangles
    """
    for i in range(len(center_pts)):
        plot_point_text(plt, center_pts[i], ".b", "{0}".format(i))


def plot_robot(plt, x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")
