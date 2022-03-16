from Robot_lib import *
from Program_config import *
from matplotlib import patches
import matplotlib.image as mpimg
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt

from Robot_map_lib import Map
from Robot_world_lib import World
from Robot_parameters import Robot_parameters, RobotType
from Plotter_base_lib import Plotter_base

robot_parameters = Robot_parameters()


class Plotter(Plotter_base):
    def __init__(self, size=(6,6), title="Path Planning Problem for an Autonomous Robot"):
        super().__init__(size, title)

    def sight(self, center, pair, cl="g", alpha=transparent, linestyle=":"):
        triangle = [center, pair[0], pair[1]]
        self.plot_triangle_fill(triangle, cl, alpha, linestyle)


    def closed_sights(self, center, true_sight, cl="g", ls_ts="-"):
        for pair in true_sight:
            self.sight(center, pair, cl, transparent, ls_ts)

    def open_sights_arc(self, center, open_sight, radius, cl="g", ls_ts="-"):
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
        collection = PatchCollection(arc_patches, facecolor='g', linestyle='solid', edgecolor='r', alpha=transparent)
        self.ax.add_collection(collection)

    def vision_area(self, center, radius, ls=":"):
        """ draw a circle that limits the vision of robot """
        vision = plt.Circle(center, radius, color="red", linestyle=ls, fill=False)
        plt.gcf().gca().add_artist(vision)

    def vision(self, center, radius, csight, osight):
        if show_circleRange:
            self.vision_area(center, radius)

        if show_closedSight:
            self.closed_sights(center, csight, cl_ts, ls_ts)

        if show_openSight:
            self.open_sights_arc(center, osight, radius)

    def goal(self, goal, r_goal=False, s_goal=False):
        self.point(goal, ls_goal)
        if show_text_goal:
            if r_goal: 
                self.text(goal, "reached goal!")
            elif s_goal:
                self.text(goal, "saw goal!")
            else:
                self.text(goal, "goal")

    def start(self, start):
        self.point_text(start, ls_start, "start!")

    def paths(self, paths, ls="-r", ls_next="-b"):
        for i in range(len(paths)):
            path = paths[i]
            if i == len(paths) - 1:
                self.line_segments(path, ls_next)
            else:
                self.line_segments(path, ls)

    def visibility_graph(self, visibility_graph, ls_vg):
        for pnode in visibility_graph:
            for verteces in visibility_graph[pnode]:
                self.line_segment([pnode, verteces], ls_vg)

    def critical_line_segments(self, critical_ls, ls_cls):
        i = 0
        for ls in critical_ls:
            self.line_segment(ls[1:3], ls_cls)
            if show_cls_orderednumber:
                self.text(ls[1], i)
                i += 1

    def robot(self, center, yaw, config):  # pragma: no cover
        x,y = center
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

    def show_map(self, world_name, map_name, iter_count, obstacles, Robot):
        # draw world and map
        if show_world and world_name is not None:
            World().display(self.plt, mpimg, world_name)
            
        # draw map obstacles 
        if show_map:
            self.perpare_title(world_name, map_name, iter_count, Robot)
            Map().display(self.plt, self.plot_title, obstacles.data())

    def show_traversal_sights(self, traversal_sights, vision_range):
        for local in traversal_sights:
            local_center = local[0]  # center of robot at local
            local_closed_sights = local[1]  # closed sight at local
            local_open_sights = local[2]  # open sight at local
            self.vision(local_center, vision_range, local_closed_sights, local_open_sights)

    def perpare_title(self, world_name, map_name, iter_count, Robot):
        self.plot_title = ""
        if world_name is not None:
            self.plot_title = world_name + ".csv"  
        else:
            self.plot_title = map_name 
        self.plot_title += ", number of iteration: {0}".format(iter_count)
        self.plot_title += ", path len: {:.2f}".format (Robot.calculate_traveled_path_cost())

    def show_animation(self, Robot, world_name, map_name, iter_count, obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point):
        if show_animation:
            # clear plot
            self.clear()
            
            # for stopping simulation with the esc key.
            self.plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
                      
            # draw map obstacles/world 
            self.show_map(world_name, map_name, iter_count, obstacles, Robot)
            
            # show_traversalSights
            if show_traversalSights:
                self.show_traversal_sights(Robot.traversal_sights, Robot.vision_range)
            
            if show_robot:
                self.robot(Robot.coordinate, 0, robot_parameters)
            
            if show_goal:
                self.goal(goal, Robot.reach_goal, Robot.saw_goal)
            
            # plot robot's vision at local (center)
            self.vision(Robot.coordinate, Robot.vision_range, closed_sights, open_sights)
            
            if show_local_openpt and len(Robot.local_open_pts) > 0:
                self.points(Robot.local_open_pts, ls_lopt)
            
            if show_active_openpt and len(Robot.global_active_open_pts) > 0:
                self.points(Robot.global_active_open_pts, ls_aopt)
            
            if show_visibilityGraph:
                self.visibility_graph(Robot.visibility_graph, ls_vg)
            
            if show_visitedPath:
                self.paths(Robot.visited_path, ls_vp, ls_goingp)
            
            if show_sketelonPath:
                self.line_segments(skeleton_path, ls_sp)
            
            if show_approximately_shortest_path:
                self.line_segments(asp, ls_asp)
            
            if show_critical_line_segments:
                self.critical_line_segments(critical_ls, ls_cls)
            
                # display next point if existing
            if show_next_point:
                if len(next_point) > 0:
                    self.point(next_point, ls_nextpt)
            
            # to set equal make sure x y axises are same resolution 
            self.set_equal()
            self.show_grid()
            self.plt.pause(0.00001)
        