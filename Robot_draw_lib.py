from Obstacles import Obstacles
from Robot import Robot
from Robot_lib import *
from Program_config import *
from matplotlib import patches
import matplotlib.image as mpimg
from matplotlib.collections import PatchCollection

from Plot_base_lib import Plot_base
 
class Plot_robot(Plot_base):
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
        vision = self.plt.Circle(center, radius, color="red", linestyle=ls, fill=False)
        self.plt.gcf().gca().add_artist(vision)

    def vision(self, center, radius, csight, osight):
        if show_circleRange:
            self.vision_area(center, radius)

        if show_closedSight:
            self.closed_sights(center, csight, cl_ts, ls_ts)

        if show_openSight:
            self.open_sights_arc(center, osight, radius)

    def paths(self, paths, ls="-r", ls_next="-b"):
        for i in range(len(paths)):
            path = paths[i]
            if i == len(paths) - 1:
                self.path(path, ls_next)
            else:
                self.path(path, ls)

    def show_configuration_space(self, config_space: list):
        self.polygons(config_space, ls = ls_cspace)
        
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

    def show_traversal_sights(self, traversal_sights, vision_range):
        for local in traversal_sights:
            local_center = local[0]  # center of robot at local
            local_closed_sights = local[1]  # closed sight at local
            local_open_sights = local[2]  # open sight at local
            self.vision(local_center, vision_range, local_closed_sights, local_open_sights)

    def show_animation(self, Robot: Robot, world_name, map_name, iter_count, obstacles:Obstacles , goal, 
                    closed_sights, open_sights, skeleton_path, asp , critical_ls, next_point):
        if show_animation:
            # clear plot
            self.clear()
            
            # for stopping simulation with the esc key.
            self.plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
                      
            '''draw map obstacles/world '''            
            # prepare title
            cost = Robot.calculate_traveled_path_cost()
            status_title = self.prepare_title(iter_count, cost)
            self.show_map(world_name=world_name, obstacles=obstacles, plot_title=status_title)
            if obstacles.enable_config_space:
                self.show_configuration_space(obstacles.config_space)
            # show_traversalSights
            if show_traversalSights:
                self.show_traversal_sights(Robot.traversal_sights, Robot.vision_range)
            
            if show_robot:
                self.robot(Robot,yaw=0)
            
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
                self.path(skeleton_path, ls_sp)
            
            if show_approximately_shortest_path:
                self.path(asp, ls_asp)
            
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
        