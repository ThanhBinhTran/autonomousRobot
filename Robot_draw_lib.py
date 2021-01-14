from Robot_lib import *
from matplotlib.patches import Arc

def plot_sight(plt, x, y, sight, cl = "g", alpha = 0.3, linestyle = ":g"):
    plt.fill([x, sight[0][0],sight[1][0]], [y, sight[0][1], sight[1][1]], color = cl, alpha = alpha, linestyle = linestyle)
    
def draw_true_sight(plt, x, y, true_sight):
    for point in true_sight:
        plot_sight(plt, x, y, point, "m", 0.3, ":")
        
def draw_true_blind_sight(plt, x, y, true_sight, blind_sight):
    print ("true_sight", true_sight)
    for point in true_sight:
        plot_sight(plt, x, y, point, "m", 0.3, ":")

    print ("blind_sight",blind_sight)
    for point in blind_sight:
        plot_sight(plt, x, y, point, "g", 0.3, ":")

    print ("DONE")

def draw_blind_sight(plt, x, y, blind_sight):
    for point in blind_sight:
        plot_sight(plt, x, y, "g", 0.3, ":")
        
def draw_vision_area(plt, x, y, radius):
    """ draw a circle that limits the vision of robot """ 
    vision = plt.Circle((x, y), radius, color="red", linestyle  = ":", fill=False)
    #alpha = 0.3
    #vision = plt.Circle((x, y), radius, color="red", linestyle  = "--", alpha = 0.3)
    plt.gcf().gca().add_artist(vision)
def draw_arc_area(plt, x, y, radius):
    # Draw arc with arrow.
    arc_radius = radius
    angle_begin = 0
    angle_end =  90
    
    arc = Arc((x, y), 
            arc_radius*2, arc_radius*2,  # ellipse width and height
            theta1=angle_begin, theta2=angle_end,      # from 0 to angle
            linestyle='-', color = "b")
    plt.axes().add_patch(arc)

def is_inside_angle_area(check_angle, angle_circle):
    diff_angle_1 = abs(check_angle - angle_circle[0])
    diff_angle_2 = abs(check_angle - angle_circle[1])
    if diff_angle_1 <= 0.000005:
        return 0
    elif check_angle > angle_circle[0] and check_angle < angle_circle[1]:
        return 1
    return -1

def get_true_sight_circle(plt, x, y, radius, true_sight):
    true_sight_circle = []
    true_angle_circle = [[0,180]]
    #get_circle_point()
    cpoints=[]
    print ("Passed here")
    for pair in true_sight:
        # process angle
        angle1 = cal_signed_angle([pair[0][0]-x,pair[0][1]-y],[1,0])  # cal angle base on ox axis
        angle2 = cal_signed_angle([pair[1][0]-x,pair[1][1]-y],[1,0])  # cal angle base on ox axis
        # if angle1 is inside and angle2 is inside
        
        in_a1 = is_inside_angle_area(angle1, true_angle_circle[0])
        in_a2 = is_inside_angle_area(angle2, true_angle_circle[0])
        #if in_a1 >= 0 and in_a2 >= 0: # both true sight points are inside circle_point, then remove that sight from circle
        #    # remove 1 - 2 from true_angle_circle
        #
        #if angle1 > 0 and angle2 > 0:
            
        for point in pair:
            cPoints = intersection(x, y, radius, [point,[x, y]])
            if is_inside_ls(cPoints[0], [point,[x, y]]):
                cpoints.append(cPoints[0])
            else:
                cpoints.append(cPoints[1])
    print ("Passed here  11111")
    for point in cpoints:
        plt.plot(point[0],point[1], "*r")
    return true_sight_circle
        
def plot_vision(plt, x, y, radius, ox_b, oy_b, true_sight, blind_sight):
    
    draw_vision_area(plt, x, y, radius)
    draw_arc_area(plt, x, y, radius)
    print_sight ("true_sight_p1:", true_sight)
    print_sight ("blind_sight_p2:", blind_sight)

    true_sight_circle = get_true_sight_circle(plt, x, y, radius, true_sight)
    draw_true_sight(plt, x, y, true_sight) 
    
