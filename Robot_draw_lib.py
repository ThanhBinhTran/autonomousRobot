from Robot_lib import *

def plot_sight(plt, x, y, sight, cl = "g", alpha = 0.3, linestyle = ":"):
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
        
def draw_vision_area(plt, x, y, config):
    """ draw a circle that limits the vision of robot """ 
    vision = plt.Circle((x, y), config.robot_vision, color="red", linestyle  = "--", fill=False)
    #alpha = 0.3
    #vision = plt.Circle((x, y), config.robot_vision, color="red", linestyle  = "--", alpha = 0.3)
    plt.gcf().gca().add_artist(vision)

def plot_vision(plt, x, y, config, ox_b, oy_b, true_sight, blind_sight):
    
    draw_vision_area(plt, x, y, config)
    
    print_sight ("true_sight_p1:", true_sight)
    print_sight ("blind_sight_p2:", blind_sight)
    
    draw_true_sight(plt, x, y, true_sight) 
    