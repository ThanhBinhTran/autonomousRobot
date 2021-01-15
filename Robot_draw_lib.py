from Robot_lib import *
from Program_config import *
from matplotlib.patches import Arc

def plot_sight(plt, x, y, pair, cl = "g", alpha = 0.3, linestyle = ":"):
    ptA, ptB = pair
    plt.fill([x, ptA[0], ptB[0]], [y, ptA[1], ptB[1]], color = cl, alpha = alpha, linestyle = linestyle)
    
def draw_true_sight(plt, x, y, true_sight):
    for point in true_sight:
        plot_sight(plt, x, y, point, "g", 0.3, "-")
        
def draw_blind_sight(plt, x, y, blind_sight):
    for point in blind_sight:
        plot_sight(plt, x, y, "g", 0.3, ":")
        
def draw_true_blind_sight(plt, x, y, true_sight, blind_sight):
    draw_true_sight(plt, x, y, true_sight)
    draw_blind_sight(plt, x, y, blind_sight)



        
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
    return -1

def get_true_sight_circle(plt, x, y, radius, true_sight):
    center = [x,y]
    true_sight_circle = []
    true_angle_circle = [[0,180]]
    #get_circle_point()
    close_cpairs=[] # all close circle pairs from true_sight 
                    # close_cpairs contains [circle point, its true point, its angle] 
    for pair in true_sight:
        spt0, spt1 = pair
        # process angle
        
        angle0 = signed_angle_xAxis([spt0[0]-x,spt0[1]-y])  # cal angle base on ox axis
        angle1 = signed_angle_xAxis([spt1[0]-x,spt1[1]-y])  # cal angle base on ox axis

        cpoints = []
        # circle point 0
        cpt_is = intersection(x, y, radius, [spt0, center])
        if inside_ls(cpt_is[0], [spt0, center]):
            cpoints.append(cpt_is[0])
        else:
            cpoints.append(cpt_is[1])
        
        # circle point 1
        cpt_is = intersection(x, y, radius, [spt1,center])
        if inside_ls(cpt_is[0], [spt1,center]):
            cpoints.append(cpt_is[0])
        else:
            cpoints.append(cpt_is[1])
        if angle0 < angle1:
            close_cpairs.append(([cpoints[0], spt0, angle0],[cpoints[1], spt1, angle1])) 
        else:
            close_cpairs.append(([cpoints[1], spt1, angle1],[cpoints[0], spt0, angle0]))
    
    print_cpairs("Close circle pairs", close_cpairs)
    
    for pairs in close_cpairs:
        plt.plot( (pairs[0][0][0],pairs[1][0][0]),(pairs[0][0][1],pairs[1][0][1]), "-b")
        
    open_cpairs =  []# its complementary of close_cpairs
                               # open_cpairs contains [circle point, its angle]
    ref_cpairs = close_cpairs
    i = 0
    while i < len(ref_cpairs) -1:
        r_pt_s, r_pt_e = ref_cpairs[i] # reference pair
        j = i + 1
        while j < len(ref_cpairs) :
            c_pt_s, c_pt_e = ref_cpairs[j] # check pair
            # check if 2 sight have mutual point by comparing angle
            # check start point (smaller angle) of ref and end point of 
            if math.isclose(r_pt_s[2],c_pt_e[2]): 
                ref_cpairs[i] = c_pt_s,r_pt_e   # extend ref to check
                ref_cpairs.pop(j)              # remove check region
                i = i - 1
                break
            elif math.isclose(r_pt_e[2],c_pt_s[2]): 
                ref_cpairs[i] = r_pt_s,c_pt_e   # extend ref to check
                ref_cpairs.pop(j)              # remove check region
                i = i - 1
                break
            
            # consider [start ref(i); start check(i)] region
            # check end ref (i) or end check(j) not inside [sr,sc] region
            er_inside = inside_angle_area(r_pt_e[0], center, [r_pt_s[0],c_pt_s[0]]) 
            ec_inside = inside_angle_area(c_pt_e[0], center, [r_pt_s[0],c_pt_s[0]]) 
            inside = False
            if not er_inside and not ec_inside:
                h = j + 1
                while h < len(ref_cpairs) :
                    h_pt_s, h_pt_e = ref_cpairs[h] # remaining pairs
                    if inside_angle_area(h_pt_s[0], center, [r_pt_s[0],c_pt_s[0]]):
                        inside = True
            if not inside:
                open_cpairs.append((r_pt_s,c_pt_s))
                ref_cpairs[i] = (c_pt_s,r_pt_e)   # extend ref to check
                ref_cpairs.pop(j)              # remove check region
                i = i - 1
                break
                
            # consider [start ref(i); end check(i)] region
            # check end ref (i) or start check(j) not inside [sr,sc] region
            er_inside = inside_angle_area(r_pt_e[0], center, [r_pt_s[0],c_pt_e[0]]) 
            sc_inside = inside_angle_area(c_pt_s[0], center, [r_pt_s[0],c_pt_e[0]]) 
            inside = False
            if not er_inside and not sc_inside:
                h = j + 1
                while h < len(ref_cpairs) :
                    h_pt_s, h_pt_e = ref_cpairs[h] # remaining pairs
                    if inside_angle_area(h_pt_s[0], center, [r_pt_s[0],c_pt_e[0]]):
                        inside = True
            if not inside:
                open_cpairs.append((r_pt_s,c_pt_e))
                ref_cpairs[i] = (c_pt_s,r_pt_e)   # extend ref to check
                ref_cpairs.pop(j)              # remove check region
                i = i - 1
                break
                
            j += 1
        i += 1
    #for pairs in ref_cpairs:
    #    plt.plot( (pairs[0][0][0],pairs[1][0][0]),(pairs[0][0][1],pairs[1][0][1]), "-<r")
        
    print_cpairs("reference circle pairs", ref_cpairs) 
    print_cpairs("Open circle pairs", open_cpairs)
    
    return true_sight_circle
        
def plot_vision(plt, x, y, radius, ox_b, oy_b, true_sight, blind_sight):
    draw_vision_area(plt, x, y, radius)
    #draw_arc_area(plt, x, y, radius)
    true_sight_circle = get_true_sight_circle(plt, x, y, radius, true_sight)
    if show_true_sight:
        draw_true_sight(plt, x, y, true_sight) 
    
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
        plot_lines(plt, pair, ls)
    
def plot_points(plt, points, ls="xr"):
    xs = [i[0] for i in points]
    ys = [i[1] for i in points]
    plt.plot(x, y, ls)