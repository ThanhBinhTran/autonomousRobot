from Robot_lib import *
from Program_config import *
from matplotlib import patches
rel_tol = 0.0000001
#import matplotlib.pyplot as plt

def plot_sight(plt, x, y, pair, cl = "g", alpha = 0.3, linestyle = ":"):
    ptA, ptB = pair
    plt.fill([x, ptA[0], ptB[0]], [y, ptA[1], ptB[1]], color = cl, alpha = alpha, linestyle = linestyle)
    
def draw_true_sight(plt, x, y, true_sight, cl="g", ls_ts="-"):
    for point in true_sight:
        plot_sight(plt, x, y, point, cl, 0.3, ls_ts)
        
def draw_blind_sight(plt, x, y, blind_sight):
    for point in blind_sight:
        plot_sight(plt, x, y, "g", 0.3, ":")
        
def draw_true_blind_sight(plt, x, y, true_sight, blind_sight):
    draw_true_sight(plt, x, y, true_sight)
    draw_blind_sight(plt, x, y, blind_sight)
        
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

def is_inside_angle_area(check_angle, angle_circle):
    return -1

def get_true_is_circle_pairs( center, radius, true_pair):
    """ get true intersection between true sight and circle -> circle pairs """
    spt0, spt1 = true_pair
    x, y = center
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

    # order by angle
    #if angle0 < angle1:
    #    result = [cpoints[0],cpoints[1]]
    #else:
    #    result = [cpoints[1],cpoints[0]]
    result = [cpoints[0],cpoints[1]]
    return result

def get_close_cpairs(ref_cpairs):
    i = 0
    while i < len(ref_cpairs) -1:
        r_cpairs = ref_cpairs[i] # reference pair
        #print ("__________", r_cpairs)
        j = i + 1
        while j < len(ref_cpairs) :
            c_cpairs = ref_cpairs[j]
            result = mutual_point(r_cpairs, c_cpairs) 
            #print (result)
            new_cpairs = []
            if   result[0][0]:
                new_cpairs = [r_cpairs[1], c_cpairs[1]]
            elif result[0][1]:
                new_cpairs = [c_cpairs[0], r_cpairs[1]]
            elif result[1][0]:
                new_cpairs = [r_cpairs[0], c_cpairs[1]]
            elif result[1][1]:
                new_cpairs = [r_cpairs[0], c_cpairs[0]]
            
            if len( new_cpairs) > 0:
                ref_cpairs[i] = new_cpairs
                ref_cpairs.pop(j)
                #ref_cpairs[j] = []
                i = i - 1
                break
            j = j + 1
        i = i + 1
    return ref_cpairs
    
def get_open_cpairs(center, close_cpairs):
    i = 0
    open_cpairs = []
    while i < len(close_cpairs) -1:
        r_cpairs = close_cpairs[i] # reference pair
        j = i + 1
        while j < len(close_cpairs) :
            c_cpairs = close_cpairs[j]
            found, ridx, cidx = open_sight(center, i, j, close_cpairs) 
            if found:
                open_cpairs.append([close_cpairs[i][ridx],close_cpairs[j][cidx]])
                
                #plot_line(plt, [close_cpairs[i][ridx],close_cpairs[j][cidx]], "-or")
                # update i element by wider one
                # remove j 
                # cidx = 1 , new = 0 and vs
                close_cpairs[i] = [close_cpairs[i][1 - ridx],close_cpairs[j][1 - cidx]]
                close_cpairs.pop(j)
                continue # begin new check with same j
            j = j + 1
        i = i + 1
        break
    
    if len(close_cpairs) > 0: # the last 1
        open_cpairs.append(close_cpairs.pop(0))
    return open_cpairs
    
def is_open_sight(center, i, j, c_cpairs, r_cpairs, remaining_pts):
    # i for ref, j for check
    i_o = 1 - i # other
    j_o = 1 -j  # other
    in1 = inside_angle_area(c_cpairs[j_o], center, (c_cpairs[j],r_cpairs[i]))
    in2 = inside_angle_area(r_cpairs[i_o], center, (c_cpairs[j],r_cpairs[i]))
    in_result = [inside_angle_area(pt, center, (c_cpairs[j],r_cpairs[i])) for pt in remaining_pts]
    in3 = -1
    for in_status in in_result: 
        if in_status >=0: # inside
            in3 = 1
            break
    if in1 >= 0 or in2 >=0 or  in3>=0:
        #print ("debug__________[NOT are open sight]",c_cpairs[j], r_cpairs[i])
        return False
    else:
        #print ("debug__________[open sight]",c_cpairs[j], r_cpairs[i])
        return True
        
def open_sight(center, i, j, close_cpairs):
    # i is always greater than j
    r_cpairs = close_cpairs[i]
    c_cpairs = close_cpairs[j]
    
    #for i in range(len( close_cpairs)):
    #    plot_point(plt, close_cpairs[i][0], ls=".b")
    #    print (close_cpairs[i][0])
    #plot_point(plt, c_cpairs[0], ls="xr")
    #plot_point(plt, r_cpairs[0], ls="xr")
    
    # try 00 first
    remaining_pts = []
    for z in range(j+1, len( close_cpairs)):
        remaining_pts.append(close_cpairs[z][0])
        remaining_pts.append(close_cpairs[z][1])
    
    found = True
    ridx, cidx = (0,0)
    # i first for ref, j second for check
    if   is_open_sight(center, 0, 1, c_cpairs, r_cpairs, remaining_pts): ridx, cidx = (0,1)
    elif is_open_sight(center, 1, 0, c_cpairs, r_cpairs, remaining_pts): ridx, cidx = (1,0)
    elif is_open_sight(center, 1, 1, c_cpairs, r_cpairs, remaining_pts): ridx, cidx = (1,1)
    elif is_open_sight(center, 0, 0, c_cpairs, r_cpairs, remaining_pts): ridx, cidx = (0,0)
    else:
        found = False
    return found, ridx, cidx


def point_isclose(q, p):
    return point_dist(p, q) < rel_tol
    
def mutual_point(pts, ref_pts):
    dist_p0r0 = point_isclose(pts[0], ref_pts[0])
    dist_p0r1 = point_isclose(pts[0], ref_pts[1])
    dist_p1r0 = point_isclose(pts[1], ref_pts[0])
    dist_p1r1 = point_isclose(pts[1], ref_pts[1])
        
    result_0 = [dist_p0r0, dist_p0r1]
    result_1 = [dist_p1r0, dist_p1r1]
    return [result_0, result_1]
    
def get_open_points(center, radius, open_cpairs):
    # get active open points
    opts = []
    for pair in open_cpairs:
        midpt = midpoint(pair[0], pair[1])
        pt_is = intersection(center[0], center[1], radius, [center, midpt])
    
        if inside_ls(midpt, [pt_is[0], center]):
            print ("debug__________ddsdsf")
            opts.append(pt_is[0])
        else:
            print ("debug__________ddsdsf")
            opts.append(pt_is[1])
            
    return opts
def get_true_sight_circle(plt, x, y, radius, true_sight):
    center = [x,y]
    true_sight_circle = []
    true_angle_circle = [[0,180]]
    #get_circle_point()
    close_cpairs=[] # all close circle pairs from true_sight 
                    # close_cpairs contains [circle point, its true point, its angle] 

    ref_cpairs = [get_true_is_circle_pairs( (x, y), radius, true_pair) for true_pair in true_sight]
    
    if print_ref_sight:
        print ("ref_cpairs circle pairs", ref_cpairs)
    if show_ref_sight:
        plot_pairs(plt, ref_cpairs, ls=":r")
    

    close_cpairs = get_close_cpairs(ref_cpairs)
    if print_close_sight:
        print_cpairs("Close circle pairs", close_cpairs)
    if show_close_sight:
        plot_pairs(plt, close_cpairs, ls_cs)
        
    open_cpairs = get_open_cpairs(center, close_cpairs)
    if print_open_sight:
        print_cpairs("Close circle pairs", open_cpairs)
    #if show_open_sight:
    #    plot_pairs(plt, open_cpairs, ls_os)

    return open_cpairs
        
def plot_vision(plt, x, y, radius, ox_b, oy_b, true_sight, blind_sight, true_sight_circle):
    draw_vision_area(plt, x, y, radius)
    draw_arc_area(plt, x, y, radius)
    
    
    if show_true_sight:
        draw_true_sight(plt, x, y, true_sight, cl_ts, ls_ts) 
        
    if show_open_sight:
        draw_true_sight(plt, x, y, true_sight_circle)
        
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
    
def plot_points(plt, ls="xr"):
    plt.plot(points[0,:], points[:,1], ls)