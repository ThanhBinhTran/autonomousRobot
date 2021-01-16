from Robot_lib import *
from Robot_draw_lib import *
from Program_config import *
import matplotlib.pyplot as plt

rel_tol = 0.0000001
    
def devide_sight_ABCD(center, A, B, C, D): #  line [A,C], [B, D]
    # divide sight into 3 different parts [R_C0_R]
    AC_BO_point = line_intersection([A,C], [B, center])
    BD_CO_point = line_intersection([B,D], [C, center])
    plt.plot (AC_BO_point[0],AC_BO_point[1], "+b")
    plt.plot (BD_CO_point[0],BD_CO_point[1], "+b")
    mid_sight = []
    blind_sight = []
    if point_dist(center, AC_BO_point) < point_dist(center, B): # A - C is closer center than B
        #print ("AC is closer", point_dist(center, AC_BO_point), "<>", point_dist(center, B))
        devided_sight = [[A,C], [BD_CO_point, D]]
        blind_sight = [B,BD_CO_point]
    else:
        #print ("AC is closer", point_dist(center, AC_BO_point), "<>", point_dist(center, B))
        devided_sight = [[A,AC_BO_point], [B,D]]
        blind_sight = [AC_BO_point,C]

    return devided_sight, blind_sight
        
def detect_blind_sight(center, ref_sight, check_sight):
    """ check sight  --> C0, C1
        reference sight -->  R0 R1 """
    true_sight = []
    blind_sight = []
    
    r_blind = False   # true if reference sight is blind sight
    c_blind = False   # true if check_sight is blind sight
    d_sight = False   # true if there need to divide (ref, check) area in to 2 parts
                      #                (ref_sub_part, common_ref/check_part, check_sub_path)
                      #             then remove ref, check sight , and create 2 new sight 
    divided_sight = []
    
    pointC_0 = inside_angle_area(check_sight[0], center, ref_sight)
    pointC_1 = inside_angle_area(check_sight[1], center, ref_sight)
    
    #print ("-->ref {0} check {1} c1 c2 {2}, {3}".format (ref_sight, check_sight, pointC_0, pointC_1))
    #if pointC_0 >= 0:
    #    plt.plot(check_sight[0][0], check_sight[0][1], "1r")
    #if pointC_1 >= 0:
    #    plt.plot(check_sight[1][0], check_sight[1][1], "1r")
    
    """ 
        reference sight fully coverages check sight
            -> if case
        reference sight and check sight have mutual edge: 
            -> elif pointC_0 == 0 or pointC_1 == 0
        reference sight and check sight have mutual area 
        # C0 inside, C1 outside
            -> elif pointC_0 > 0 and pointC_1 < 0:
        reference sight and check sight have mutual area 
        # C1 inside, C0 outside
            -->elif pointC_0 < 0 and pointC_1 > 0:
        reference sight and check sight are 2 distinct area 
            ---> else:   # both Check C0 C1 are outside
    """
    if pointC_0 >= 0 and pointC_1 >= 0: # reference sight fully coverages check_sight
        # if check 
        c_blind = True
        blind_sight.append(check_sight)

    elif pointC_0 == 0 or pointC_1 == 0: # ether C0 or C1 is at the boundary segment
        """ check if check sight fully coverages ref_sight """
        pointR_0 = inside_angle_area(ref_sight[0], center, check_sight)
        pointR_1 = inside_angle_area(ref_sight[1], center, check_sight)
        if pointR_0 > 0 or pointR_1 > 0: # ref sight is inside check sight
            r_blind = True
            blind_sight.append(ref_sight)
        # else ref and check are 2 distinct 
        
    elif pointC_0 > 0 and pointC_1 < 0:
        """ check if R0 is inside area of [R1, C1]
            if R0 is outside then R1 is inside area of [R0, C1] for sure
        """
        d_sight = True
    
        pointR_0 = inside_angle_area(ref_sight[0], center, [ref_sight[1], check_sight[1]])
        if pointR_0 >=0 :  # R1 C0 R0 C1 
            divided_sight, b_sight = devide_sight_ABCD(center, ref_sight[1], check_sight[0], ref_sight[0], check_sight[1])
        else: # R0 C0 R1 C1 
            divided_sight, b_sight = devide_sight_ABCD(center, ref_sight[0], check_sight[0], ref_sight[1], check_sight[1])
        blind_sight.append (b_sight)  
    
    elif pointC_0 < 0 and pointC_1 > 0: 
        """ check if R0 is inside area of [R1, C0]
            if R0 is outside then R1 is inside area of [R0, C0] for sure
        """
        d_sight = True
       
        pointR_0 = inside_angle_area(ref_sight[0], center, [ref_sight[1], check_sight[0]])
        if pointR_0 >=0 :  # R1 C1 R0 C0
            #print ("____4      1")
            divided_sight, b_sight = devide_sight_ABCD(center, ref_sight[1], check_sight[1], ref_sight[0], check_sight[0])
        else: # R0 C1 R1 C0
            divided_sight, b_sight = devide_sight_ABCD(center, ref_sight[0], check_sight[1], ref_sight[1], check_sight[0])
        blind_sight.append (b_sight) 
        
    else:   # both Check C0 C1 are outside
        # Check if ref_sight is inside Check_sight,
        pointR_0 = inside_angle_area(ref_sight[0], center, [check_sight[1], check_sight[0]])
        if pointR_0 >=0: # C R C
            r_blind = True

    return blind_sight, divided_sight, r_blind, c_blind, d_sight
    
def remove_blind_sight(center, boundary_points):
    true_sight = boundary_points
    blind_sight = []
    
    i = 0 # start with index = 0
    while i < len(true_sight) -1:
        j = i + 1
        while j < len(true_sight) :
            #print_pairs ("BEFORE: true sight", true_sight)
            bs, ds, r_blind, c_blind, d_sight = detect_blind_sight(center, true_sight[i], true_sight[j])

            #print_pairs (" [Local] blind sight", bs)
            #print_pairs(" [Local] divide sight", ds)
            #print ("status of sight: r {0}, c {1}, d {2}".format(r_blind, c_blind, d_sight))
            blind_sight.extend(bs)
            if d_sight:  # separate into 2 new sight
                true_sight[i] = ds[0]
                true_sight[j] = ds[1]
                i = i - 1
                break
            elif c_blind: # remove check sight cause it's blind sight
                true_sight.pop(j)
                j = j - 1
            elif r_blind: # replace ref sight by check sight cause it's blind sight
                get_check = true_sight.pop(j)
                true_sight[i] = get_check
                i = i - 1
                break
            #print_pairs ("AFTER: true sight", true_sight) 
            j += 1
        i += 1 
    return true_sight, blind_sight
    
def get_true_blind_sight(x, y, boundary_points):
    true_sight = []
    blind_sight = []
    
    if len( boundary_points ) >=2:
        true_sight, blind_sight = remove_blind_sight([x, y], boundary_points)
    else: # only 1 sight then no collision (blind sight) in the given boundary_points
        true_sight = boundary_points
    return true_sight, blind_sight
    
def get_all_boardary_pairs(x, y, config, ox_b, oy_b):
    """ find all boundary pairs among all obstacle line segments and circle """
    boundary_points = []
    for i in range(len(ox_b)-1):
        ptA = [ox_b[i], oy_b[i]]
        ptB = [ox_b[i+1], oy_b[i+1]]
        
        is_points = intersection(x, y, config.robot_vision, [ptA, ptB])
        
        if len(is_points) > 0:
            if show_intersection_line:
                plot_line(plt, is_points, ls_is)
        
            """ find boundary pair between a single line segment and circle """
            boundary_point = []
            
            for point in is_points:
                is_pt_in =  inside_ls(point, [ptA, ptB])
                if is_pt_in is not None:  # found intersection point is inside the line segment
                    boundary_point.append(is_pt_in)
                else:                      # intersection point is not inside the line segment
                    ptA_in = inside_ls(ptA, is_points)
                    if ptA_in is not None: # found intersection point is inside the line segment
                        boundary_point.append(ptA_in)
                    ptB_in = inside_ls(ptB, is_points)
                    if ptB_in is not None: # found intersection point is inside the line segment
                        boundary_point.append(ptB_in)
                        
            if len (boundary_point) > 0:
                boundary_points.append( [boundary_point[0],boundary_point[1]])

    return boundary_points
    
def get_true_sight(x, y, config, ox_b, oy_b):
    # get boundary pairs, [start point x,  end point x], x start < x end
    boundary_points = get_all_boardary_pairs(x, y, config, ox_b, oy_b)
    
    if print_boundary_points:
        print_pairs ("boundary_points", boundary_points)
    if show_boundary_points:
        plot_pairs(plt, boundary_points, ls_bp)    
    
    true_sight, blind_sight = get_true_blind_sight(x, y, boundary_points)
    #print_pairs ("true_sight", true_sight)
    #print_pairs ("blind_sight", blind_sight)
    

    return true_sight, blind_sight

def inside_true_sight(pt, center, radius, true_sight):
    outside = True
    if point_dist(pt, center) <= radius: # inside vision area
        outside = False
        # check if pt is inside true sight angle
        inside_open_sight = True
        visible = False
        for ts_pair in true_sight:
            pt_in = inside_angle_area(pt, center, ts_pair)
            if pt_in >= 0:  # inside angle of true sight
                inside_open_sight = False
                pt_in = inside_angle_area( pt, ts_pair[0], (center, ts_pair[1]))
                if pt_in >= 0: 
                    visible = True
                    #print ("found close ", pt, true_sight)
                    return True
                else:
                    #print ("found in blind sight ", pt, true_sight)
                    return False
        #print ("found open", pt, true_sight)
        return True
    else:
        #print ("Outside", pt)
        return False
    #return not outside and (inside_open_sight or visible)








                    