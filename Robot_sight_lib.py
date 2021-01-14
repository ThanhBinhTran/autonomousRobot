from Robot_lib import *
from Robot_draw_lib import *
import matplotlib.pyplot as plt
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
    #draw_true_blind_sight(plt,center[0], center[1], devided_sight, [blind_sight]) 
    
    return [devided_sight, blind_sight]
        
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
    
    pointC_0 = is_inside_area(check_sight[0], center, ref_sight)
    pointC_1 = is_inside_area(check_sight[1], center, ref_sight)
    
    print ("_ ref {0} check {1} c1 c2 {2}, {3}".format (ref_sight, check_sight, pointC_0, pointC_1))
    if pointC_0 >= 0:
        plt.plot(check_sight[0][0], check_sight[0][1], "1r")
    if pointC_1 >= 0:
        plt.plot(check_sight[1][0], check_sight[1][1], "1r")
    if pointC_0 >= 0 and pointC_1 >= 0: # whole check_sight are inside ref_sight, true blind
        #print ("____1")
        true_sight.append(ref_sight)
        blind_sight.append(check_sight)
        c_blind = True
    elif pointC_0 == 0 or pointC_1 == 0: # ether C0 or C1 is at the boundary segment
        """ check if C0 C1 coverages ref_sight """
        #print ("____2")
        pointR_0 = is_inside_area(ref_sight[0], center, check_sight)
        pointR_1 = is_inside_area(ref_sight[1], center, check_sight)
        if pointR_0 < 0 or pointR_1 < 0: # ref sight is outside of the area of check
            #print ("____2 1")
            true_sight.append(ref_sight)
            true_sight.append(check_sight)
        else: # C1/C0 R0 [R1|C0/C1]
            #print ("____2 1")
            true_sight.append(check_sight)
            blind_sight.append(ref_sight)
            r_blind = True
    elif pointC_0 > 0 and pointC_1 < 0: # C0 inside, C1 outside
        """ check if R0 is inside area of [R1, C1]
            if R0 is outside then R1 is inside area of [R0, C1] for sure
        """
        #print ("____3")
        # remove 2 sights (ref, check) to create 3 new ones
        d_sight = True
        
        b_sight = []
        
        pointR_0 = is_inside_area(ref_sight[0], center, [ref_sight[1], check_sight[1]])
        if pointR_0 >=0 :  # R1 C0 R0 C1 
            #print ("____3    1")
            [divided_sight, b_sight] = devide_sight_ABCD(center, ref_sight[1], check_sight[0], ref_sight[0], check_sight[1])
        else: # R0 C0 R1 C1 
            #print ("____3     2")
            [divided_sight, b_sight] = devide_sight_ABCD(center, ref_sight[0], check_sight[0], ref_sight[1], check_sight[1])
        blind_sight.append (b_sight)  
    elif pointC_0 < 0 and pointC_1 > 0: # C1 inside, C0 outside
        #print ("____4")
        """ check if R0 is inside area of [R1, C0]
            if R0 is outside then R1 is inside area of [R0, C0] for sure
        """
        # remove 2 sights (ref, check) to create 3 new ones
        d_sight = True
        b_sight = []
        
        pointR_0 = is_inside_area(ref_sight[0], center, [ref_sight[1], check_sight[0]])
        if pointR_0 >=0 :  # R1 C1 R0 C0
            #print ("____4      1")
            [divided_sight, b_sight] = devide_sight_ABCD(center, ref_sight[1], check_sight[1], ref_sight[0], check_sight[0])
        else: # R0 C1 R1 C0
            #print ("____4       2")
            [divided_sight, b_sight] = devide_sight_ABCD(center, ref_sight[0], check_sight[1], ref_sight[1], check_sight[0])
        blind_sight.append (b_sight) 
    else:   # both Check C0 C1 are outside
        # Check if ref_sight is insde Check_sight,
        pointR_0 = is_inside_area(ref_sight[0], center, [check_sight[1], check_sight[0]])
        if pointR_0 >=0: # C R C
            true_sight.append (check_sight)
            blind_sight.append (ref_sight)
            r_blind = True
        else:  # 2 distinct sights
            true_sight.append (ref_sight)
            true_sight.append (check_sight)
            
        #print ("____5")
        
        
    return [true_sight, blind_sight, divided_sight, r_blind, c_blind, d_sight]
    
def remove_blind_sight(center, boundary_points):
    true_sight = []
    blind_sight = []
    i = 0 # start with index = 0
    while i < len(boundary_points) -1:
        #print ("NEW I", i, boundary_points)
        j = i + 1
        while j < len(boundary_points) :
            #print ("BEFORE ", i,j,  len(boundary_points),boundary_points)
            [ts, bs, ds, r_blind, c_blind, d_sight] = detect_blind_sight(center, boundary_points[i], boundary_points[j])
            print_sight ("__true sight:", ts)
            print_sight ("__blind sight:", bs)
            print_sight ("__divide sight:", ds)
            blind_sight.extend(bs)
            # if blind sight is in true sight, then remove it
            for bsight in bs:
                if bsight in true_sight:
                    true_sight.remove(bsight)
                    
            for sight in ts:                                                   # real true sight isn't in blind_sight
                if (sight not in blind_sight) and (sight not in true_sight): # prevent insert duplicate
                    true_sight.append(sight)
                    
            if d_sight:
                # remove ref, check sight in true sight if any
                # replace ref, check sight by new ref, check in boundary_points
                try:
                    true_sight.remove(boundary_points[i])
                    true_sight.remove(boundary_points[j])
                except:
                    print("Not in list")
                boundary_points[i] = ds[0]
                boundary_points[j] = ds[1]
                #print_sight("new boundary_points", boundary_points)
                i = i - 1
                break
            elif c_blind: # remove (lager index of check sight) first if its true blind
                #print ("remove check sight")
                boundary_points.pop(j)
                j = j - 1
            elif r_blind:
                #print ("remove ref sight")
                boundary_points.pop(i)
                i = i - 1
                break
            #print ("->AFTER ", i,j,  len(boundary_points), boundary_points) 
            j += 1
        i += 1 
    return [true_sight, blind_sight]
    
def get_true_blind_sight(x, y, boundary_points):
    true_sight = []
    blind_sight = []
    if len( boundary_points ) >=2:
        [true_sight, blind_sight] = remove_blind_sight([x, y], boundary_points)
    else: # only 1 sight then no collision in the given sight
        true_sight = boundary_points
    return [true_sight, blind_sight]
    
def get_all_boardary_pairs(x, y, config, ox_b, oy_b):
    """ find all boundary pairs among all obstacle line segments and circle """
    boundary_points = []
    print_point("Map obstacle", ox_b, oy_b)
    for i in range(len(ox_b)-1):
        is_points = intersection(x, y, config.robot_vision, [[ox_b[i], oy_b[i]], [ox_b[i+1], oy_b[i+1]]])
        if len(is_points) > 0:
            # draw intersection points
            #for point in is_points:
            #plt.plot(point[0],point[1], ".g"
        
            """ find boundary pair between a single of line segment and circle """
            boundary_point = []
            for point in is_points:
                b_point = is_inside_ls(point, [[ox_b[i],oy_b[i]], [ox_b[i+1], oy_b[i+1]]])
                if b_point is not None:            # found intersection point is inside the line segment
                    boundary_point.append(b_point)
                else:                               # intersection point is not inside the line segment
                    b_point = is_inside_ls([ox_b[i],oy_b[i]], is_points)
                    if b_point is not None:            # found intersection point is inside the line segment
                        boundary_point.append(b_point)
                    b_point = is_inside_ls([ox_b[i+1],oy_b[i+1]], is_points)
                    if b_point is not None:            # found intersection point is inside the line segment
                        boundary_point.append(b_point)
            if len (boundary_point) > 0:
                #print ("boundary_points ", boundary_point)
                boundary_points.append( [boundary_point[0],boundary_point[1]])
    
    #print ("boundary_points: ")
    #for bPair in boundary_points:
        #print (bPair)
    return boundary_points
    
def get_true_sight(x, y, config, ox_b, oy_b):
    boundary_points = get_all_boardary_pairs(x, y, config, ox_b, oy_b)
    print ("debug:", boundary_points)
    [true_sight, blind_sight] = get_true_blind_sight(x, y, boundary_points)
    return [true_sight, blind_sight]
