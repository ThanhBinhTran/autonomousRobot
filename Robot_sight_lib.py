from Robot_lib import *
from Robot_draw_lib import *
from Program_config import *
import matplotlib.pyplot as plt

rel_tol = 0.0000001
    
def devide_sight_ABCD(center, A, B, C, D): #  line [A,C], [B, D], B inside AC but D outside
    # divide sight into 3 different parts [R_C0_R]
    AC_BO_point = line_intersection([A,C], [B, center])
    BD_CO_point = line_intersection([B,D], [C, center])
    plt.plot (AC_BO_point[0],AC_BO_point[1], "+b")
    plt.plot (BD_CO_point[0],BD_CO_point[1], "+b")
    mid_sight = []
    if point_dist(center, AC_BO_point) < point_dist(center, B): # A - C is closer center than B
        #print ("AC is closer", point_dist(center, AC_BO_point), "<>", point_dist(center, B))
        devided_sight = [[A,C], [BD_CO_point, D]]
    else:
        #print ("AC is closer", point_dist(center, AC_BO_point), "<>", point_dist(center, B))
        devided_sight = [[A,AC_BO_point], [B,D]]

    return devided_sight
        
def detect_blind_sight(center, ref_sight, check_sight):
    """ check sight  --> C0, C1
        reference sight -->  R0 R1 """
    r = ref_sight
    c = check_sight
    true_sight = []
    
    r_blind = False   # true if reference sight is blind sight
    c_blind = False   # true if check_sight is blind sight
    d_sight = False   # true if there need to divide (ref, check) area in to 2 parts
                      #                (ref_sub_part, common_ref/check_part, check_sub_path)
                      #             then remove ref, check sight , and create 2 new sight 
    divided_sight = []
    
    # get inside status of c1, c0
    c0_in, c0_code = inside_angle_area(c[0], center, [r[0], r[1]])
    c1_in, c1_code = inside_angle_area(c[1], center, [r[0], r[1]])
       
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
    print ("check me ____",c[0], c0_in, c0_code,"-" ,c[1], c1_in, c1_code)
    if c0_in and c1_in: # reference sight fully coverages check_sight
        print ("check me  1")
        if c0_code <=1 and c1_code == 2: # mutual c0 while c1 inside
            print ("check me  1  1")
            # check if c1 is inside (r0, center r1)
            c1_in  = inside_angle_area(c[1], r[0] , [center, r[1]])[0]
            if c1_in:
                print ("check me  1  1 1")
                is_pt = line_intersection([center, c[1]],[r[0], r[1]])
                if c0_code == 0:
                    divided_sight = [ [is_pt, r[1]], [c[0], c[1]] ]
                else:
                    divided_sight = [ [r[0], is_pt], [c[0], c[1]] ]
                plt.plot((is_pt[0]),(is_pt[1]), "or")
                d_sight = True
            else:
                print ("check me  1  1 2")
                c_blind = True
        elif c0_code == 2 and c1_code <=1: # mutual c1 while c0 inside
            print ("check me  1  2")
            # check if c1 is inside (r0, center r1)
            c0_in  = inside_angle_area(c[0], r[0] , [center, r[1]])[0]
            if c0_in:
                print ("check me  1  2   1")
                is_pt = line_intersection([center, c[0]],[r[0], r[1]])
                if c1_code == 0:
                    divided_sight = [[r[0], is_pt], [c[0], c[1]] ]
                else:
                    divided_sight = [[is_pt, r[1]], [c[0], c[1]] ]
                d_sight = True
            else:
                print ("check me  1  2   2")
                c_blind = True
        elif (c0_code == 0 and c1_code == 1) or (c0_code == 1 and c1_code == 0):
            # exactly same 
            c0_in  = inside_angle_area(c[0], r[0] , [center, r[1]])[0]
            c1_in  = inside_angle_area(c[1], r[0] , [center, r[1]])[0]
            if c0_in and c0_in: # c0 c1 is closer
                r_blind = True
            else:
                c_blind = True
        else:
            print ("check me8")
            c_blind = True
            
    elif (c0_in and c0_code <2) or (c1_in and c1_code<2): # ether C0 or C1 is at the boundary segment
        print ("check me  2  0   0", c0_in ,c0_code, c1_in, c1_code)
        """ check if check sight fully coverages ref_sight """
        r0_in, r0_code = inside_angle_area(r[0], center, [c[0], c[1]])
        r1_in, r0_code = inside_angle_area(r[1], center, [c[0], c[1]])
        if r0_in and r1_in: # ref sight is inside check sight
            print ("check me  2  1   0")
            r_blind = True
       
    elif c0_in and not c1_in :
        print ("check me  3  0   0")
        """ check if R0 is inside area of [R1, C1]
            if R0 is outside then R1 is inside area of [R0, C1] for sure
        """
        d_sight = True
    
        r0_in, r0_code = inside_angle_area(r[0], center, [r[1], c[1]])
        if r0_in :  # R1 C0 R0 C1 
            print ("check me  3  1   0")
            divided_sight = devide_sight_ABCD(center, r[1], c[0], r[0], c[1])
        else: # R0 C0 R1 C1 
            print ("check me  3  2   0")
            divided_sight = devide_sight_ABCD(center, r[0], c[0], r[1], c[1])

    elif not c0_in  and c1_in: 
        print ("check me  4  0   0")
        """ check if R0 is inside area of [R1, C0]
            if R0 is outside then R1 is inside area of [R0, C0] for sure
        """
        d_sight = True
       
        r0_in, r0_code = inside_angle_area(r[0], center, [r[1], c[0]])
        if r0_in :  # R1 C1 R0 C0
            print ("check me  4  1   0")
            #print ("____4      1")
            divided_sight = devide_sight_ABCD(center, r[1], c[1], r[0], c[0])
        else: # R0 C1 R1 C0
            print ("check me  4  2   0")
            divided_sight = devide_sight_ABCD(center, r[0], c[1], r[1], c[0])
       
    else:   # both Check C0 C1 are outside
        print ("check me  5  0   0")
        # Check if ref_sight is inside Check_sight,
        r0_in, r0_code = inside_angle_area(r[0], center, [c[1], c[0]])
        if r0_in: # C R C
            print ("check me  5  1   0")
            r_blind = True

    return divided_sight, r_blind, c_blind, d_sight
    
def remove_blind_sight(center, boundary_points):
    true_sight = boundary_points
    
    i = 0 # start with index = 0
    while i < len(true_sight) -1:
        j = i + 1
        while j < len(true_sight) :
            print ("Checking ", true_sight[i], "-->", true_sight[j])
            ds, r_blind, c_blind, d_sight = detect_blind_sight(center, true_sight[i], true_sight[j])
            print_pairs(" [Local] divide sight", ds)
            print ("status of sight: r {0}, c {1}, d {2}".format(r_blind, c_blind, d_sight))
            if d_sight:  # separate into 2 new sight
                true_sight[i] = ds[0]
                true_sight[j] = ds[1]
                #i = i - 1
                #break
            elif c_blind: # remove check sight cause it's blind sight
                true_sight.pop(j)
                j = j - 1
            elif r_blind: # replace ref sight by check sight cause it's blind sight
                get_check = true_sight.pop(j)
                # update i so need to recheck
                true_sight[i] = get_check
                i = i-1
                break
 
            j += 1
        i += 1 
    return true_sight
    
def get_true_blind_sight(x, y, boundary_points):
    true_sight = []
    
    if len( boundary_points ) >=2:
        true_sight = remove_blind_sight([x, y], boundary_points)
    else: # only 1 sight then no collision (blind sight) in the given boundary_points
        true_sight = boundary_points
    return true_sight
    
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
    
    true_sight = get_true_blind_sight(x, y, boundary_points)
    #print_pairs ("true_sight", true_sight)
    return true_sight
    
def inside_global_true_sight(pt, radius, traversal_path):
    print ("len(traversal_path) 123", len(traversal_path))
    if len(traversal_path)>0:
        result = [inside_local_true_sight(pt, x, radius, tsight) for x,tsight,notuse in traversal_path]
        #print ("inside global sight result: ", result)
        return np.sum(result) >= 0
    else:
        return True
def inside_local_true_sight(pt, center, radius, true_sight):
    outside = True
    if point_dist(pt, center) <= radius: # inside vision area
        outside = False
        # check if pt is inside true sight angle
        inside_open_sight = True
        visible = False
        for ts_pair in true_sight:
            pt_in = inside_angle_area(pt, center, ts_pair)[0]
            if pt_in :  # inside angle of true sight
                inside_open_sight = False
                pt_in = inside_angle_area( pt, ts_pair[0], (center, ts_pair[1]))[0]
                if pt_in : 
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
    rcp_len_b = len( ref_cpairs) # number of ref_cpairs at begin
    i = 0
    while i < len(ref_cpairs) -1:
        r_cpairs = ref_cpairs[i] # reference pair
        #print ("__________", r_cpairs)
        j = i + 1
        while j < len(ref_cpairs) :
            c_cpairs = ref_cpairs[j]
            #print ("__________", c_cpairs)
            result = mutual_point(r_cpairs, c_cpairs) 
            print (result)
            new_cpairs = []
            if   result[0][0]:
                #print ("check me #213 0")
                new_cpairs = [r_cpairs[1], c_cpairs[1]]
            elif result[0][1]:
                #print ("check me #213 1")
                new_cpairs = [c_cpairs[0], r_cpairs[1]]
            elif result[1][0]:
                #print ("check me #213 2")
                new_cpairs = [r_cpairs[0], c_cpairs[1]]
            elif result[1][1]:
                #print ("check me #213 3")
                new_cpairs = [r_cpairs[0], c_cpairs[0]]
            
            if len( new_cpairs) > 0:
                ref_cpairs[i] = new_cpairs
                ref_cpairs.pop(j)
                #ref_cpairs[j] = []
                i = i - 1
                break
            j = j + 1
        i = i + 1
    rcp_len_e = len(ref_cpairs)
    if rcp_len_e == 1:
        if rcp_len_b > 1: # all compress into 1
            return [[ref_cpairs[0][0], ref_cpairs[0][1], False]]
        else:
            return [[ref_cpairs[0][0], ref_cpairs[0][1], True]]
    return ref_cpairs
    
def divide_open_cpair(center, inangle, vs, ve):
    return_pairs = []
    print ("divide_open_cpair: ", math.degrees(inangle))
    angle = abs(inangle)
    if angle >= math.pi*4/3:      # 240 degree
    #divide into 3 parts
        r_angle = angle/ 3
        print ("divide into 3 parts, with angle = ", math.degrees(r_angle))
        v1 = rotate_vector_center(center, vs, -r_angle)
        v2 = rotate_vector_center(center, vs, -2*r_angle)
        return_pairs.append([vs, v1])
        return_pairs.append([v1, v2])
        return_pairs.append([v2, ve])
    elif angle >= math.pi*2/3:      # 120 degree:
    #divide into 2 parts
        r_angle = angle/ 2
        print ("divide into 2 parts, with angle = ", math.degrees(r_angle))
        v1 = rotate_vector_center(center, vs, -r_angle)
        return_pairs.append([vs, v1])
        return_pairs.append([v1, ve])
    else: # <= 120 degree
        return_pairs.append([vs, ve])
    return return_pairs
        
def divide_open_cpair_complement (center, cpair):
    return_pairs = []
    angle, vs, ve = get_angle_info(center, cpair[0], cpair[1])
    angle = 2*math.pi - abs(angle)
    return_pairs = divide_open_cpair(center, angle, ve, vs) 
    return return_pairs

def init_open_cpair(center, radius, ptS, ptE):
    # open_cpairs = [point Start, point end , and open point]
    oPt = get_a_open_point_from_a_pair(center, radius, [ptS,ptE])
    return [ptS, ptE, oPt]
    
def get_open_cpairs(center, radius, goal, close_cpairs):
    # declare open circle pairs for open sight
    o_cpairs = []
        
    #clone new close circle pairs
    c_cpairs = [] 
    c_cpairs.extend(close_cpairs)
    
    print (c_cpairs)
    if len(c_cpairs) == 0: # no obstacle detected
        print ("No obstacle detected")
        vector_cg_unit = unit_vector( np.subtract(goal, center))
        vs = np.multiply (vector_cg_unit, radius) + center
        vs = rotate_vector_center(center, vs, math.pi/3)

        pairs_extend = divide_open_cpair_complement (center, [vs, vs])
        print ("Check me 41023", center, (vs[0], vs[0]))
        for pair in pairs_extend:
            o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
                  
    elif len(c_cpairs) == 1: # only a obstacle detected
        print ("only 1 obstacle detected")
        print ("c_cpairs", c_cpairs)
        if c_cpairs[0][2]: # True close detected then divide open sight
            pairs_extend = divide_open_cpair_complement (center, c_cpairs[0])
            for pair in pairs_extend:
                o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
        else: # complement close_cpairs
            o_cpairs.append(init_open_cpair(center, radius, c_cpairs[0][0], c_cpairs[0][1]))
    else:
        i = 0
        while i < len(c_cpairs) -1:
            j = i + 1
            while j < len(c_cpairs) :
                found, ridx, cidx = open_sight(center, i, j, c_cpairs) 
                print ("Check me", i, j, found, ridx, cidx)
                if found:
                    ptA = c_cpairs[i][ridx]
                    ptB = c_cpairs[j][cidx]
                    angle, vs, ve = get_angle_info(center, ptA, ptB)
                    pairs_extend = divide_open_cpair(center, angle, vs, ve)

                    for pair in pairs_extend:
                        o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
                    c_cpairs[i] = [c_cpairs[i][1 - ridx],c_cpairs[j][1 - cidx]]
                    c_cpairs.pop(j)
                    continue # begin new check with same j
                j = j + 1
            i = i + 1
            break
        
        if len(c_cpairs) > 0: # the last 1
            ptA, ptB = c_cpairs[0]
            # get all close point, which are from close pairs, but ptA, ptB
            allPts = []
            for pair in close_cpairs:
                allPts.append(pair[0])
                allPts.append(pair[1])
            allPts.remove(ptA)
            allPts.remove(ptB)
            # check if sight [ptA, ptB] are true open sight or not
            # if not revise sight is the true one
            all_in = [inside_angle_area(pt, center, (ptA, ptB))[0] for pt in allPts]
            print ("check me 012333", all_in)
            
            if np.sum(all_in) == 0: # true open pair
                
                angle, vs, ve = get_angle_info(center, ptA, ptB)
                pairs_extend = divide_open_cpair(center, angle, vs, ve)
                for pair in pairs_extend:
                    o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
            else:
                pairs_extend = divide_open_cpair_complement (center, c_cpairs[0])
                for pair in pairs_extend:
                    o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
    print ("o_cpairs", o_cpairs)
    return o_cpairs
    
    
def is_open_sight(center, i, j, c_cpairs, r_cpairs, remaining_pts):
    # i for ref, j for check
    i_o = 1 - i # other
    j_o = 1 -j  # other
    in1 = inside_angle_area(r_cpairs[i_o], center, (c_cpairs[j],r_cpairs[i]))[0]
    in2 = inside_angle_area(c_cpairs[j_o], center, (c_cpairs[j],r_cpairs[i]))[0]
    in_result = [inside_angle_area(pt, center, (c_cpairs[j],r_cpairs[i]))[0] for pt in remaining_pts]
    in3 = False
    print ("check me ",i, j, in1, in2, in_result)
    in3 = sum(in_result) > 0 # at least 1 inside -> failed
    if in1  or in2 or  in3:
        #print ("debug__________[NOT are open sight]",c_cpairs[j], r_cpairs[i])
        return False
    else:
        #print ("debug__________[open sight]",c_cpairs[j], r_cpairs[i])
        return True
        
def open_sight(center, i, j, close_cpairs):
    # i is always greater than j
    r_cpairs = close_cpairs[i]
    c_cpairs = close_cpairs[j]
    
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

def get_a_open_point_from_a_pair(center, radius, pair):
    midpt = midpoint(pair[0], pair[1])
    pt_is = intersection(center[0], center[1], radius, [center, midpt])
    
    if inside_ls(midpt, [pt_is[0], center]):
        return pt_is[0]
    else:
        return pt_is[1]
           
def get_open_close_sight(plt, x, y, radius, goal, true_sight):
    center = [x,y]
    print ("Center", center)
    print ("true_sight", true_sight)

    ref_cpairs = [get_true_is_circle_pairs( (x, y), radius, true_pair) for true_pair in true_sight]
    
    if print_ref_sight:
        print ("ref circle pairs", ref_cpairs)

    close_cpairs = get_close_cpairs(ref_cpairs)
    if print_close_sight:
        print_cpairs("close circle pairs", close_cpairs)

        
    open_cpairs = get_open_cpairs(center, radius, goal, close_cpairs)
    if print_open_sight:
        print_cpairs("open circle pairs", open_cpairs)

    return open_cpairs, close_cpairs              