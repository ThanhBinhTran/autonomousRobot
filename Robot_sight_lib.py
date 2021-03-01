from Robot_lib import *
from Robot_draw_lib import *
from Program_config import *
import numpy as np

#import matplotlib.pyplot as plt
rel_tol = 0.0000001
    
def devide_sight_ABCD(center, A, B, C, D): #  line [A,C], [B, D], B inside AC but D outside
    # divide sight into 3 different parts [R_C0_R]
    AC_BO_point = line_intersection([A,C], [B, center])
    BD_CO_point = line_intersection([B,D], [C, center])
    mid_sight = []
    if point_dist(center, AC_BO_point) < point_dist(center, B): # A - C is closer center than B
        #print ("AC is closer", point_dist(center, AC_BO_point), "<>", point_dist(center, B))
        devided_sight = [[A,C], [BD_CO_point, D]]
    else:
        #print ("AC is closer", point_dist(center, AC_BO_point), "<>", point_dist(center, B))
        devided_sight = [[A,AC_BO_point], [B,D]]

    return devided_sight
        
def detect_blind_sight(center, ref_sight, check_sight):
    ''' 
    check sight  --> C0, C1
    reference sight -->  R0 R1 
    '''
    r = ref_sight
    c = check_sight
    t_sight = []
    
    r_blind = False   # true if reference sight is blind sight
    c_blind = False   # true if check_sight is blind sight
    d_sight = False   # true if there need to divide (ref, check) area in to 2 parts
                      #                (ref_sub_part, common_ref/check_part, check_sub_path)
                      #             then remove ref, check sight , and create 2 new sight 
    divided_sight = []
    
    # get inside status of c1, c0
    c0_in, c0_code = inside_angle_area(c[0], center, [r[0], r[1]])
    c1_in, c1_code = inside_angle_area(c[1], center, [r[0], r[1]])
    #print ("_^^_ inside status:", c0_in, c0_code, c1_in, c1_code)
    ''' 
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
    '''
    if c0_in and c1_in: 
    # check sight completely inside reference sight
        if c0_code <=1 and c1_code == 2: # mutual c0 while c1 inside
            # check if c1 is inside (r0, center r1)
            c1_in  = inside_angle_area(c[1], r[0] , [center, r[1]])[0]
            if c1_in:
                is_pt = line_intersection([center, c[1]],[r[0], r[1]])
                if c0_code == 0:
                    divided_sight = [ [is_pt, r[1]], [c[0], c[1]] ]
                else:
                    divided_sight = [ [r[0], is_pt], [c[0], c[1]] ]
                d_sight = True
            else:
                c_blind = True
        elif c0_code == 2 and c1_code <=1: # mutual c1 while c0 inside
            # check if c1 is inside (r0, center r1)
            c0_in  = inside_angle_area(c[0], r[0] , [center, r[1]])[0]
            if c0_in:
                is_pt = line_intersection([center, c[0]],[r[0], r[1]])
                if c1_code == 0:
                    divided_sight = [[r[0], is_pt], [c[0], c[1]] ]
                else:
                    divided_sight = [[is_pt, r[1]], [c[0], c[1]] ]
                d_sight = True
            else:
                c_blind = True
        elif (c0_code == 0 and c1_code == 1) or (c0_code == 1 and c1_code == 0):
            # exactly same 
            c_in0, in0_code  = inside_angle_area(c[0], r[0] , [center, r[1]])
            c_in1, in1_code  = inside_angle_area(c[1], r[0] , [center, r[1]])
            if c_in0 and c_in1: # c0 c1 is closer
                r_blind = True
            else:
                c_blind = True
        else:
            # if check line is closer to center than reference line
            #  then dividing reference into 3 parts Rs-C0, C0-C1, C1-Re
            # else true sight is reference sight
            cin, cin_code = inside_angle_area(c[0], r[0] , [center, r[1]])
            if cin:
                d_sight = True
                is_c0R = line_intersection([center, c[0]],[r[0], r[1]])
                is_c1R = line_intersection([center, c[1]],[r[0], r[1]])
                if inside_ls(is_c0R, [r[0], is_c1R]): # R0 isC0R isC1R R1
                    divided_sight = [[r[0], is_c0R], [c[0], c[1]], [is_c1R,r[1]] ]
                else: # R0 isC1R isC0R R1
                    divided_sight = [[r[0], is_c1R], [c[0], c[1]], [is_c0R,r[1]] ]
            else:
                c_blind = True
            
    elif (c0_in and c0_code <2) or (c1_in and c1_code<2): # ether C0 or C1 is at the boundary segment
        '''
        check if check sight fully coverages ref_sight
        '''
        r0_in, r0_code = inside_angle_area(r[0], center, [c[0], c[1]])
        r1_in, r1_code = inside_angle_area(r[1], center, [c[0], c[1]])
        r0_inccc, r_code = inside_angle_area(r[0], c[0], [center, c[1]])
        r1_inccc, r_code = inside_angle_area(r[1], c[0], [center, c[1]])
        #print ("_^^_ inside status[R]:", r0_in, r0_code, r1_in, r1_code, r0_inccc, r1_inccc)
        if r0_in and r1_in: # ref sight is inside check sight
            # check if r is closer than check sight
            if r0_inccc and r1_inccc: # reference sight is closer
                isR0C = line_intersection([center, r[0]],[c[0], c[1]])
                isR1C = line_intersection([center, r[1]],[c[0], c[1]])
                d_sight = True
                if   not c0_in and c1_code == 0: # c0 R1 R0=C1
                # [Reference sight] - [C0: isR1C] in this order
                    divided_sight = [[r[0], r[1]], [c[0], isR1C]]
                elif not c0_in and c1_code == 1: # c0 R0 R1=C1
                # [Reference sight] - [C0: isR0C] in this order
                    divided_sight = [[r[0], r[1]], [c[0], isR0C]]
                elif not c1_in and c0_code == 0: # c1 R1 R0=C0
                # [Reference sight] - [C1: isR1C] in this order
                    divided_sight = [[r[0], r[1]], [c[1], isR1C]]
                elif not c1_in and c0_code == 1: # c1 R0 R1=C1
                # [Reference sight] - [C1: isR0C] in this order
                    divided_sight = [[r[0], r[1]], [c[1], isR0C]]
            else:
                r_blind = True
    elif c0_in and not c1_in :
        ''' 
        check if R0 is inside area of [R1, C1]
        if R0 is outside then R1 is inside area of [R0, C1] for sure
        '''
        d_sight = True
    
        r0_in, r0_code = inside_angle_area(r[0], center, [r[1], c[1]])
        if r0_in :  # R1 C0 R0 C1 
            divided_sight = devide_sight_ABCD(center, r[1], c[0], r[0], c[1])
        else: # R0 C0 R1 C1 
            divided_sight = devide_sight_ABCD(center, r[0], c[0], r[1], c[1])

    elif not c0_in  and c1_in: 
        ''' check if R0 is inside area of [R1, C0]
            if R0 is outside then R1 is inside area of [R0, C0] for sure
        '''
        d_sight = True
       
        r0_in, r0_code = inside_angle_area(r[0], center, [r[1], c[0]])
        if r0_in :  # R1 C1 R0 C0
            divided_sight = devide_sight_ABCD(center, r[1], c[1], r[0], c[0])
        else: # R0 C1 R1 C0
            divided_sight = devide_sight_ABCD(center, r[0], c[1], r[1], c[0])
       
    else:   # both Check C0 C1 are outside
        # Check if ref_sight is inside Check_sight,
        r0_in, r0_code = inside_angle_area(r[0], center, [c[1], c[0]])
        if r0_in: # C R C
            r_blind = True

    return divided_sight, r_blind, c_blind, d_sight
    
def remove_blind_pairs( center, b_pairs):
    t_pairs = b_pairs
    
    i = 0 # start with index = 0
    while i < len(t_pairs) -1:
        j = i + 1
        while j < len(t_pairs) :
        
            #print ("_+_Checking {0}{1} ".format(t_pairs[i][0],t_pairs[i][1]), "--> {0}{1}".format(t_pairs[j][0], t_pairs[j][1]) )
            ds, r_blind, c_blind, d_sight = detect_blind_sight(center, t_pairs[i], t_pairs[j])
            #print_pairs(" [Local] divide sight", ds)
            #print ("status of sight: r {0}, c {1}, d {2}".format(r_blind, c_blind, d_sight))
            if d_sight:  # separate into 2 new sight
                #print ("#%@ ", t_pairs[i], ds[0])
                t_pairs[i] = np.array(ds[0])
                t_pairs[j] = np.array(ds[1])
                
                #if len(ds) == 3:
                    #print ("___(#^ len = 3", ds)
                    #t_pairs.append(ds[2])
                #i = i - 1
                #break
            elif c_blind: # remove check sight cause it's blind sight
                #t_pairs.pop(j)
                t_pairs= np.delete(t_pairs, j, axis=0)
                j = j - 1
            elif r_blind: # replace ref sight by check sight cause it's blind sight
                # update i so need to recheck
                t_pairs[i] = t_pairs[j]
                t_pairs= np.delete(t_pairs, j, axis=0)
                i = i-1
                break
 
            j += 1
        i += 1 
        #print_pairs ("true_sight loop[{0}]".format(i), boundary_pts)
    return t_pairs
    
def true_pairs(center, b_pairs):
    '''
    get true pairs from boundary pairs by remove blind pairs
    '''
    t_pairs = []
    
    if len( b_pairs ) >=2:
        t_pairs = remove_blind_pairs(center, b_pairs)
    else: # true sight is boundary_pt if there is only 1 pair
        t_pairs = b_pairs
    return t_pairs
    
def get_all_boundary_pairs(center, robot_vision, ob):
    ''' 
    find all boundary pairs among all obstacle line segments and circle 
    '''
    x,y = center
    b_pairs = []
    for i in range(len(ob)-1):
        
        ptA = ob[i]  
        ptB = ob[i+1]

        is_points = intersection(x, y, robot_vision, [ptA, ptB])
        
        #print ("____+$: intersection point: {0} of {1}{2} ".format(is_points,ptA, ptB) )
        
        if len(is_points) > 0:
            ''' find boundary pair between a single line segment and circle '''
            b_pts = [] # boundary points
            
            for point in is_points:
                pt_in =  inside_ls(point, [ptA, ptB])
                #print ("inside_ls status *%# ", pt_in, " of point ", point)
                if pt_in:  # found intersection point is inside the line segment
                    b_pts.append(point)
                else:  # intersection point is not inside the line segment
                    ptA_in = inside_ls(ptA, is_points)
                    
                    if ptA_in: # found intersection point is inside the line segment
                        b_pts.append(ptA)
                    ptB_in = inside_ls(ptB, is_points)
                    if ptB_in : # found intersection point is inside the line segment
                        b_pts.append(ptB)
                        
            if len (b_pts) > 0:
                
                pd = point_dist(b_pts[0], b_pts[1])
                if not math.isclose(pd, 0): # make sure p1 != p2
                    b_pairs.append( [b_pts[0],b_pts[1]])
    return b_pairs
    
def get_true_pairs( center, robot_vision, ob):
    # get boundary pairs, [start point x,  end point x], x start < x end
    b_pairs = get_all_boundary_pairs( center, robot_vision, ob)

    if print_boundary_pairs:
        print_pairs ("boundary_pairs", b_pairs)
    
    
    t_pairs = true_pairs(center, b_pairs)
    if print_true_pairs:
        print_pairs ("true_pairs", t_pairs)
        
    return t_pairs
    
def inside_global_true_sight(pt, radius, traversal_path):
    result = [inside_local_true_sight(pt, x, radius, tsight) for x,tsight, _ in traversal_path]
    ret_result = np.sum(result) > 0
    #print ("inside global sight result: ", result, ", return :", ret_result)
    return ret_result
    
def inside_local_true_sight(pt, center, radius, t_sight):
    outside = True
    if point_dist(pt, center) < radius: # inside vision area
        outside = False
        # check if pt is inside true sight angle
        inside_open_sight = True
        visible = False
        for ts_pair in t_sight:
            pt_in = inside_angle_area(pt, center, ts_pair)[0]
            if pt_in :  # inside angle of true sight
                inside_open_sight = False
                pt_in = inside_angle_area( pt, ts_pair[0], (center, ts_pair[1]))[0]
                if pt_in : 
                    visible = True
                    #print ("found close ", pt, t_sight)
                    return True
                else:
                    #print ("found in blind sight ", pt, t_sight)
                    return False
        #print ("found open", pt, t_sight)
        return True
    else:
        #print ("Outside", pt)
        return False
    #return not outside and (inside_open_sight or visible)

def get_true_is_circle_pairs( center, radius, true_pair):
    ''' get true intersection between true sight and circle -> circle pairs '''
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

def get_close_cpairs(center, ref_cpairs):
    ret_result = []
    #clone new reference circle pairs
    rPairs = [] 
    rPairs.extend(ref_cpairs)
    if len(rPairs) == 1:
        ret_result = [[rPairs[0][0], rPairs[0][1], True]]
    else:
        i = 0
        while i < len(rPairs) -1:
            r_cpairs = rPairs[i] # reference pair
            #print ("__________", r_cpairs)
            j = i + 1
            while j < len(rPairs) :
                c_cpairs = rPairs[j]
                #print ("__________", c_cpairs)
                result = mutual_point(r_cpairs, c_cpairs) 
                #print (result)
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
                    rPairs[i] = new_cpairs
                    rPairs.pop(j)
                    #rPairs[j] = []
                    i = i - 1
                    break
                j = j + 1
            i = i + 1

        
        # once there is only one closed cpair
        # check if its whether true or fake
        if len(rPairs) == 1:
            ios = is_open_sight(center, rPairs[0], ref_cpairs)
            if ios: # closed cpair is true close
                ret_result = [[rPairs[0][0], rPairs[0][1], False]]
            else: # close cpair is complement area
                ret_result = [[rPairs[0][0], rPairs[0][1], True]]
        else:
            ret_result = rPairs
    return ret_result
    
def divide_open_cpair(center, inangle, vs, ve):
    return_pairs = []
    #print ("divide_open_cpair: ", math.degrees(inangle))
    angle = abs(inangle)
    if angle >= math.pi*4/3:      # 240 degree
    #divide into 3 parts
        r_angle = angle/ 3
        #print ("divide into 3 parts, with angle = ", math.degrees(r_angle))
        v1 = rotate_vector_center(center, vs, -r_angle)
        v2 = rotate_vector_center(center, vs, -2*r_angle)
        return_pairs.append([vs, v1])
        return_pairs.append([v1, v2])
        return_pairs.append([v2, ve])
    elif angle >= math.pi*2/3:      # 120 degree:
    #divide into 2 parts
        r_angle = angle/ 2
        #print ("divide into 2 parts, with angle = ", math.degrees(r_angle))
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
    
    if len(c_cpairs) == 0: # no obstacle detected
        #print ("No obstacle detected")
        #check if goal is at center
        if not math.isclose(point_dist(goal, center), 0):
            vector_cg_unit = unit_vector( np.subtract(goal, center))
        else:
            vector_cg_unit = [1,0]
            
        vs = np.multiply (vector_cg_unit, radius) + center
        vs = rotate_vector_center(center, vs, math.pi/3)

        pairs_extend = divide_open_cpair_complement (center, [vs, vs])
        for pair in pairs_extend:
            o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
                  
    elif len(c_cpairs) == 1: # only a obstacle detected
        #print ("only 1 obstacle detected")
        #print ("close cpairs", c_cpairs)
        if c_cpairs[0][2]: # TRUE: True close detected then open_sight is its complement
            pairs_extend = divide_open_cpair_complement (center, c_cpairs[0])
        else: # FALSE, this FAKE close sight is open_sight False
            angle, vs, ve = get_angle_info(center, c_cpairs[0][0], c_cpairs[0][1])
            pairs_extend = divide_open_cpair(center, angle, vs, ve)
            
        for pair in pairs_extend:
            o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))
            #o_cpairs.append(init_open_cpair(center, radius, c_cpairs[0][0], c_cpairs[0][1]))
            
    else:
        #print ("more than 2 obstacles detected")
        i = 0
        while i < len(c_cpairs) -1:
            j = i + 1
            while j < len(c_cpairs) :
                found, ridx, cidx = open_sight(center, i, j, c_cpairs) 
                #print ("_Pair r {0} c {1}".format(c_cpairs[i], c_cpairs[i]) )
                #print ("__Open result {0}, index {1} {2}\n".format(found, ridx, cidx) )
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
            ios = is_open_sight(center, [ptA, ptB], close_cpairs)
            
            if ios: # true open pair
                angle, vs, ve = get_angle_info(center, ptA, ptB)
                pairs_extend = divide_open_cpair(center, angle, vs, ve)
            else:
                pairs_extend = divide_open_cpair_complement (center, c_cpairs[0])

            for pair in pairs_extend:
                o_cpairs.append(init_open_cpair(center, radius, pair[0], pair[1]))

    return o_cpairs
    
def get_allpts_except(all_pairs, e_pairs):
    '''
    get all points, which are from pairs, but except points from except pair
    '''
    allPts = []
    for pair in all_pairs:
        allPts.append(pair[0])
        allPts.append(pair[1])
    allPts.remove(e_pairs[0])
    allPts.remove(e_pairs[1])
    return allPts


            
def is_open_sight(center, cpair, all_pairs):
    #print ("___Pair {0}, all points {1}".format(cpair, all_pairs))
    #print ("___Len checking point", len (all_pairs))
    all_pts = []
    for pair in all_pairs:
        all_pts.append(pair[0])
        all_pts.append(pair[1])
    #for point in all_pts:
    #    print ("___ points", point )
    in_status = [inside_closed_angle_area(pt, center, cpair) for pt in all_pts]
    
    ret_result = sum(in_status) == 0 # sum true == 0, all must outside
    
    #print ("____inside status: ", in_status, " -> result: ", ret_result)
    return ret_result
    
def is_open_sight_pairs(center, pt_i, pt_j, i, j, all_pairs):
    # i for ref, j for check
    r_cpairs = all_pairs[i]
    c_cpairs = all_pairs[j]
    ret_result = is_open_sight(center, [r_cpairs[pt_i],c_cpairs[pt_j]], all_pairs)

    #print ("_Check pair (r, c): {0} {1}, index i: {2} j: {3}, result: {4}".format(r_cpairs[pt_i],c_cpairs[pt_j], pt_i, pt_j,  ret_result) )

    return ret_result
    
        
def open_sight(center, i, j, close_cpairs):
    # i is always greater than j
    
    found = True
    ridx, cidx = (0,0)
    # i first for ref, j second for check
    if   is_open_sight_pairs(center, 0, 1, i, j, close_cpairs): ridx, cidx = (0,1)
    elif is_open_sight_pairs(center, 1, 0, i, j, close_cpairs): ridx, cidx = (1,0)
    elif is_open_sight_pairs(center, 1, 1, i, j, close_cpairs): ridx, cidx = (1,1)
    elif is_open_sight_pairs(center, 0, 0, i, j, close_cpairs): ridx, cidx = (0,0)
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
           
def get_open_close_sight(x, y, radius, goal, t_sight):
    center = [x,y]

    ref_cpairs = [get_true_is_circle_pairs( (x, y), radius, true_pair) for true_pair in t_sight]
    
    if print_ref_sight:
        print ("\n_ref circle pairs", ref_cpairs)
    
    close_cpairs = get_close_cpairs((x,y), ref_cpairs)
    if print_close_sight:
        print_cpairs("\n_close circle pairs", close_cpairs)
        
    open_cpairs = get_open_cpairs(center, radius, goal, close_cpairs)
    if print_open_sight:
        print_cpairs("\n_open circle pairs", open_cpairs)

    return open_cpairs, close_cpairs
    
def scan_around(center, robot_vision, ob, goal):
    '''
    this function is to scan obstacle 
    return true sight, closed sight, and open sight
    '''
    t_sight = get_true_pairs(center, robot_vision, ob)
    osight, csight = get_open_close_sight(center[0], center[1], robot_vision, goal, t_sight)
    
    return t_sight, osight, csight
    
def get_explorered_sight(center, goal, robotvision, tpairs, osight):
    '''     
    extend map from local true sights
    '''
    map = []
    temp_tpairs = np.array(tpairs)
    temp_osight = np.array(osight)
    print ("temp_tpairs", temp_tpairs)
    print ("temp_osight", temp_osight)

    angle_tpairs = [math.degrees(unsigned_angle_xAxis(point)) for point in temp_tpairs[:,0]]
    angle_osight = [math.degrees(unsigned_angle_xAxis(point)) for point in temp_osight[:,0]]
    
    print ("angle_tpairs", angle_tpairs)
    print ("angle_osight", angle_osight)
    
    angle_tpairs_idx_sort = np.argsort(angle_tpairs)
    angle_osight_idx_sort = np.argsort(angle_osight)
    print ("angle_tpairs_idx_sort", angle_tpairs_idx_sort)
    print ("angle_osight_idx_sort", angle_osight_idx_sort)
    i = 0
    j = 0
    lasti = False
    lastj = False
    idx_i = 0
    idx_j = 0
    while i < len(angle_tpairs):
        while j < len(angle_osight):
            pre_i = idx_i
            pre_j = idx_j
            idx_i = angle_tpairs_idx_sort[i]
            idx_j = angle_osight_idx_sort[j]
            print ("idx_i idx_j", idx_i, idx_j, temp_tpairs[idx_i], osight[idx_j][0:2])
            if angle_tpairs[idx_i] < angle_osight[idx_j]:
                if lastj:
                    map.append([osight[pre_j][1],temp_tpairs[idx_i][0]])
                map.append(temp_tpairs[idx_i])

                lasti = True
                lastj = False
                break
            else:
                if lasti:
                    map.append([temp_tpairs[idx_i][1], osight[pre_j][1]])                
                map.append([osight[idx_j][0],osight[idx_j][2]])
                map.append([osight[idx_j][2],osight[idx_j][1]])
                lasti = True
                lastj = False
            j += 1
        i += 1
    if i != len(angle_tpairs):  # i remain
        while i < len(angle_tpairs):
            idx_i = angle_tpairs_idx_sort[i]
            map.append(temp_tpairs[idx_i])
        i += 1
    else:
        while j < len(angle_osight):
            idx_j = angle_osight_idx_sort[j]
            map.append([osight[idx_j][0],osight[idx_j][2]])
            map.append([osight[idx_j][2],osight[idx_j][1]])
            j += 1
    map = np.array(map)
    #print ("__________map", map)
    
    return map
    '''
    extend map from local true sights
    '''
    if len(emap) > 0:
        new_map = np.array(ltsight)
        if len(new_map) > 0:
            emap = np.concatenate((emap, ltsight), axis=0)
            emap = np.unique(emap, axis=0)
        #print ("new map", new_map)
        #is_belongline = [belong_line(new_map, eline) for eline in emap]
        #print ("is_belongline", is_belongline)
    else:
        emap = np.array(ltsight)
    return emap
