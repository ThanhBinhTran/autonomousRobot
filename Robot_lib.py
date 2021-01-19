import numpy as np
import math

def intersection(x, y, radius, ls_points): # line segment points
    """ find the two points where a secant intersects a circle """
    p_is = []
    p1x,p1y = ls_points[0]
    p2x,p2y = ls_points[1]
    dx, dy = np.subtract(p2x,p1x), np.subtract(p2y, p1y)
    a = dx**2 + dy**2
    b = 2 * (dx * (p1x - x) + dy * (p1y - y))
    c = (p1x - x)**2 + (p1y - y)**2 - radius**2
    discriminant = b**2 - 4 * a * c
    if discriminant > 0:
        t1 = (-b + discriminant**0.5) / (2 * a)
        t2 = (-b - discriminant**0.5) / (2 * a)
        p_is.append([dx * t1 + p1x, dy * t1 + p1y])
        p_is.append([dx * t2 + p1x, dy * t2 + p1y])
    return p_is
    
def inside_ls(point, ls_points): # line segment points
    """
    check if a point is whether inside give line segment 
    return a point if it is inside, otherwise return None 
    """
    d1 = np.subtract(point, ls_points[0])
    d2 = np.subtract(point, ls_points[1])
    position = np.dot (d1, d2)
    ##print (position, point, ls_points)
    if position <= 0:
        return point
    return None

def rotate_vector_center(center, v, radians):
    vector_vc = np.subtract(v, center)
    r_vector_vc = rotate_vector(vector_vc, radians)
    result =  np.add(center, r_vector_vc)
    return np.add(center, r_vector_vc)
    
def rotate_vector( v, radians):
    """Only rotate a point around the origin (0, 0)."""
    x, y = v
    rx = x * math.cos(radians) + y * math.sin(radians)
    ry = -x * math.sin(radians) + y * math.cos(radians)
    return rx, ry
    
def unit_vector(vector):
    """ Returns the unit vector of the vector."""
    return vector / np.linalg.norm(vector)
    
def unsigned_angle(v1, v2):
    """Finds angle between two vectors"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.dot(v1_u, v2_u))

    #return math.degrees(angle)
    return angle
    
def signed_angle_xAxis(point):
    """
    Finds angle between two vectors
    return signed angle (+) for anti clockwise, and (-) for clockwise
    it works when base vector is (1,0)
    """
    
    angle = math.atan2(point[1], point[0])
    #print (math.degrees(angle))
    return angle

    
def signed_angle(v1, v2):
    """
    Finds angle between two vectors
    return signed angle (+) for anti clockwise, and (-) for clockwise
    """
    angle = signed_angle_xAxis(v1)  # cal angle between vector (A) and ox axis
    v_b = rotate_vector(v2, angle)  # rotate vector B according to rotation_radians 
    return signed_angle_xAxis(v_b)  
    
def get_angle_info(center, ptA, ptB):
    """ return angle, start edge , end edge of given angle in anti-clockwise """
    
    vectorA = np.subtract(ptA, center)
    vectorB = np.subtract(ptB, center)
    
    angle = signed_angle(vectorA, vectorB)
   
    if angle < 0:
        vs = ptB # start 
        ve = ptA # end
    else:
        vs = ptA # start
        ve = ptB # end
    return angle, vs, ve
    
def inside_angle_area(check_pt, center, ref_bd): # is check point inside reference boundaries
    """ check if a check_pt is where inside (ref[start]- center - ref[end]) area
        return True if inside
                    additional code = 0, on the second of edge ref_bd
                    additional code = 1, on the second of edge ref_bd
                    additional code = 2, in among of 2 the edges of ref_bd
        return Flase if outside
        using math.isclose to avoid the error of floating check_pt computation
    """
    #print ("inside_angle_area check {0}, center{1}, ref{2}".format( check_pt, center, ref_bd))
    vector_a = np.subtract(ref_bd[0], center)
    vector_b = np.subtract(ref_bd[1], center)
    vector_p = np.subtract(check_pt, center)
    #print ("___* input:" , check_pt, center, ref_bd)
    #print ("___* vector abp:" , vector_a, vector_b, vector_p)
    angle_sight = signed_angle(vector_a, vector_b)
    angle_check_pt = signed_angle(vector_a, vector_p)
    diff_angle = abs(angle_sight - angle_check_pt)
    rel_tol = 0.0000001
    ret_result = False
    ret_code = 0
    if abs(angle_check_pt) < rel_tol:
        #print ("____* same edge 1")
        ret_result = True
        ret_code = 0
    elif diff_angle < rel_tol:
        #print ("____* same edge 2")
        ret_result = True
        ret_code = 1

    elif angle_sight * angle_check_pt < 0: # diff side
        #print ("____* diff side")
        ret_result = False
    else:
        # compare angles in unsigned
        abs_angle_sight = abs(angle_sight)
        abs_angle_check_pt = abs(angle_check_pt)
        if abs_angle_sight > abs_angle_check_pt:
            #print ("____* inside")
            ret_result = True
            ret_code = 2
        else:
            #print ("____* outside")
            ret_result = False
    #print ("_____*", math.degrees(angle_sight), math.degrees(angle_check_pt), ret_result, ret_code)
    return ret_result, ret_code
    
def inside_closed_angle_area(check_pt, center, ref_bd): # is check point inside reference boundaries
    """ check if a check_pt is where inside closed angle of (ref[start]- center - ref[end]) area
        return True if inside (not boundary)
        return Flase if outside

    """
    in_status, in_code = inside_angle_area(check_pt, center, ref_bd)
    #print ("___+ inside_closed_angle_area:", in_status, in_code, "Check point", check_pt )
    return (in_status and in_code == 2 )
    
def midpoint(P, Q):
    """ return mid point of Q,P """
    x = (P[0] + Q[0])/2
    y = (P[1] + Q[1])/2
    return [x, y]
    
def lineFromPoints(P, Q):
 
    a = Q[1] - P[1]
    b = P[0] - Q[0]
    c = a*(P[0]) + b*(P[1])
    # ax + by = c
    return [a, b, c]

def point_dist(p, q):
    """ calculate distance of 2 point """
    return math.hypot(q[0] - p[0], q[1] - p[1])
    #return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)
    #return math.dist(p, q)
    
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])
    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]
    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return [x, y]
    
def print_pairs(message_ID, pairs):
    print ("{0}, len: {1}".format(message_ID, len(pairs)))
    for pair in pairs:
        print (pair[0],pair[1])

def print_point(message_ID, point_x, point_y):
    print (message_ID)
    for i in range(len(point_x)):
        print ("{0} {1}".format(point_x[i],point_y[i]))
                
def print_cpairs(message_ID, cpairs): # print circle pairs
    print ("{0}, len: {1}".format(message_ID, len(cpairs)))
    for pairs in cpairs:
        print ("pair ", pairs)