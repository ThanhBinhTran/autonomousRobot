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
    
def is_inside_ls(point, ls_points): # line segment points
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
     
def rotate_origin_only( v, radians):
    """Only rotate a point around the origin (0, 0)."""
    x, y = v
    rx = x * math.cos(radians) + y * math.sin(radians)
    ry = -x * math.sin(radians) + y * math.cos(radians)
    return rx, ry
    
def unit_vector(vector):
    """ Returns the unit vector of the vector."""
    return vector / np.linalg.norm(vector)
def angle_between(v1, v2):
    """Finds angle between two vectors"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle =  np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    return angle * 180 / math.pi
    
def cal_angle(v1, v2):
    """Finds angle between two vectors"""
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.dot(v1_u, v2_u))
    return angle * 180 / math.pi
def cal_signed_angle_base(v1, v2):
    """
    Finds angle between two vectors
    return signed angle (+) for anti clockwise, and (-) for clockwise
    it works when base vector is (1,0)
    """
    angleB = math.atan2(v2[1],v2[0])
    angleA = math.atan2(v1[1],v1[0])
    return angleB - angleA
    
def cal_signed_angle(v1, v2):
    """
    Finds angle between two vectors
    return signed angle (+) for anti clockwise, and (-) for clockwise
    """
    angle = cal_signed_angle_base(v1, [1,0]) # cal angle between vector (A) and ox axis
    v_a = rotate_origin_only(v1, -angle)  # rotate vector A to ox axis
    v_b = rotate_origin_only(v2, -angle)  # rotate vector B according to rotation_radians       
    signed_angle = cal_signed_angle_base(v_a, v_b)
    return signed_angle  * 180 / math.pi
    
def is_inside_area(point, center, boundaries):
    """ check if a point is where inside boundaries area
        return 1 if inside
        return 0 if at the boundary line
        return 1 if outside
    """
    tiny_diff_angle = 0.00000008 # there is not accuracy computational computer on floating point   
                                 # then accept very tiny diff result
    vector_a = np.subtract(boundaries[0], center)
    vector_b = np.subtract(boundaries[1], center)
    vector_p = np.subtract(point, center)
    
    angle_sight = cal_signed_angle(vector_a, vector_b)
    angle_point = cal_signed_angle(vector_a, vector_p)
    print ("angle_sight", angle_sight)
    print ("angle_point", angle_point)
    abs_diff_angle = abs(angle_point - angle_sight)
    if abs(angle_point) <= tiny_diff_angle or abs_diff_angle < tiny_diff_angle:
        #print ("at boundaries_: point {0}, boundaries{1}".format(point, boundaries) )
        return 0
    elif angle_sight * angle_point < 0: # diff side
        #print ("diff side: point {0}, boundaries{1}".format(point, boundaries) )
        return -1
    else:
        # compare angles in unsigned
        abs_angle_sight = abs(angle_sight)
        abs_angle_point = abs(angle_point)
        if abs_angle_sight > abs_angle_point:
            #print ("inside: point {0}, boundaries{1}".format(point, boundaries) )
            return 1
        else:
            ##print ("outside: point {0}, boundaries{1}".format(point, boundaries) )
            return -1
            
    return -1 
    
def lineFromPoints(P, Q):
 
    a = Q[1] - P[1]
    b = P[0] - Q[0]
    c = a*(P[0]) + b*(P[1])
    # ax + by = c
    return [a, b, c]

def point_dist(p, q):
    """ calculate distance of 2 point """
    return math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)
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
    
def print_sight(message_ID, pairs):
    print (message_ID)
    for pair in pairs:
        print (pair[0],pair[1])

def print_point(message_ID, point_x, point_y):
    print (message_ID)
    for i in range(len(point_x)):
        print ("{0} {1}".format(point_x[i],point_y[i]))