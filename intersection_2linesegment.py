# A Python3 program to find if 2 given line segments intersect or not 
import matplotlib.pyplot as plt

class Point: 
	def __init__(self, x, y): 
		self.x = x 
		self.y = y 

# Given three colinear points p, q, r, the function checks if 
# point q lies on line segment 'pr' 
def onSegment(p, q, r): 
	if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and
		(q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
		return True
	return False

def orientation(p, q, r): 
	# to find the orientation of an ordered triplet (p,q,r) 
	# function returns the following values: 
	# 0 : Colinear points 
	# 1 : Clockwise points 
	# 2 : Counterclockwise 
	
	# See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/ 
	# for details of below formula. 
	
	val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
	if (val > 0): 
		
		# Clockwise orientation 
		return 1
	elif (val < 0): 
		
		# Counterclockwise orientation 
		return 2
	else: 
		
		# Colinear orientation 
		return 0

# The main function that returns true if 
# the line segment 'p1q1' and 'p2q2' intersect. 
def doIntersect(p1,q1,p2,q2): 
	
	# Find the 4 orientations required for 
	# the general and special cases 
	o1 = orientation(p1, q1, p2) 
	o2 = orientation(p1, q1, q2) 
	o3 = orientation(p2, q2, p1) 
	o4 = orientation(p2, q2, q1) 

	# General case 
	if ((o1 != o2) and (o3 != o4)): 
		return True

	# Special Cases 

	# p1 , q1 and p2 are colinear and p2 lies on segment p1q1 
	if ((o1 == 0) and onSegment(p1, p2, q1)): 
		return True

	# p1 , q1 and q2 are colinear and q2 lies on segment p1q1 
	if ((o2 == 0) and onSegment(p1, q2, q1)): 
		return True

	# p2 , q2 and p1 are colinear and p1 lies on segment p2q2 
	if ((o3 == 0) and onSegment(p2, p1, q2)): 
		return True

	# p2 , q2 and q1 are colinear and q1 lies on segment p2q2 
	if ((o4 == 0) and onSegment(p2, q1, q2)): 
		return True

	# If none of the cases 
	return False

# Driver program to test above functions: 
A = (10, 0) 
B = (0, 10) 
C = (0, 0) 
D = (5,10) 


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
    return (x, y)

result = line_intersection((A, B), (C, D))
print (result)
plt.plot((A[0], B[0]),(A[1],B[1]))
plt.plot((C[0], D[0]),(C[1],D[1]))
plt.plot(result[0],result[1], ".r")
plt.grid(True)
plt.show()