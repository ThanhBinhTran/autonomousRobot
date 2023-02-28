import ctypes
import glob
import numpy as np

# find the shared library, the path depends on the platform and Python version
libfile = glob.glob('build/*/cgal_intersection_tri*.so')[0]

# 1. open the shared library
mylib = ctypes.CDLL(libfile)

# 2. tell Python the argument and result types of function main
mylib.main.restype = ctypes.c_int
mylib.main.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64), np.ctypeslib.ndpointer(dtype=np.float64),
		       np.ctypeslib.ndpointer(dtype=np.float64), np.ctypeslib.ndpointer(dtype=np.float64)]

# 3. call function mysum
#pts_triA = np.array([0,0, 0,1, 1,0], np.double)
#pts_triB = np.array([0,0, 0,1, 1,1], np.double)

pts_triA = np.array([0,0, 2,0, 1,2], np.double)
pts_triB = np.array([1,-1, 2,1, 0,1], np.double)

pts_data = np.array([0,0, 0,0, 0,0, 0,0, 0,0, 0,0], np.double) # maximun of 6 points of intersection of 2 triangles
pt_centre = np.array([0,0], np.double) # centre of mass of intersection polygon if found.
result = mylib.main(pts_triA, pts_triB, pts_data, pt_centre)

#print('calling result: number of intersection points {0}'.format(result))
#print('2 triangles: {0}, {1}'.format (pts_triA, pts_triB))
#print('intersection points: {0}'.format (pts_data))

''' plotting the result'''
import matplotlib.pyplot as plt

plt.plot( (pts_triA[0],pts_triA[2],pts_triA[4],pts_triA[0]), (pts_triA[1],pts_triA[3],pts_triA[5],pts_triA[1]), color='black')
plt.plot( (pts_triB[0],pts_triB[2],pts_triB[4],pts_triB[0]), (pts_triB[1],pts_triB[3],pts_triB[5],pts_triB[1]), color='blue')
is_pts_x = []
is_pts_y = []
for i in range (result):
    is_pts_x.append(pts_data[2*i])
    is_pts_y.append(pts_data[2*i+1])
if len(is_pts_x)>0:
    plt.plot(is_pts_x, is_pts_y, marker='H', color='red', ls=':')
    plt.plot(pt_centre[0], pt_centre[1], marker='*', color='red')

plt.axis('equal')
plt.show()
