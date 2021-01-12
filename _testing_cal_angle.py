from Robot_lib import *
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

vectors_t = np.array([
               [ 1, 0],
               [ 1, 1],
               [ 0, 1],
               [-1, 1],
               [-1, 0],
               [-1,-1],
               [ 0,-1],
               [ 1,-1],
               [ 1, 0]
               ])
va = [0,  1]
vb = [-1, -1]

print ("----unsigned-------------------------------------")
for i in range (len(vectors_t) -1):
    for j in range(i+1, len(vectors_t)):
        print (angle_between(vectors_t[i], vectors_t[j]))
        print (cal_angle(vectors_t[i], vectors_t[j]))
    print ("_")

print ("----signed-------------------------------------")
for i in range (len(vectors_t) -1):
    for j in range(i+1, len(vectors_t)):
        print (cal_signed_angle(vectors_t[i], vectors_t[j]), vectors_t[i], vectors_t[j])
        print (cal_signed_angle(vectors_t[j], vectors_t[i]), vectors_t[j], vectors_t[i])
    print ("_")    

#for point in vectors_t:
#    plt.plot([0,point[0]], [0, point[1]])
#plt.plot([0,va[0]], [0, va[1]],'r.-.')
#plt.plot([0,vb[0]], [0, vb[1]])
#plt.show()
#plt.grid(True)


vec = [1,0,0]
vec1 = [1,0]
A = [0,3]
B = [1,4]

rotation_degrees = -45
rotation_radians = np.radians(rotation_degrees)

print (rotate_around_point_lowperf(B, rotation_radians, A))