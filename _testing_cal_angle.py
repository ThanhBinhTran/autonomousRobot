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

center = (34.6, 43.5)
A = (10,0)
B = (23, 28)
ox_axis = A
v1 = np.subtract(B, center)
print ("signed angle")
print (cal_signed_angle(ox_axis, v1))

plt.plot([0,ox_axis[0]], [0, ox_axis[1]], 'b-')
plt.plot([center[0],B[0]], [center[1], B[1]], 'r.-.')
plt.plot([0,v1[0]], [0, v1[1]], 'b.-')


plt.grid(True)
plt.show()

