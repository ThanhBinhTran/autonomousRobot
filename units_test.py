from Robot_lib import *
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from point import *
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

print ("----signed-------------------------------------")
for i in range (len(vectors_t)):
    vi = vectors_t[i]
    #signed_angle_point = signed_angle_xAxis(vi)
    #print (math.degrees(signed_angle_point),vi)
    for j in range(i+1, len(vectors_t)):
        vj = vectors_t[j]
        aij = signed_angle(vi, vj)
        aji = signed_angle(vj, vi)
        print (math.degrees(aij), vi, vj)
        print (math.degrees(aji), vj, vi)
    print ("_")    

center = (34.6, 43.5)
A = (10,0)
B = (23, 28)
ox_axis = A
v1 = np.subtract(B, center)
sa = math.degrees(signed_angle(ox_axis, v1))
print ("signed angle")
print (sa)

plt.plot([0,ox_axis[0]], [0, ox_axis[1]], 'b-')
plt.plot([center[0],B[0]], [center[1], B[1]], 'r.-.')
plt.plot([0,v1[0]], [0, v1[1]], 'b.-')


plt.grid(True)
plt.show()

