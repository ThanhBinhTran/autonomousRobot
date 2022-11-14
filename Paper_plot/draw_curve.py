import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
verts = [
   (55, 40),   # P0
   (55, 50),  # P1
   (55, 55),  # P2
   (60, 55),  # P3
]
roads =np.array([
    [[40, 30], [40, 40], [30, 40]],
    [[60, 30], [60, 40], [70, 40]],
    [[40, 70], [40, 60], [30, 60]],
    [[60, 70], [60, 60], [70, 60]]
])

mid_lines =np.array([
    [[50, 30], [50, 40]],
    [[30, 50], [40, 50]],
    [[60, 50], [70, 50]],
    [[50, 60], [50, 70]]
]) 

codes = [
    Path.MOVETO,
    Path.CURVE4,
    Path.CURVE4,
    Path.CURVE4,
]
print (roads[0][:,0])
path = Path(verts, codes)
print (path)
fig, ax = plt.subplots()
patch = patches.PathPatch(path, facecolor='none', lw=2)
print (patch.get_data_transform())
ax.add_patch(patch)

xs, ys = zip(*verts)
ax.plot(xs, ys, '.:k')

# display roads
for i in range(4):
    plt.plot(roads[i][:,0], roads[i][:,1], '-b')
# display mid line
for i in range(4):
    plt.plot(mid_lines[i][:,0], mid_lines[i][:,1], '--b')
    
# display control point
for i in range(4):
    ax.text(verts[i][0], verts[i][1], "P{0}".format(i))

plt.axis("equal")
plt.grid(True)
plt.show()