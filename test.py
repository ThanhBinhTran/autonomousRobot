from matplotlib import pyplot
from shapely.geometry import Point
from shapely.ops import unary_union
from descartes import PolygonPatch



polygons = [Point(i, 0).buffer(0.7) for i in range(1)]

fig = pyplot.figure()

# 1
ax = fig.add_subplot(121)

for ob in polygons:
    p = PolygonPatch(ob, fc='g', ec='g', alpha=0.5, zorder=1)
    ax.add_patch(p)

ax.set_title('a) polygons')
#2
ax = fig.add_subplot(122)

u = unary_union(polygons)
patch2b = PolygonPatch(u, fc='b', ec='b', alpha=0.5, zorder=2)
ax.add_patch(patch2b)
print (polygons)


print ("\n\n\n\n")
print (u)
ax.set_title('b) union')

pyplot.show()
