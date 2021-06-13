import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
import getopt
from Robot_csv_lib import *
from Robot_map_lib import *
from Robot_world_lib import *

mapname = "_map.csv"

# python program -m <mapname>
try:
    opts, args = getopt.getopt(sys.argv[1:],"hm:", ["map="])
except getopt.GetoptError:
    print ('python find -m <mapname>')
    sys.exit(2)

patternStr = ""
for opt, arg in opts:
    if opt == '-h':
        print ('python find -m <mapname>')
        sys.exit()
    elif opt in ("-m", "--map"):
            mapname = arg
    else:
        print ('python find -m <mapname>')
        sys.exit()

points = []
first_line = True

print ("____________display:{0}_________".format(mapname))

read_map_from_world(mapname)
ob = read_map_csv(mapname + ".csv")
# draw map obstacles 
map_display(plt, mapname, ob)
# draw image of world
world_display(plt, mpimg, mapname)

plt.axis("equal")
plt.grid(True)
plt.show()