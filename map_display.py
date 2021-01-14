import matplotlib.pyplot as plt
import sys
import getopt
from Robot_csv_lib import *
from Robot_map_lib import *

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

[ox,oy] = read_map_csv(mapname)
for i in range (len(ox)):
    print (ox[i], oy[i])

map_display(plt, mapname, ox, oy)
plt.grid(True)
plt.show()