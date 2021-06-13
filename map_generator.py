import matplotlib.pyplot as plt
import sys
import getopt
from Robot_map_lib import *
from Robot_csv_lib import *

N = 16 #number of points
mapname = "_map.csv"


help_mgs = 'python map_generator -n <number_of_points> -m <mapname>'
try:
    opts, args = getopt.getopt(sys.argv[1:],"hn:m:", ["number=","mapname="])
except getopt.GetoptError:
    print (help_mgs)
    sys.exit(2)

patternStr = ""
for opt, arg in opts:
    if opt == '-h':
        print (help_mgs)
        sys.exit()
    elif opt in ("-n", "--number"):
            try:
                N = int(arg)
            except:
                print ("DEBUG ", opt, arg, patternStr)
                sys.exit(2)
    elif opt in ("-m", "--mapname"):
            try:
                mapname = arg
            except:
                print ("DEBUG ", opt, arg, patternStr)
                sys.exit(2)
                
    else:
        print (help_mgs)
        sys.exit()

# input number of points of map, and click to generate map, middle mouse click to stop
mappoints = map_generator(plt, N)

x = [int(i[0]) for i in mappoints]
y = [int(i[1]) for i in mappoints]
for i in range (len(x)):
    print (x[i],y[i])

plt.plot(x, y, "-y")
plt.grid(True)
plt.show()

# save map to file
print ("save map points to file: {0}".format(mapname))
write_map_csv(mapname, mappoints, ["x","y"])