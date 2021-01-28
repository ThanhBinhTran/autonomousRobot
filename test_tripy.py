
import matplotlib.pyplot as plt
import numpy as np
import tripy
import csv

def read_map_csv(mapname):
    ''' read coordinate of a map from csv file '''
    first_line = True
    ob=[]
    
    with open(mapname, newline='') as f:
        reader = csv.reader(f, delimiter=',', quoting=csv.QUOTE_NONE)
        for row in reader:
            if first_line:
                first_line = False
                continue
            ob.append((int(row[0]),int(row[1])))
    return ob
    
def map_display(mapname, ob):
    # displaying the title 
    plt.title("Display map: {0}".format(mapname))
    x = [point[0] for point in ob]
    y = [point[1] for point in ob]
    plt.plot(x, y, '-1b')
    
def plot_line(line):
    plt.plot((line[0][0],line[1][0]), (line[0][1],line[1][1]), ":c")
    
def plot_triangles(triangles):
    for triangle in triangles:
        plot_line((triangle[0],triangle[1]))
        plot_line((triangle[1],triangle[2]))
        plot_line((triangle[2],triangle[0]))
        
def main():
    print(__file__ + " start!!")
    
    mapname = "_MuchMoreFun.csv"
    
    polygon = read_map_csv(mapname) 
    #polygon = np.array(polygon)
    
    print ("polygon", polygon)
    
    triangles = tripy.earclip(polygon)
    print ("triangles", triangles)

          
    # draw map obstacles 
    map_display( mapname, polygon)
    
    # display triangle
    plot_triangles(triangles)    
    
    plt.axis("equal") # make sure ox oy axises are same resolution open local points

    plt.show()

if __name__ == '__main__':
    main()
