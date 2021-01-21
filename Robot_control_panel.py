import numpy as np
import sys, getopt

def print_help():
    print ('program -n <run times> -m <map name> -s <x_y> -g <x_y>')
    print ('Example: program -n 1 -m _map.csv -s 10.0_10.0 -g r')
    
def menu():

    runtimes = 1
    
    #mapname = "_river.csv" 
    mapname = "_map.csv"
    #mapname = "_MuchMoreFun.csv"
    start_point = [0, 0]

    #goal = np.array(np.random.randint(100, size=(1,2)))
    goal = np.array( [70, 70] )
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hn:s:g:m:", ["runtimes=","start=","goal=","mapname="])
    except getopt.GetoptError:
        print_help()
        sys.exit(2)
    #print (opts, args)
    for opt, arg in opts:
        if opt == '-h':
            print_help()
            sys.exit()
        elif opt in ("-n", "--runtimes"):
            try:
                runtimes = int(arg)
            except ValueError:
                runtimes = 1
                print('invalid run times, use default (1)')
                sys.exit(2)
        elif opt in ("-m", "--mapname"):
            try:
                mapname = arg
            except:
                print_help()
                sys.exit(2)
        elif opt in ("-s", "--start"):
            try:
                pt = arg.split('_')
                start_point = np.array( [float(pt[0]), float(pt[1])] )
            except ValueError:
                print('invalid start point')
                sys.exit(2)
        elif opt in ("-g", "--goal"):
            try:
                if arg.find("r") != -1:# random
                    goal = np.array(np.random.randint(100, size=(1,2)))[0]
                else:
                    pt = arg.split('_')
                    goal = np.array( [float(pt[0]), float(pt[1])] )
            except ValueError:
                print('invalid goal point')
                sys.exit(2)
    
    return runtimes, mapname, start_point, goal