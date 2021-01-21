import sys, getopt
def print_help():
    print ('program -n <run times> -m <map name>')
    print ('Example: program -n 1 -m _map.csv')
    
def menu():

    runtimes = 1
    
    #mapname = "_river.csv" 
    mapname = "_map.csv"
    #mapname = "_MuchMoreFun.csv"
    
    try:
        opts, args = getopt.getopt(sys.argv[1:],"hn:sx:sy:m:", ["runtimes=","start_x=","start_y=","mapname="])
    except getopt.GetoptError:
        print_help()
        sys.exit(2)

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
        elif opt in ("-sx", "--start_x"):
            try:
                print (arg)
                start_point[0] = float(arg)
            except ValueError:
                print('invalid start_point_x')
                sys.exit(2)
        elif opt in ("-sy", "--start_y"):
            try:
                print (arg)
                start_point[1] = float(arg)
            except ValueError:
                print('invalid start_point_y')
                sys.exit(2)
    
    return runtimes, mapname