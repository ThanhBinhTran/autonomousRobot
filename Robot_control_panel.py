import sys, getopt
def print_help():
    print ('program -n <run_time> -s <start x y>')
    print ('Example: program -n 1 -s 10 23')
    
def menu():

    run_times = -1

    try:
        opts, args = getopt.getopt(sys.argv[1:],"hn:sx:sy:", ["run_time=","start_x=","start_y="])
    except getopt.GetoptError:
        print_help()
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print_help()
            sys.exit()
        elif opt in ("-n", "--run_time"):
            try:
                run_times = int(arg)
            except ValueError:
                run_times = 1
                print('invalid run times, use default (1)')
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
                
    return run_times