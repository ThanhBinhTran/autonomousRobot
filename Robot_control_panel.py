import argparse

'''
get user inputs
'''

def menu():
    
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', type=int, help='Number of runtimes', default=1)
    parser.add_argument('-m', help='map data', default='_map.csv')
    parser.add_argument('-w', help='world model')
    parser.add_argument('-sx', type=float, help='start point x', default=0.0)
    parser.add_argument('-sy', type=float, help='start point y', default=0.0)
    parser.add_argument('-gx', type=float, help='goal point x', default=50.0)
    parser.add_argument('-gy', type=float, help='goal point y', default=50.0)
    args = parser.parse_args()
    print ("___________________")
    print (args)

    return args