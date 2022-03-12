import argparse

"""
get user inputs
"""

def menu():
    
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="run_times", type=int, help='Number of runtimes', default=1)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_map.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=-1.0)
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=0.0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=0.0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=50.0)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=50.0)
    
    args = parser.parse_args()
    print ("___________________")
    print (args)

    return args