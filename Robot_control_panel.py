import argparse

'''
get user inputs
'''

def menu():
    
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', help='Number of runtimes', default=1)
    parser.add_argument('-m', help='map data', default='_map.csv')
    parser.add_argument('-w', help='world model')
    parser.add_argument('-s', help='start point <x_y>', default=[0.0, 0.0])
    parser.add_argument('-g', help='goal point <x_y>', default=[70.0, 70.0])
    args = parser.parse_args()
    print ("___________________")
    print (args)

    return args