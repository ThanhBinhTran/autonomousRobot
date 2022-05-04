import argparse

"""
get user inputs
"""

def menu_RRT():
    parser = argparse.ArgumentParser(description='RRT animation.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=1)
    parser.add_argument('-m', metavar="map name", help='map name', default='_block.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-step_size', metavar="step_size", type=float, help='step size', default=5.0)
    parser.add_argument('-rx', metavar="random_area x lim ", type=float, help='random area x_lim', default=0.0)
    parser.add_argument('-ry', metavar="random_area y lim", type=float, help='random area y_lim', default=100.0)
    parser.add_argument('-ss', metavar="sample_size", type=int, help='sample size', default=50)
    parser.add_argument('-radius', metavar="radius", type=float, help='radius', default=15.0)
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=0.0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=0.0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=100.0)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=100.0)
    
    args = parser.parse_args()

    return args