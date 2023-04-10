import argparse

"""
get user inputs
"""


def RRTree_user_input():
    parser = argparse.ArgumentParser(description='RRT animation.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=0)
    parser.add_argument('-m', metavar="map name", help='map name', default='_map_blocks_1.csv')
    parser.add_argument('-w', metavar="world_image", help='world model',  default=None)
    parser.add_argument('-step_size', metavar="step_size", type=float, help='step size', default=5.0)
    parser.add_argument('-d', metavar="node density", type=int, help='node density', default=6)
    parser.add_argument('-radius', metavar="neighbour radius", type=float, help='radius', default=10.0)
    parser.add_argument('-r', metavar="vision range", type=float, help='vision range', default=10.0)
    parser.add_argument('-s', metavar="start x, y", nargs='+', type=float, help='start point x', default= (0, 0) )
    parser.add_argument('-g', metavar="goal x y", nargs='+', type=float, help='goal point x', default= (99, 99) )

    args = parser.parse_args()

    return args
