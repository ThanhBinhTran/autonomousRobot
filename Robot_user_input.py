import argparse

"""
get user inputs
"""


def robot_user_input():
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=50)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_MuchMoreFun.csv')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=20)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.5)
    parser.add_argument('-s', metavar="start x y", nargs='+', type=float, help='start x y', default=(0,0))
    parser.add_argument('-g', metavar="goal x y", nargs='+', type=float, help='goal x y', default=(75,50))
    parser.add_argument('-d', metavar="node density", type=int, help='node density', default=7)

    parser.add_argument('-p', metavar="picking strategy", type=str,
                        help='input : (g) for global first, (n) for neighbor first', default='g')

    parser.add_argument('-open_pts_type', metavar="open points type ", type=str,
                        help='input: (r) for get open points from RRTreeStar; (o) for get from open arcs', default='r')
    args = parser.parse_args()
    return args
