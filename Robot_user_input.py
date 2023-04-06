import argparse

"""
get user inputs
"""


def robot_user_input():
    parser = argparse.ArgumentParser(description='Code for Autonomous Robot.')
    parser.add_argument('-n', metavar="number of iteration", type=int, help='number of iteration', default=50)
    parser.add_argument('-m', metavar="data_map", help='map data', default='_MuchMoreFun.csv')
    parser.add_argument('-w', metavar="world_image", help='world model')
    parser.add_argument('-r', metavar="vision_range", type=float, help='vision range', default=20)
    parser.add_argument('-radius', metavar="robot radius", type=float, help='robot radius', default=0.5)
    parser.add_argument('-sx', metavar="start_x", type=float, help='start point x', default=0)
    parser.add_argument('-sy', metavar="start_y", type=float, help='start point y', default=0)
    parser.add_argument('-gx', metavar="goal_x", type=float, help='goal point x', default=75)
    parser.add_argument('-gy', metavar="goal_y", type=float, help='goal point y', default=50)
    parser.add_argument('-d', metavar="node density", type=int, help='node density', default=4)

    parser.add_argument('-p', metavar="picking strategy", type=str,
                        help='input : (g) for global first, (n) for neighbor first', default='g')

    parser.add_argument('-open_pts_type', metavar="open points type ", type=str,
                        help='input: (r) for get open points from RRTreeStar; (o) for get from open arcs', default='r')
    args = parser.parse_args()
    return args
