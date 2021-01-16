"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


def intersection(x, y, radius, ls_points): # line segment points

    """ find the two points where a secant intersects a circle """
    p_is = []
    p1x,p1y = ls_points[0]
    p2x,p2y = ls_points[1]

    dx, dy = np.subtract(p2x,p1x), np.subtract(p2y, p1y)

    a = dx**2 + dy**2
    b = 2 * (dx * (p1x - x) + dy * (p1y - y))
    c = (p1x - x)**2 + (p1y - y)**2 - radius**2

    discriminant = b**2 - 4 * a * c
    if discriminant > 0:
        t1 = (-b + discriminant**0.5) / (2 * a)
        t2 = (-b - discriminant**0.5) / (2 * a)
        p_is.append([dx * t1 + p1x, dy * t1 + p1y])
        p_is.append([dx * t2 + p1x, dy * t2 + p1y])
    return p_is
