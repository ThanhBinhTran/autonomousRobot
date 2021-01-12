from graphics import *

def intersection(center, radius, p1, p2):

    """ find the two points where a secant intersects a circle """

    dx, dy = p2.x - p1.x, p2.y - p1.y

    a = dx**2 + dy**2
    b = 2 * (dx * (p1.x - center.x) + dy * (p1.y - center.y))
    c = (p1.x - center.x)**2 + (p1.y - center.y)**2 - radius**2

    discriminant = b**2 - 4 * a * c
    assert (discriminant > 0), 'Not a secant!'

    t1 = (-b + discriminant**0.5) / (2 * a)
    t2 = (-b - discriminant**0.5) / (2 * a)

    return Point(dx * t1 + p1.x, dy * t1 + p1.y), Point(dx * t2 + p1.x, dy * t2 + p1.y)

def main(win):
    center = Point(0.0, 0.0)

    # Enter radius

    radius = float(input("Put in radius: "))

    # Draw circle and center dot

    Circle(center, radius).draw(win)
    Circle(center, 0.3).draw(win)

    # Enter the y intercept of the line
    yinter = float(input("Put in y intercept: "))

    # Draw line

    p1, p2 = Point(-10.0, 0.0), Point(10.0, yinter)
    Line(p1, p2).draw(win)

    # Mark two points of intercept in red

    for i, root in enumerate(intersection(center, radius, p1, p2), start=1):
        print("Root {}:".format(i), root)
        dot = Circle(root, 0.3)
        dot.setFill('red')
        dot.draw(win)

win = GraphWin()
win.setCoords(-10.0, -10.0, 10.0, 10.0)

main(win)

win.getMouse()