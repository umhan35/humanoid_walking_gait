import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate.fitpack import splrep, splev

from util.point import Point
from walking.helpers import bezier_curves
from walking.helpers.bezier_curves import LineSegment


x = (0, 2.0, 6.0, 8.0)
y = (-47.7, -44.7, -44.7, -47.7)

tck = splrep(x, y)

x2 = np.linspace(0, 8)
y2 = splev(x2, tck)

points = [Point((x_, 0, y_)) for (x_, y_) in zip(x, y)]
l1 = LineSegment(points[0], points[1])
l2 = LineSegment(points[1], points[2])
l3 = LineSegment(points[2], points[3])

yy2 = [bezier_curves.get_interpolated_position(x2_/8, l1, l2, l3)[2] for x2_ in x2]

plt.plot(x, y, 'o', x, y, '--', x2, y2, x2, yy2)

plt.show()