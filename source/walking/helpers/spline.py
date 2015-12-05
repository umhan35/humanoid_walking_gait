
import numpy as np
from scipy.interpolate.fitpack import splrep, splev


class TwoDSpline:

    def __init__(self, knots_x, knots_y, n_points):
        self.knots_x = knots_x
        self.knots_y = knots_y

        tck = splrep(knots_x, knots_y)
        self.spline_x = np.linspace(knots_x[0], knots_x[-1], n_points)
        self.spline_y = splev(self.spline_x, tck)

    def get_point_at(self, count):
        return self.spline_x[count-1], self.spline_y[count-1]