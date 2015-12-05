import matplotlib
matplotlib.use('Qt4Agg')
# matplotlib.use('TKAgg')

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl

from util.transformations import apply_matrix_to_vertices

mpl.rcParams['figure.figsize'] = 10, 8


class Painter:

    def __init__(self, azimuth=5, elevation=45):
        self.fig, self.ax = self.configure_plot(azimuth, elevation)

    @staticmethod
    def configure_plot(azimuth, elevation):

        fig = plt.figure()

        ax = fig.add_subplot(111, projection='3d')
        plt.ion()
        ax.view_init(azim=azimuth, elev=elevation)  # default: 30, 60 nice: 5, 45

        ## tried to make x, y, z axis have the same unit, but not working...
        # ax = fig.add_subplot(111, projection='3d', aspect='equal')
        # ax.axis('equal')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        return fig,ax

    def draw(self):
        # NOTE the following 2 lines of code to set aspect "equal" are from
        # http://stackoverflow.com/questions/8130823/set-matplotlib-3d-plot-aspect-ratio
        scaling = np.array([getattr(self.ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        self.ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)

        plt.draw()

    def show(self, block=True):

        # NOTE the following 2 lines of code to set aspect "equal" are from
        # http://stackoverflow.com/questions/8130823/set-matplotlib-3d-plot-aspect-ratio
        scaling = np.array([getattr(self.ax, 'get_{}lim'.format(dim))() for dim in 'xyz'])
        self.ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]]*3)

        plt.show(block=block)

    def draw_point(self, p, color='bo', line_width=1):
        return self.ax.plot([p[0]], [p[1]], [p[2]], color, linewidth=line_width)

    def draw_line(self, a, b, line_width=1, color='c-'):
        link = np.array([a, b])
        return self.draw_with_points(link, color=color, line_width=line_width)

    def draw_with_points(self, verts, color='c-', line_width=1):
        x_values = verts[:,0]
        y_values = verts[:,1]
        z_values = verts[:,2]

        return self.ax.plot(x_values, y_values, z_values, color, linewidth=line_width)[0]

    def draw_global_coordinates(self, unit=2):
        self.draw_coordinates(unit=unit)

    def draw_coordinates(self, transformation_matrix=None, unit=1):
        x_axis = np.array([[0,0,0], [unit,0,0]])
        y_axis = np.array([[0,0,0], [0,unit,0]])
        z_axis = np.array([[0,0,0], [0,0,unit]])

        if transformation_matrix is not None:
            x_axis = apply_matrix_to_vertices(x_axis, transformation_matrix)
            y_axis = apply_matrix_to_vertices(y_axis, transformation_matrix)
            z_axis = apply_matrix_to_vertices(z_axis, transformation_matrix)

        self.draw_with_points(x_axis, color='r-')
        self.draw_with_points(y_axis, color='g-')
        self.draw_with_points(z_axis, color='b-')