# draw a house with a window and a door
# translate & rotate using matrix


import matplotlib.pyplot as plt # conda install anaconda
import numpy as np
import math


fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_aspect('equal')


def draw(shape):
    # print(shape)
    x_values = shape[:,1]
    y_values = shape[:,0]
    z_values = shape[:,2]

    plt.plot(x_values, y_values, z_values, 'r-')


def draw_shapes(shapes):
    for s in shapes:
        draw(s)
        s = rotate(s)
        draw(s)


def rotate(np_array):
    rotated = np.zeros(np_array.shape)

    # rotate_matrix = np.array([[np.cos(d), -1 * np.sin(d)], [np.sin(d), np.cos(d)]])
    # rotate_matrix = np.array([[np.cos(d), -1 * np.sin(d)], [np.sin(d), np.cos(d)]])
    # rotate_matrix = np.array([[.94,.34,-3],   [-.34, .94, -5],   [0,0,1]])
    rotate_matrix = np.array([[-1.83,0.49,0],   [-1.99, -1.92, -0],   [0,0,1]])

    for i, point in enumerate(np_array):
        rotated[i] = rotate_matrix.dot(np.array(point))

    return rotated


house_body = np.array([[0,0,1], [0,3,1], [2,3,1], [3,2,1], [2,0,1], [0,0,1]])

plt.axis([-10,10,-10,10])
draw_shapes([house_body])

ax.set_xlabel('Y')
ax.set_ylabel('X')
plt.show()
