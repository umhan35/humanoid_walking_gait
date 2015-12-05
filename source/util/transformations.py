import numpy as np


# tx, ty, tz are angles
def get_rotate_matrix(tx, ty, tz): # tx: pitch, ty: yaw, tz: roll
    rotated_x = get_rotate_matrix_along_axis('x', tx)
    rotated_y = get_rotate_matrix_along_axis('y', ty)
    rotated_z = get_rotate_matrix_along_axis('z', tz)

    rotated_xyz = rotated_z.dot(rotated_y.dot(rotated_x))

    return rotated_xyz


def get_rotate_matrix_along_axis(axis, radians):

    # counter clockwise
    rotate_matrices = {'z': np.array([[np.cos(radians), -1 * np.sin(radians), 0, 0], [np.sin(radians), np.cos(radians), 0, 0], [0,0,1,0], [0,0,0,1]]),
                       'y': np.array([[np.cos(radians), 0, -1 * np.sin(radians), 0], [0,1,0,0], [np.sin(radians), 0, np.cos(radians), 0], [0,0,0,1]]),
                       'x': np.array([[1,0,0,0], [0, np.cos(radians), -1 * np.sin(radians), 0], [0, np.sin(radians), np.cos(radians), 0], [0,0,0,1]]) }

    return rotate_matrices[axis]


def get_translate_matrix(dx, dy, dz):
    return np.array([[1,0,0,dx], [0,1,0,dy], [0,0,1,dz], [0,0,0,1]])


def apply_matrix_to_vertices(xyz_points, matrix):
    transformed = np.zeros(xyz_points.shape)

    for i, point in enumerate(xyz_points):
        point = np.array([[point[0]], [point[1]], [point[2]], [1]])

        result_matrix = matrix.dot(point)
        transformed[i] = (result_matrix[0], result_matrix[1], result_matrix[2])

    return transformed


def apply_matrix_to_vertex(xyz_point, matrix):
    return apply_matrix_to_vertices(np.array([xyz_point]), matrix)[0]


def apply_matrix_to_origin(matrix):
    return apply_matrix_to_vertex(np.array([0,0,0]), matrix)


def translate_3d_point(xyz_point, dx=0, dy=0, dz=0):
    return apply_matrix_to_vertex(xyz_point, get_translate_matrix(dx, dy, dz))
