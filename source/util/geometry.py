from numpy import sqrt, square
from math import acos


def distance_between_2_vertices_in_3d(a, b):
    return sqrt(square(a[0] - b[0]) + square(a[1] - b[1]) + square(a[2] - b[2]))


def get_angle_with_3_sides_known(opposite_side_length, any_other_side_1, any_other_side_2):
    return acos(
        (square(any_other_side_1) + square(any_other_side_2) - square(opposite_side_length))
        /
        (2 * any_other_side_1 * any_other_side_2))