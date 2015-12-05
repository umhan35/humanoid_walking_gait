import numpy as np


def dh(θ, d, a, α):
    """
    :param θ: the angle between x0 and x1 measured in a plane normal to z0
    :param d: the distance between the origin o0 and the intersection of the x1 axis with z0 measured along the z0 axis
    :param a: the distance between the axes z0 and z1, and is measured along the axis x1
    :param α: the angle between the axes z0 and z1, measured in a plane normal to x1
    :return:
    """

    cθ = np.cos(θ)
    sθ = np.sin(θ)
    cα = np.cos(α)
    sα = np.sin(α)

    A = [[cθ, -sθ * cα,  sθ * sα, a * cθ],
         [sθ,  cθ * cα, -cθ * sα, a * sθ],
         [0, sα, cα, d],
         [0, 0, 0, 1]]

    return np.array(A)
