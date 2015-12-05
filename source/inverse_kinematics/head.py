from math import atan

from util.geometry import get_angle_with_3_sides_known
from robot_spec.length_data import *
from util.point import Point
from util.pi import half_pi


def move_head(target_position, painter):
    target_position = Point(target_position)
    painter.draw_point(target_position)

    head_bottom_position = Point((0, 0, torso_height + neck_length))
    head_top_position = Point((0, 0, torso_height + neck_length + head_height))

    distance_between_head_bottom_and_target = head_bottom_position.distance_to(target_position)
    assert distance_between_head_bottom_and_target == head_height
    distance_between_head_top_and_target = head_top_position.distance_to(target_position)

    neck_lateral_radians = get_angle_with_3_sides_known(distance_between_head_top_and_target, head_height, head_height)

    def get_neck_transversal_radians(target_x, target_y):
        if target_x != 0:
            radiance = atan(target_y / target_x)
        else:  # perpendicular
            if target_y > 0:
                radiance = half_pi
            elif target_y < 0:
                radiance = -half_pi
            else:
                radiance = 0

        return radiance

    neck_transversal_radians = get_neck_transversal_radians(target_x=target_position[0], target_y=target_position[1])

    return neck_transversal_radians, neck_lateral_radians
