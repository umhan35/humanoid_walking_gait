from math import atan

from numpy import pi, degrees

from util.transformations import translate_3d_point, get_translate_matrix, get_rotate_matrix_along_axis, apply_matrix_to_vertex
from robot_spec.length_data import foot_height, upper_leg_length, lower_leg_length, leg_length
from robot_spec.position_data import left_hip_top_position, right_hip_top_position, left_hip_bottom_position, right_hip_bottom_position
from util.float import is_float_equal
from util.geometry import get_angle_with_3_sides_known
from util.angle import to_degrees
from util.point import Point


class InverseKinematicsLeg():

    DEBUG = False

    def __init__(self, is_left, hip_transversal_radians, target_position, robot_model, painter):

        self.robot_model = robot_model
        self.painter = painter

        self.is_left = is_left
        if self.DEBUG: print('is_left', is_left)
        if self.is_left:
            self.hip_top_position = Point(left_hip_top_position)
            self.hip_bottom_position = Point(left_hip_bottom_position)
        else:
            self.hip_top_position = Point(right_hip_top_position)
            self.hip_bottom_position = Point(right_hip_bottom_position)
        if self.DEBUG: print('hip top & bottom\n\t', self.hip_top_position, '\n\t', self.hip_bottom_position)

        self.hip_transversal_radians = hip_transversal_radians

        self.target_position = Point(target_position)
        self.target_at_ankle = Point(translate_3d_point(target_position, dz=foot_height))
        if self.DEBUG: print('target & at ankle\n\t', self.target_position, '\n\t', self.target_at_ankle)

        # counteract hip transversal
        self.target_at_ankle__with_hip_transversal_counteracted = self._get_target_with_hip_transversal_counteracted()
        if self.DEBUG: print('target_at_ankle__with_hip_transversal_counteracted\n\t', self.target_at_ankle__with_hip_transversal_counteracted)

    def get_angles(self):
        if self.DEBUG: self.painter.draw_point(self.target_position, color='y.')
        if self.DEBUG: self.painter.draw_point(self.target_at_ankle, color='y.')

        knee_lateral_radians, knee_interior_radians, knee_exterior_radians = self._get_knee_lateral()
        if self.DEBUG: print('knee_lateral_radians', degrees(knee_lateral_radians))

        hip_frontal_radians = self._get_hip_frontal()
        if self.DEBUG: print('hip_frontal_radians', degrees(hip_frontal_radians))

        hip_lateral_radians = - self._get_hip_lateral(hip_frontal_radians, knee_lateral_radians)
        if self.DEBUG: print('hip_lateral_radians', degrees(hip_lateral_radians))

        ankle_lateral_radians = -(abs(hip_lateral_radians) - knee_exterior_radians)
        if not self.is_left:
            ankle_lateral_radians *= -1
        if self.DEBUG: print('ankle_lateral_radians', ankle_lateral_radians)

        ankle_frontal_radians = hip_frontal_radians
        if self.DEBUG: print('ankle_frontal_radians', ankle_frontal_radians)

        if self.DEBUG: print('fd (hip_transversal, hip_frontal, hip_lateral, knee_lateral)')
        if self.DEBUG: print('  ', to_degrees((self.hip_transversal_radians, hip_frontal_radians, hip_lateral_radians, knee_lateral_radians)))
        return hip_frontal_radians, hip_lateral_radians, knee_lateral_radians, ankle_lateral_radians, ankle_frontal_radians

    def _get_target_with_hip_transversal_counteracted(self):
        move_back_to_origin = get_translate_matrix(-self.hip_top_position.x, -self.hip_top_position.y, -self.hip_top_position.z)
        rotate_along_z_axis = get_rotate_matrix_along_axis('z', self.hip_transversal_radians)
        move_back_to_point = get_translate_matrix(self.hip_top_position.x, self.hip_top_position.y, self.hip_top_position.z)

        target_at_ankle__with_hip_transversal_counteracted = apply_matrix_to_vertex(self.target_at_ankle, move_back_to_point.dot(rotate_along_z_axis.dot(move_back_to_origin)))

        return Point(target_at_ankle__with_hip_transversal_counteracted)

    def _get_knee_lateral(self):

        d1 = self.hip_bottom_position.distance_to(self.target_at_ankle__with_hip_transversal_counteracted)
        if self.DEBUG: print('d1', d1, self.hip_bottom_position, self.target_at_ankle__with_hip_transversal_counteracted)
        assert is_float_equal(d1, leg_length) or d1 < leg_length, '{} {} {}'.format(d1, leg_length, d1-leg_length)

        if is_float_equal(d1, leg_length):
            knee_interior_radians = pi
        else:
            knee_interior_radians = get_angle_with_3_sides_known(d1, upper_leg_length, lower_leg_length)
        knee_exterior_radians = pi - knee_interior_radians
        if self.DEBUG: print('knee interior & exterior angles', degrees(knee_interior_radians), degrees(knee_exterior_radians))

        if self.is_left:
            knee_lateral_radians = knee_exterior_radians
        else:
            knee_lateral_radians = -knee_exterior_radians

        return knee_lateral_radians, knee_interior_radians, knee_exterior_radians

    def _get_hip_frontal(self):

        target_at_ankle_with_hip_transversal_counteracted__y_offset_to_hip_bottom = abs(self.target_at_ankle__with_hip_transversal_counteracted.y - self.hip_bottom_position.y)
        target_at_ankle_with_hip_transversal_counteracted__z_offset_to_hip_bottom = abs(self.target_at_ankle__with_hip_transversal_counteracted.z - self.hip_bottom_position.z)

        hip_frontal_radians = atan(
                                    target_at_ankle_with_hip_transversal_counteracted__y_offset_to_hip_bottom
                                    /
                                    target_at_ankle_with_hip_transversal_counteracted__z_offset_to_hip_bottom)

        if self.is_left:
            return -hip_frontal_radians
        else:
            return hip_frontal_radians

    def _get_hip_lateral(self, hip_frontal_radians, knee_lateral_radians):

        if self.is_left:
            current_ankle = Point(self.robot_model.draw_left_lower_limp(left_hip_frontal_radians=hip_frontal_radians, left_knee_lateral_radians=knee_lateral_radians, draw=False)[-2].vertex)
        else:
            current_ankle = Point(self.robot_model.draw_right_lower_limp(right_hip_frontal_radians=hip_frontal_radians, right_knee_lateral_radians=knee_lateral_radians, draw=False)[-2].vertex)

        opposite_side_length = current_ankle.distance_to(self.target_at_ankle__with_hip_transversal_counteracted)
        if is_float_equal(opposite_side_length, 0):
            return 0

        target_at_ankle_with_hip_transversal_counteracted__radius = self.hip_bottom_position.distance_to(self.target_at_ankle__with_hip_transversal_counteracted)
        current_ankle_radius = self.hip_bottom_position.distance_to(current_ankle)
        assert is_float_equal(target_at_ankle_with_hip_transversal_counteracted__radius, current_ankle_radius), '{} {}'.format(target_at_ankle_with_hip_transversal_counteracted__radius, current_ankle_radius)

        assert opposite_side_length < current_ankle_radius + target_at_ankle_with_hip_transversal_counteracted__radius
        hip_lateral_radians = get_angle_with_3_sides_known(opposite_side_length, current_ankle_radius, target_at_ankle_with_hip_transversal_counteracted__radius)
        if self.is_left:
            return hip_lateral_radians
        else:
            return -hip_lateral_radians