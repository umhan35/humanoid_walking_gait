from math import pi, cos, asin

from util.float import is_float_equal, is_float_greater_than_0, is_float_less_than_0
from util.geometry import get_angle_with_3_sides_known, distance_between_2_vertices_in_3d
from robot_spec.length_data import shoulder_y_offset, lower_and_upper_arm_difference, outer_shoulder_z_offset, arm_length, \
    upper_arm_length, lower_arm_length
from util.pi import half_pi
from util.point import Point
from robot_spec.position_data import left_outer_shoulder_position, right_outer_shoulder_position


class InverseKinematicsArm():

    def __init__(self, target_position, robot_model, painter, is_left):

        self.robot_model = robot_model
        self.painter = painter

        self.target_position = Point(target_position)
        self.is_left = is_left

        self.outer_shoulder_position = left_outer_shoulder_position if self.is_left else right_outer_shoulder_position
        self.draw_arm = self.robot_model.draw_left_arm if self.is_left else self.robot_model.draw_right_arm

        self.is_target_above_body = self.target_position.x > 0

    def _validate_arm_target_position(self):
        target_x, target_y, target_z = self.target_position
        # for both left and right arms

        is_target_not_behind_body = is_float_equal(target_x, 0) or is_float_greater_than_0(target_x)
        assert is_target_not_behind_body, target_x

        # when (0, 0, 180)
        assert abs(target_y) >= shoulder_y_offset - lower_and_upper_arm_difference

    def _get_elbow_lateral_radians(self):
        # print('\n###### elbow lateral ######\n')

        d1 = distance_btw_outer_shoulder_position_and_target = distance_between_2_vertices_in_3d(
            self.outer_shoulder_position, self.target_position)
        # print('d1', d1, 'arm_length', arm_length, 'lower_upper_arm_difference', abs(lower_arm_length - upper_arm_length))
        assert is_float_equal(d1, arm_length) or d1 < arm_length

        if is_float_equal(d1, lower_and_upper_arm_difference):
            _elbow_interior_radiance = 0
        elif is_float_equal(d1, arm_length):
            _elbow_interior_radiance = pi
        else:
            _elbow_interior_radiance = get_angle_with_3_sides_known(d1, upper_arm_length, lower_arm_length)
        # print('_elbow_interior_degree', degrees(_elbow_interior_radiance))

        _elbow_exterior_radiance = pi - _elbow_interior_radiance
        # print('_elbow_exterior_degree', degrees(_elbow_exterior_radiance))
        assert 0 <= _elbow_interior_radiance <= pi

        if self.is_left:
            elbow_lateral_radians = -_elbow_exterior_radiance
        else:
            elbow_lateral_radians = _elbow_exterior_radiance

        # print('elbow_lateral_radians', degrees(elbow_lateral_radians))
        return elbow_lateral_radians, _elbow_interior_radiance, _elbow_exterior_radiance

    def _get_shoulder_frontal_radians(self, _elbow_interior_radiance, _elbow_exterior_radiance):

    #######
    ####### shoulder frontal......
    #######

        # lower arm projected to arm plane (form by upper, lower arm and the target position)

        # print('\n###### shoulder frontal ######\n')

        is_elbow_exterior_btw_0_and_90 = (0 <= _elbow_exterior_radiance <= half_pi)
        if is_elbow_exterior_btw_0_and_90:
            # print('cos(_elbow_exterior_radiance)', cos(_elbow_exterior_radiance))
            lower_arm_length_projected_on_arm_line = cos(_elbow_exterior_radiance) * lower_arm_length
            # print('lower_arm_length_projected_on_arm_line', lower_arm_length_projected_on_arm_line, lower_arm_length)
            assert not is_float_less_than_0(lower_arm_length_projected_on_arm_line), lower_arm_length_projected_on_arm_line
            arm_length_projected_on_arm_line = upper_arm_length + lower_arm_length_projected_on_arm_line
            # print('arm_length_projected_on_arm_line', arm_length_projected_on_arm_line)
        else:  # >90
            # print('cos(_elbow_interior_radiance)', cos(_elbow_interior_radiance))
            lower_arm_length_projected_on_arm_line = cos(_elbow_interior_radiance) * lower_arm_length
            # print('lower_arm_length_projected_on_arm_line', lower_arm_length_projected_on_arm_line, lower_arm_length)
            assert not is_float_less_than_0(lower_arm_length_projected_on_arm_line), lower_arm_length_projected_on_arm_line
            arm_length_projected_on_arm_line = abs(upper_arm_length - lower_arm_length_projected_on_arm_line)
            # print('arm_length_projected_on_arm_line', arm_length_projected_on_arm_line)

        target_y_offset_to_outer_shoulder = abs(self.target_position.y - self.outer_shoulder_position[1])
        # print('target_y_offset_to_outer_shoulder', target_y_offset_to_outer_shoulder)
        tmp = target_y_offset_to_outer_shoulder / arm_length_projected_on_arm_line
        # print('target_y_offset_to_outer_shoulder / arm_length_projected_on_arm_line', target_y_offset_to_outer_shoulder / arm_length_projected_on_arm_line)
        if is_float_equal(tmp, 1):
            angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y = half_pi
        else:
            if target_y_offset_to_outer_shoulder == 0 and self.is_target_above_body:
                angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y = pi
            else:
                angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y = asin(target_y_offset_to_outer_shoulder / arm_length_projected_on_arm_line)
        # print('angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y', degrees(angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y))

        is_target_on_body_plane = is_float_equal(self.target_position.x, 0)
        is_target_in_front_of_body = not is_target_on_body_plane and self.target_position.x > 0
        is_target_on_or_in_front_of_body_plane = is_target_on_body_plane or is_target_in_front_of_body

        is_target_on_outer_shoulder_natural_plane = is_float_equal(self.target_position.y, self.outer_shoulder_position[1])

        is_target_on_shoulder = is_float_equal(self.target_position.z, outer_shoulder_z_offset)
        is_target_above_shoulder = not is_target_on_shoulder and self.target_position.z > outer_shoulder_z_offset
        is_target_below_shoulder = not is_target_on_shoulder and self.target_position.z < outer_shoulder_z_offset
        is_target_on_or_above_shoulder = is_target_on_shoulder or is_target_above_shoulder

        if self.is_left:
            is_target_right_to_right_outer_shoulder = not is_target_on_outer_shoulder_natural_plane and self.target_position.y > self.outer_shoulder_position[1]
            is_target_on_or_left_to_right_outer_shoulder_natural_plane = is_target_on_outer_shoulder_natural_plane or is_target_right_to_right_outer_shoulder
            is_target_left_to_right_outer_shoulder = not is_target_on_outer_shoulder_natural_plane and self.target_position.y < self.outer_shoulder_position[1]

            case1 = is_target_on_or_in_front_of_body_plane and is_target_on_or_left_to_right_outer_shoulder_natural_plane and is_target_on_or_above_shoulder
            case2 = is_target_on_or_in_front_of_body_plane and is_target_left_to_right_outer_shoulder and is_target_below_shoulder

            if case1 or case2:
                shoulder_frontal_radians = -(half_pi - angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y)
            else:
                shoulder_frontal_radians = half_pi - angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y
        else:
            is_target_left_to_left_outer_shoulder = not is_target_on_outer_shoulder_natural_plane and self.target_position.y < self.outer_shoulder_position[1]
            is_target_on_or_right_to_left_outer_shoulder_natural_plane = is_target_on_outer_shoulder_natural_plane or is_target_left_to_left_outer_shoulder
            is_target_right_to_left_outer_shoulder = not is_target_on_outer_shoulder_natural_plane and self.target_position.y > self.outer_shoulder_position[1]

            case1 = is_target_on_or_in_front_of_body_plane and is_target_on_or_right_to_left_outer_shoulder_natural_plane and is_target_on_or_above_shoulder
            case2 = is_target_on_or_in_front_of_body_plane and is_target_right_to_left_outer_shoulder and is_target_below_shoulder

            if case1 or case2:
                shoulder_frontal_radians = half_pi - angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y
            else:
                shoulder_frontal_radians = -(half_pi - angle_btw__lower_arm_projected_on_arm_line__and__target_y_to_shoulder_y)

        # print('shoulder_frontal_radians', degrees(shoulder_frontal_radians))
        return shoulder_frontal_radians

    def __get_shoulder_lateral(self, shoulder_frontal_radians, elbow_lateral_radians):
        is_target_above_shoulder = self.target_position.z > outer_shoulder_z_offset
        # print('shoulder_frontal_radians', degrees(shoulder_frontal_radians))

        def _project_on_outer_shoulder_natural_plane(point):
            return (point[0], self.outer_shoulder_position[1], point[2])

        current_wrist_with_shoulder_lateral_be_0 = self.draw_arm(0, shoulder_frontal_radians, elbow_lateral_radians, draw=False)[-1]
        the_current_wrist_on_outer_shoulder_natural_plane = _project_on_outer_shoulder_natural_plane(current_wrist_with_shoulder_lateral_be_0)
        # print('wrist current:', current_wrist_with_shoulder_lateral_be_0)
        # print('wrist on shoulder natural plane:', the_current_wrist_on_outer_shoulder_natural_plane)

        target_on_outer_shoulder_natural_plane = _project_on_outer_shoulder_natural_plane(self.target_position)

        # print('wrist & target on shoulder natural plane:', the_current_wrist_on_outer_shoulder_natural_plane, target_on_outer_shoulder_natural_plane)
        # painter.draw_link(left_outer_shoulder_position, the_current_wrist_on_outer_shoulder_natural_plane, color='g-')
        # painter.draw_link(left_outer_shoulder_position, target_on_outer_shoulder_natural_plane, color='g-')

        distance_btw_outer_shoulder_position_and_target_on_outer_shoulder_natural_plane = distance_between_2_vertices_in_3d(self.outer_shoulder_position, target_on_outer_shoulder_natural_plane)
        distance_btw_outer_shoulder_position_and_current_wrist_on_outer_shoulder_natural_plane = distance_between_2_vertices_in_3d(self.outer_shoulder_position, the_current_wrist_on_outer_shoulder_natural_plane)
        assert is_float_equal(distance_btw_outer_shoulder_position_and_target_on_outer_shoulder_natural_plane, distance_btw_outer_shoulder_position_and_current_wrist_on_outer_shoulder_natural_plane),\
                              '{}, {}'.format(distance_btw_outer_shoulder_position_and_target_on_outer_shoulder_natural_plane, distance_btw_outer_shoulder_position_and_current_wrist_on_outer_shoulder_natural_plane)

        outer_shoulder_working_circle_radius__on_outer_shoulder_natural_plane = distance_btw_outer_shoulder_position_and_target_on_outer_shoulder_natural_plane

        if is_float_equal(0, outer_shoulder_working_circle_radius__on_outer_shoulder_natural_plane):
            _shoulder_lateral_radians = 0
        else:
            # print(outer_shoulder_working_circle_radius__on_outer_shoulder_natural_plane, distance_between_2_vertices_in_3d(left_outer_shoulder_position, the_current_wrist_on_outer_shoulder_natural_plane))
            assert is_float_equal(distance_between_2_vertices_in_3d(self.outer_shoulder_position, the_current_wrist_on_outer_shoulder_natural_plane),
                                  outer_shoulder_working_circle_radius__on_outer_shoulder_natural_plane)

            distance_between_above_2_projected_on_outer_shoulder_natural_plane = distance_between_2_vertices_in_3d(the_current_wrist_on_outer_shoulder_natural_plane, target_on_outer_shoulder_natural_plane)
            # if is_float_equal(distance_between_above_2_projected_on_outer_shoulder_natural_plane, 2 * outer_shoulder_working_circle_radius__on_outer_shoulder_natural_plane):
            # else:
            _shoulder_lateral_radians = get_angle_with_3_sides_known(distance_between_above_2_projected_on_outer_shoulder_natural_plane, outer_shoulder_working_circle_radius__on_outer_shoulder_natural_plane, outer_shoulder_working_circle_radius__on_outer_shoulder_natural_plane)

            if not is_target_above_shoulder:
                _shoulder_lateral_radians *= -1

        if not self.is_left:
            _shoulder_lateral_radians *= -1

        # print('_shoulder_lateral_radians', degrees(_shoulder_lateral_radians))
        return _shoulder_lateral_radians

    def get_angles(self):
        # print('mla', target_position)
        # painter.draw_point(self.target_position)

        self._validate_arm_target_position()

        elbow_lateral_radians, _elbow_interior_radiance, _elbow_exterior_radiance = self._get_elbow_lateral_radians()

        shoulder_frontal_radians = self._get_shoulder_frontal_radians(_elbow_interior_radiance, _elbow_exterior_radiance)

        # shoulder lateral...... Tried to extract to another method, but somehow failed...

        ### first project target arm on xz plane, based on what we know(the 2 angles: elbow lateral and shoulder frontal),
        ### calculate the other angle in the arm triangle,
        ### then calculate big_angle_btw__outer_shoulder_to_target__and__outer_shoulder_vertical_line and
        ### angle_btw_projected_upper_arm_and_projected_d1

        # print('\n###### shoulder lateral ######\n')

        # assert abs(target_y) >= outer_shoulder_y_offset

        shoulder_lateral_radians = self.__get_shoulder_lateral(shoulder_frontal_radians, elbow_lateral_radians)

        curr_wrist_position = self.draw_arm(shoulder_lateral_radians, shoulder_frontal_radians, elbow_lateral_radians, draw=False)[-1]

        def is_point_equal(a, b):
            return is_float_equal(a[0], b[0]) and is_float_equal(a[1], b[1]) and is_float_equal(a[2], b[2])

        # if not is_float_equal(curr_wrist_position[0], target_x) or not is_float_equal(curr_wrist_position[1], target_y) or not is_float_equal(curr_wrist_position[2], target_z):
        if not is_point_equal(curr_wrist_position, self.target_position):

            for changed_shoulder_frontal_radians in (-shoulder_frontal_radians, shoulder_frontal_radians):
                shoulder_frontal_radians = changed_shoulder_frontal_radians
                shoulder_lateral_radians = self.__get_shoulder_lateral(shoulder_frontal_radians, elbow_lateral_radians)
                curr_wrist_position = self.draw_arm(shoulder_lateral_radians, shoulder_frontal_radians, elbow_lateral_radians, draw=False)[-1]
                if not is_point_equal(curr_wrist_position, self.target_position):
                    shoulder_lateral_radians *= -1
                    # print('shoulder_frontal_radians', degrees(shoulder_frontal_radians))
                    # print('_shoulder_lateral_radians', degrees(shoulder_lateral_radians))
                    curr_wrist_position = self.draw_arm(shoulder_lateral_radians, shoulder_frontal_radians, elbow_lateral_radians, draw=False)[-1]

                if is_point_equal(curr_wrist_position, self.target_position):
                    # print('equals now!!')
                    break

        # print('\nfinal_degrees', (degrees(shoulder_lateral_radians), degrees(shoulder_frontal_radians), degrees(elbow_lateral_radians)))
        return shoulder_lateral_radians, shoulder_frontal_radians, elbow_lateral_radians
