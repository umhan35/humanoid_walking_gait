from time import sleep
from walking.helpers.spline import TwoDSpline
from walking.template_for_robot_interpolation_walking import TemplateForRobotInterpolationWalking


class RobotSplineInterpolationWalking(TemplateForRobotInterpolationWalking):

    DEBUG = False

    def _take_steps(self, is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position):

        if self.DEBUG: print('is_swing_leg_left', is_swing_leg_left)
        if self.DEBUG: print('positions', initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)

        spline_knots_x = (initial_position.x, lift_up_front_position.x, lift_up_front_forward_position.x, put_down_position.x)
        spline_knots_z = (initial_position.z, lift_up_front_position.z, lift_up_front_forward_position.z, put_down_position.z)

        spline = TwoDSpline(spline_knots_x, spline_knots_z, self.step_timer.max_count)

        while True:

            next_count = self.timer.next_count()
            if self.DEBUG: print('\nnext_count', next_count)

            time_fraction = next_count / self.step_timer.max_count

            spline_point = spline.get_point_at(next_count)
            if self.DEBUG: print('spline_point', spline_point)
            interpolated_position = (spline_point[0], initial_position.y, spline_point[1])
            if self.DEBUG: print('interpolated_position', interpolated_position)

            self.calculate_and_actuate_lower_limps_angle_set_at_good_timing(is_swing_leg_left=is_swing_leg_left, interpolated_position=interpolated_position, time_fraction=time_fraction)

            if self.timer.is_done():
                sleep(self.duration_of_rest)

                self.timer.restart()

                self.set_current_walking_set_index_to_next()

                is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position\
                    = self._get_is_swing_leg_left_and_walk_trajectory_four_positions__with_current_working_set(self.current_walking_set_index)

                if self.DEBUG: print('is_swing_leg_left', is_swing_leg_left)
                if self.DEBUG: print('positions', initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)

                spline_knots_x = (initial_position.x, lift_up_front_position.x, lift_up_front_forward_position.x, put_down_position.x)
                spline_knots_z = (initial_position.z, lift_up_front_position.z, lift_up_front_forward_position.z, put_down_position.z)

                spline = TwoDSpline(spline_knots_x, spline_knots_z, self.step_timer.max_count)
