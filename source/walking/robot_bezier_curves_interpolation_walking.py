from time import sleep

from .template_for_robot_interpolation_walking import TemplateForRobotInterpolationWalking
from walking.helpers import bezier_curves


class RobotBezierCurveInterpolationWalking(TemplateForRobotInterpolationWalking):

    def _take_steps(self, is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position):

        lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section\
            = self._get_3_walking_trajectory_sections(initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)

        while True:

            next_count = self.timer.next_count()

            time_fraction = next_count / self.step_timer.max_count
            interpolated_position = bezier_curves.get_interpolated_position(time_fraction, lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section)

            self.calculate_and_actuate_lower_limps_angle_set_at_good_timing(is_swing_leg_left=is_swing_leg_left, interpolated_position=interpolated_position, time_fraction=time_fraction)

            if self.timer.is_done():
                sleep(self.duration_of_rest)

                self.timer.restart()

                self.set_current_walking_set_index_to_next()

                is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position\
                    = self._get_is_swing_leg_left_and_walk_trajectory_four_positions__with_current_working_set(self.current_walking_set_index)

                lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section\
                    = self._get_3_walking_trajectory_sections(initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)
