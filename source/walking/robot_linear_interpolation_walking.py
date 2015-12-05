from time import sleep
from .template_for_robot_interpolation_walking import TemplateForRobotInterpolationWalking


class RobotLinearInterpolationWalking(TemplateForRobotInterpolationWalking):

    DEBUG = False

    def _take_steps(self, is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position):

        lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section\
            = self._get_3_walking_trajectory_sections(initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)

        path_length = lift_up_front_trajectory_section.length + lift_up_front_forward_trajectory_section.length + put_down_trajectory_section.length

        def get_translation_unit(trajectory_section, count):
            return ((trajectory_section.end.x - trajectory_section.initial.x) / count, 0, (trajectory_section.end.z - trajectory_section.initial.z) / count)

        _lift_up_front_count = self.step_timer.max_count * (lift_up_front_trajectory_section.length / path_length)
        _lift_up_front_forward_count = self.step_timer.max_count * (lift_up_front_forward_trajectory_section.length / path_length)
        _put_down_count = self.step_timer.max_count * (put_down_trajectory_section.length / path_length)
        lift_up_front_count_threshold = _lift_up_front_count
        put_down_count_threshold = _lift_up_front_count + _lift_up_front_forward_count
        lift_up_front_translation_unit = get_translation_unit(lift_up_front_trajectory_section, _lift_up_front_count)
        lift_up_front_forward_translation_unit = get_translation_unit(lift_up_front_forward_trajectory_section, _lift_up_front_forward_count)
        put_down_translation_unit = get_translation_unit(put_down_trajectory_section, _put_down_count)

        if self.DEBUG: print('paths')
        if self.DEBUG: print(path_length)
        if self.DEBUG: print(lift_up_front_translation_unit[0] * _lift_up_front_count, _lift_up_front_count)
        if self.DEBUG: print(lift_up_front_forward_translation_unit[0] * _lift_up_front_forward_count, _lift_up_front_forward_count)
        if self.DEBUG: print(put_down_translation_unit[0] * _put_down_count, _put_down_count)

        self.curr_interpolated_position = initial_position

        while True:

            next_count = self.timer.next_count()
            if self.DEBUG: print('next_count max', next_count, self.step_timer.max_count)

            time_fraction = next_count / self.step_timer.max_count
            if self.DEBUG: print('time_fraction', time_fraction)

            if next_count < lift_up_front_count_threshold:
                if self.DEBUG: print('< lift_up_front_count_threshold')
                self.curr_interpolated_position = self.curr_interpolated_position.translate(*lift_up_front_translation_unit)
                print('curr_interpolated_position', self.curr_interpolated_position, lift_up_front_position)
            elif next_count < put_down_count_threshold:
                if self.DEBUG: print('< put_down_count_threshold')
                self.curr_interpolated_position = self.curr_interpolated_position.translate(*lift_up_front_forward_translation_unit)
                print('curr_interpolated_position', self.curr_interpolated_position, lift_up_front_forward_position)
            else:
                if self.DEBUG: print('> put_down_count_threshold')
                self.curr_interpolated_position = self.curr_interpolated_position.translate(*put_down_translation_unit)
                if self.DEBUG: print('curr_interpolated_position', self.curr_interpolated_position, put_down_position)

            self.calculate_and_actuate_lower_limps_angle_set_at_good_timing(is_swing_leg_left=is_swing_leg_left, interpolated_position=self.curr_interpolated_position, time_fraction=time_fraction)

            if self.timer.is_done():
                sleep(self.duration_of_rest)

                self.timer.restart()

                self.set_current_walking_set_index_to_next()

                is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position\
                    = self._get_is_swing_leg_left_and_walk_trajectory_four_positions__with_current_working_set(self.current_walking_set_index)

                lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section\
                    = self._get_3_walking_trajectory_sections(initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)

                path_length = lift_up_front_trajectory_section.length + lift_up_front_forward_trajectory_section.length + put_down_trajectory_section.length

                _lift_up_front_count = self.step_timer.max_count * (lift_up_front_trajectory_section.length / path_length)
                _lift_up_front_forward_count = self.step_timer.max_count * (lift_up_front_forward_trajectory_section.length / path_length)
                _put_down_count = self.step_timer.max_count * (put_down_trajectory_section.length / path_length)
                lift_up_front_count_threshold = _lift_up_front_count
                put_down_count_threshold = _lift_up_front_count + _lift_up_front_forward_count
                lift_up_front_translation_unit = get_translation_unit(lift_up_front_trajectory_section, _lift_up_front_count)
                lift_up_front_forward_translation_unit = get_translation_unit(lift_up_front_forward_trajectory_section, _lift_up_front_forward_count)
                put_down_translation_unit = get_translation_unit(put_down_trajectory_section, _put_down_count)

                if self.DEBUG: print(self.current_walking_set_index)
                if self.DEBUG: print('paths')
                if self.DEBUG: print(path_length)
                if self.DEBUG: print(lift_up_front_translation_unit[0] * _lift_up_front_count, _lift_up_front_count)
                if self.DEBUG: print(lift_up_front_forward_translation_unit[0] * _lift_up_front_forward_count, _lift_up_front_forward_count)
                if self.DEBUG: print(put_down_translation_unit[0] * _put_down_count, _put_down_count)

                self.curr_interpolated_position = initial_position