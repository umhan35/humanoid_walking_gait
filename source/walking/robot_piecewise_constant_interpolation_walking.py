from time import sleep

from util.point import Point
from walking.template_for_robot_interpolation_walking import TemplateForRobotInterpolationWalking


class RobotPiecewiseConstantInterpolationWalking(TemplateForRobotInterpolationWalking):

    DEBUG = False

    def _take_steps(self, is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position):

        self.trajectory_sections = self._get_3_walking_trajectory_sections(initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)

        def get_path_length():
            a, b, c, d = initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position
            return (b.x - a.x) + abs(b.z - a.z) + abs(c.x - b.x) + abs(c.z - b.z) + abs(d.x - c.x) + abs(d.z - c.z)

        # path length, path unit length
        path_length = get_path_length()
        increment_unit = path_length / self.step_timer.max_count
        if self.DEBUG: print('\npath_length', path_length)
        if self.DEBUG: print('increment_unit', increment_unit)

        self.curr_interpolated_position = initial_position
        self.current_trajectory_section_index = 0

        while True:

            while True:  # traverse all 3 trajectories

                next_count = self.timer.next_count()
                if self.DEBUG: print('\nnext_count', next_count)

                time_fraction = next_count / self.step_timer.max_count

                if self.current_trajectory_section_index == len(self.trajectory_sections):
                    if self.DEBUG: print('current_trajectory: last')
                    break

                curr_trajectory = self.trajectory_sections[self.current_trajectory_section_index]
                if self.DEBUG: print('curr_trajectory', self.current_trajectory_section_index, curr_trajectory.initial, curr_trajectory.end)

                curr_midpoint = curr_trajectory.mid_point
                if self.DEBUG: print('curr_midpoint', curr_midpoint)

                curr_x, curr_y, curr_z = self.curr_interpolated_position.x, self.curr_interpolated_position.y, self.curr_interpolated_position.z
                if self.DEBUG: print('curr_interpolated_position', self.curr_interpolated_position)

                end_x, init_z, end_z = curr_trajectory.end.x, curr_trajectory.initial.z, curr_trajectory.end.z

                def is_curr_z_reached_end_z():
                    z_diff = end_z - init_z

                    if z_diff == 0 or curr_z == end_z:
                        return True
                    elif z_diff > 0:
                        if curr_z > end_z and curr_z - increment_unit < end_z:
                            return True
                        else:
                            return False
                    else:
                        # z_diff < 0
                        if curr_z < end_z and curr_z - (-increment_unit) < end_z:
                            return True
                        else:
                            return False

                is_curr_z_reached_end_z = is_curr_z_reached_end_z()
                if is_curr_z_reached_end_z:
                    if self.DEBUG: print('end z reached.')
                else:
                    if self.DEBUG: print('end z not reached')

                if curr_x < curr_midpoint.x and (curr_z == init_z or abs(curr_z - init_z) < abs(increment_unit)):
                    # reaching half x
                    if self.DEBUG: print('reaching half x', curr_midpoint.x, curr_x)
                    curr_x += increment_unit
                elif (curr_x == curr_midpoint.x or (curr_x > curr_midpoint.x and curr_x - increment_unit < curr_midpoint.x)) and not is_curr_z_reached_end_z:
                    # half x reached, reaching end z
                    if self.DEBUG: print('half x reached, reaching end z', end_z, curr_z)
                    if self.DEBUG: print('curr_z', curr_z)
                    if end_z > curr_z:
                        curr_z += increment_unit
                    else:
                        curr_z -= increment_unit
                    if self.DEBUG: print('curr_z', curr_z)
                elif is_curr_z_reached_end_z and curr_x < end_x:
                    # end z reached, reaching end x
                    if self.DEBUG: print('end z reached, reaching end x', end_x, curr_x)

                    # curr_x += increment_unit

                    ### HACK
                    if next_count >= self.step_timer.max_count:
                        curr_x = end_x
                        self.current_trajectory_section_index += 1
                        break
                    else:
                        curr_x += increment_unit
                elif curr_x == end_x or (curr_x > end_x and curr_x - increment_unit < end_x):
                    if self.DEBUG: print('end x reached, ', end_x, curr_x)
                    self.current_trajectory_section_index += 1
                else:
                    if self.DEBUG: print('WARNING')
                    exit()

                self.curr_interpolated_position = Point((curr_x, curr_y, curr_z))

                self.calculate_and_actuate_lower_limps_angle_set_at_good_timing(is_swing_leg_left=is_swing_leg_left, interpolated_position=self.curr_interpolated_position, time_fraction=time_fraction)

            if self.timer.is_done():
                return
                sleep(self.duration_of_rest)

                self.timer.restart()

                self.set_current_walking_set_index_to_next()

                is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position\
                    = self._get_is_swing_leg_left_and_walk_trajectory_four_positions__with_current_working_set(self.current_walking_set_index)

                self.trajectory_sections = self._get_3_walking_trajectory_sections(initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position)

                self.curr_interpolated_position = initial_position
                self.current_trajectory_section_index = 0
