from time import sleep
from robot_client_server.robot_clients import RealRobotClient

from util.point import Point
from walking.helpers.performance_timer import PerformanceTimer
from walking.robot_walking_base import RobotWalkingBase


class TemplateForRobotInterpolationWalking(RobotWalkingBase):

    DEBUG = False

    def __init__(self, hip_height, torso_angle, lift_off_slope, step_height, step_length, put_down_slope, duration_of_rest, walking_frequency, robot_server):
        super().__init__(hip_height, torso_angle, lift_off_slope, step_height, step_length, put_down_slope, walking_frequency, robot_server)

        self.duration_of_rest = duration_of_rest

        self.walking_sets = [{'is_swing_leg_left': True, 'step_length': step_length/2, 'current_position_relative_to_initial_foot_position': Point((0, 0, 0))},
                             {'is_swing_leg_left': False, 'step_length': step_length, 'current_position_relative_to_initial_foot_position': Point((-step_length/2, 0, 0))},
                             {'is_swing_leg_left': True, 'step_length': step_length, 'current_position_relative_to_initial_foot_position': Point((-step_length/2, 0, 0))}]

        self.timer = PerformanceTimer(interval=self.step_timer.frequency, duration=self.step_timer.single_step_time)

    def walk(self, sleep_time_after_squat):
        self.squat()
        sleep(sleep_time_after_squat)

        self.timer.start()
        self._take_steps(*self._get_is_swing_leg_left_and_walk_trajectory_four_positions__with_current_working_set(self.current_walking_set_index))

    def calculate_and_actuate_lower_limps_angle_set_at_good_timing(self, is_swing_leg_left, interpolated_position, time_fraction):

        lower_limps_angle_set = self._lower_limp_kinematics_engine.get_lower_limps_angle_set__with__both_tilted_torso_and_hip_offset_effect(interpolated_position, is_swing_leg_left)
        self._twist_waist_to_balance(lower_limps_angle_set, is_swing_leg_left, time_fraction)

        self.timer.sleep_until_next_interval()

        self.timer.record_time()

        self._actuate_lower_limps(*lower_limps_angle_set)