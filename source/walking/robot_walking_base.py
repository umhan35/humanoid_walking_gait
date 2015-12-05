from joint.joint_angle import RightHipFrontal, LeftHipFrontal
from util.point import Point
from robot_spec.position_data import get_initial_foot_position
from walking.helpers.bezier_curves import LineSegment
from walking.helpers.step_timer import StepTimer
from walking.composition.robot_walking_lower_limp_kinematics_engine import RobotWalkingLowerLimpKinematicsEngine


class RobotWalkingBase:
    """
    Responsible for actuate the robot lower limp & real robot walking process.
    By contrast, `RobotWalkingLowerLimpKinematicsEngine` class is low level.
    """

    DEBUG = False

    lower_limp_speed = 80

    hip_frontal_offset_angle_radians = 0.1
    balanced_hip_frontal_degrees = 2

    def __init__(self, hip_height, torso_angle, lift_off_slope, step_height, step_length, put_down_slope, walking_frequency, robot_server):
        self._lower_limp_kinematics_engine = RobotWalkingLowerLimpKinematicsEngine(hip_height, torso_angle, lift_off_slope, step_height, step_length, put_down_slope)

        self.robot_server = robot_server

        # set up walking parameter sets
        self.walking_sets = None
        self.current_walking_set_index = 0

        if self.DEBUG: print('walking_frequency', walking_frequency)

        self.step_timer = StepTimer(walking_frequency)

    def squat(self):
        if self.DEBUG: print('squat')
        self._actuate_lower_limps(*self._lower_limp_kinematics_engine.get_squat_lower_limps_angles_set__with_tilted_torso_effect())

    def _actuate_lower_limps(self, left_lower_limp_angles, right_lower_limp_angles):
        self._counter_balance_hip_frontal_servo_offset(left_lower_limp_angles, right_lower_limp_angles)

        self.robot_server.actuate_left_lower_limp(left_lower_limp_angles, speed=self.lower_limp_speed)
        self.robot_server.actuate_right_lower_limp(right_lower_limp_angles, speed=self.lower_limp_speed)

    def _twist_waist_to_balance(self, lower_limps_angle_set, is_swing_leg_left, time_fraction):
        range_percentage = abs(0.5 - time_fraction)

        if is_swing_leg_left:
            t = RightHipFrontal().outward(self.balanced_hip_frontal_degrees).radians * range_percentage  # right leg
            lower_limps_angle_set[1][1] += t
        else:
            t = LeftHipFrontal().outward(self.balanced_hip_frontal_degrees).radians * range_percentage  # left leg
            lower_limps_angle_set[0][1] += t

    def _counter_balance_hip_frontal_servo_offset(self, left_lower_limp_angles, right_lower_limp_angles):
        # hip frontal outwards
        left_lower_limp_angles[1] += -self.hip_frontal_offset_angle_radians
        right_lower_limp_angles[1] += self.hip_frontal_offset_angle_radians

        # ankle frontal inwards
        left_lower_limp_angles[5] += -self.hip_frontal_offset_angle_radians
        right_lower_limp_angles[5] += self.hip_frontal_offset_angle_radians

    def set_current_walking_set_index_to_next(self):

        self.current_walking_set_index += 1
        if self.DEBUG: print('current_walking_set_index', self.current_walking_set_index)

        if self.current_walking_set_index == 3:
            self.current_walking_set_index = 1

        if self.DEBUG: print('_current_walking_set_index', self.current_walking_set_index)

    def _get_is_swing_leg_left_and_walk_trajectory_four_positions__with_current_working_set(self, index):

        current_walking_set = self.walking_sets[index]

        is_swing_leg_left = current_walking_set['is_swing_leg_left']
        step_length = current_walking_set['step_length']
        current_position_relative_to_initial_foot_position = current_walking_set['current_position_relative_to_initial_foot_position']

        initial_foot_position = Point(get_initial_foot_position(is_left=is_swing_leg_left))

        initial_position = initial_foot_position.translate(current_position_relative_to_initial_foot_position.x, current_position_relative_to_initial_foot_position.y, current_position_relative_to_initial_foot_position.z)
        lift_up_front_position, lift_up_front_forward_position, put_down_position\
            = self._lower_limp_kinematics_engine.get_walk_trajectory_three_positions(is_swing_leg_left=is_swing_leg_left, step_length=step_length, current_position_relative_to_initial_foot_position=current_position_relative_to_initial_foot_position)

        return is_swing_leg_left, initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position

    @staticmethod
    def _get_3_walking_trajectory_sections(initial_position, lift_up_front_position, lift_up_front_forward_position, put_down_position):
        lift_up_front_trajectory_section = LineSegment(initial_position, lift_up_front_position)
        lift_up_front_forward_trajectory_section = LineSegment(lift_up_front_position, lift_up_front_forward_position)
        put_down_trajectory_section = LineSegment(lift_up_front_forward_position, put_down_position)

        return lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section
