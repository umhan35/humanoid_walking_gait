from inverse_kinematics.lower_limp import InverseKinematicsLeg
from util.painter import Painter
from robot_spec.position_data import get_initial_foot_position
from util.point import Point
from robot_model import RobotModel
from walking.composition.upper_limb_while_walking import UpperLimbWhileWalking
from walking.composition.walk_trajectory import WalkTrajectory


class RobotWalkingLowerLimpKinematicsEngine:
    """
    Responsible for computing the control points on the walking trajectory
    """

    def __init__(self, hip_height, torso_angle, lift_off_slope, step_height, step_length, put_down_slope):
        self.hip_height = hip_height
        self.torso_angle = torso_angle
        self.lift_off_slope = lift_off_slope
        self.step_height = step_height
        self.step_length = step_length
        self.put_down_slope = put_down_slope

        self.upper_limp_part = UpperLimbWhileWalking(torso_angle, hip_height)
        self.hip_offset = self.upper_limp_part.hip_offset

        self.painter = Painter()
        self.robot_model = RobotModel(self.painter)

    def get_squat_lower_limps_angles_set__with_tilted_torso_effect(self):
        squat_left_lower_limp_angles = self._get_one_lower_limp_angles__with__both_tilted_torso_and_hip_offset_effect(get_initial_foot_position(is_left=True), is_left=True)
        squat_right_lower_limp_angles = self._get_one_lower_limp_angles__with__both_tilted_torso_and_hip_offset_effect(get_initial_foot_position(is_left=False), is_left=False)

        return squat_left_lower_limp_angles, squat_right_lower_limp_angles

    def get_walk_trajectory_three_positions(self, is_swing_leg_left, step_length, current_position_relative_to_initial_foot_position):
        """
        when ``step_length`` can be set to half step length, the robot only take half step.
        """

        ratio = step_length / self.step_length

        walk_trajectory = WalkTrajectory(self.hip_offset, self.lift_off_slope / ratio, self.step_height, step_length, self.put_down_slope / ratio)

        lift_up_front_position = walk_trajectory.get_lift_front_position(is_left=is_swing_leg_left, foot_position_relative_to=current_position_relative_to_initial_foot_position)

        lift_up_front_forward_position = walk_trajectory.get_lift_up_front_forward_position(lift_up_front_position)

        put_down_position = walk_trajectory.get_put_down_position(lift_up_front_forward_position)

        return lift_up_front_position, lift_up_front_forward_position, put_down_position

    def get_feet_positions_set__with__both_tilted_torso_and_hip_offset_effect(self, is_swing_leg_left, step_length, current_position_relative_to_initial_foot_position):
        """
        when ``step_length`` is set to half step length, the robot only take half step.
        """

        lift_up_front_position, lift_up_front_forward_position, put_down_position\
            = self.get_walk_trajectory_three_positions(is_swing_leg_left, step_length, current_position_relative_to_initial_foot_position)

        lower_limps_angle_set_for_lift_up_front_position = self.get_lower_limps_angle_set__with__both_tilted_torso_and_hip_offset_effect(swing_leg_foot_position=lift_up_front_position, is_swing_leg_left=is_swing_leg_left)
        lower_limps_angle_set_for_lift_up_front_forward_position = self.get_lower_limps_angle_set__with__both_tilted_torso_and_hip_offset_effect(swing_leg_foot_position=lift_up_front_forward_position, is_swing_leg_left=is_swing_leg_left)
        lower_limps_angle_set_for_put_down_position = self.get_lower_limps_angle_set__with__both_tilted_torso_and_hip_offset_effect(swing_leg_foot_position=put_down_position, is_swing_leg_left=is_swing_leg_left)

        return (lower_limps_angle_set_for_lift_up_front_position, lower_limps_angle_set_for_lift_up_front_forward_position, lower_limps_angle_set_for_put_down_position)

    def get_lower_limps_angle_set__with__both_tilted_torso_and_hip_offset_effect(self, swing_leg_foot_position, is_swing_leg_left):

            swing_lower_limp_angles = self._get_one_lower_limp_angles__with__both_tilted_torso_and_hip_offset_effect(swing_leg_foot_position, is_left=is_swing_leg_left)
            support_lower_limp_angles = self._get_support_lower_limp_angles(is_support_leg_left=not is_swing_leg_left, swing_leg_foot_position=swing_leg_foot_position)

            return self._convert_swing_and_support_lower_limps_to_left_and_right(is_swing_leg_left, swing_lower_limp_angles, support_lower_limp_angles)

    def _convert_swing_and_support_lower_limps_to_left_and_right(self, is_swing_leg_left, swing_lower_limp_angles, support_lower_limp_angles):
        if is_swing_leg_left:
            return [swing_lower_limp_angles, support_lower_limp_angles]
        else:
            return [support_lower_limp_angles, swing_lower_limp_angles]

    def _get_support_lower_limp_angles(self, is_support_leg_left, swing_leg_foot_position):
        opposite_ground_position = WalkTrajectory.project_to_opposite_ground(Point(swing_leg_foot_position), is_left=is_support_leg_left)
        support_lower_limp_angles = self._get_one_lower_limp_angles__with__both_tilted_torso_and_hip_offset_effect(opposite_ground_position, is_left=is_support_leg_left)

        return support_lower_limp_angles

    def _get_one_lower_limp_angles__with__both_tilted_torso_and_hip_offset_effect(self, position, is_left):
        """
        Returns a new position and lower limp angles
        """
        position_with_hip_offset = self. upper_limp_part.add_hip_offset_effect(position)

        lower_limp_angles_without_tilted_torso_effect = self._get_lower_limp_angles(position_with_hip_offset, is_left=is_left)
        lower_limp_angles = self.upper_limp_part.add_tilted_torso_effect(lower_limp_angles_without_tilted_torso_effect, is_left=is_left)

        return lower_limp_angles

    def _get_lower_limp_angles(self, target_position, is_left):

        lower_limp_angles_except_hip_transversal = InverseKinematicsLeg(is_left=is_left, hip_transversal_radians=0, target_position=target_position, robot_model=self.robot_model, painter=self.painter).get_angles()
        lower_limp_angles = tuple([0] + list(lower_limp_angles_except_hip_transversal))

        return lower_limp_angles
