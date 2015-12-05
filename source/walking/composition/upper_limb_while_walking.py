from joint.joint_angle import LeftHipLateral, RightHipLateral
from robot_spec.length_data import initial_hip_height
from util.point import Point


class UpperLimbWhileWalking:

    def __init__(self, torso_angle, hip_height):
        self.torso_angle = torso_angle
        self.hip_height = hip_height

        self.hip_offset = initial_hip_height - self.hip_height

    def add_tilted_torso_effect(self, lower_limp_angles, is_left):
        """
        basically add hip lateral
        """

        lower_limp_angles = list(lower_limp_angles)

        tilted_torso_left_hip_lateral, tilted_torso_right_hip_lateral = self.get_hip_laterals_while_torso_is_tilted()

        hip_lateral_index = 2

        if is_left:
            lower_limp_angles[hip_lateral_index] += tilted_torso_left_hip_lateral
        else:
            lower_limp_angles[hip_lateral_index] += tilted_torso_right_hip_lateral

        return lower_limp_angles

    def add_hip_offset_effect(self, position):
        return Point(position).translate(dz=self.hip_offset)

    def get_hip_laterals_while_torso_is_tilted(self):
        left_hip_lateral = LeftHipLateral().forward(self.torso_angle).angle
        right_hip_lateral = RightHipLateral().forward(self.torso_angle).angle

        return left_hip_lateral, right_hip_lateral
