import unittest

from inverse_kinematics.arm import InverseKinematicsArm
from robot_model import RobotModel
from util.painter import Painter
from util.float import is_float_equal
from util.angle import to_radians, angle_range


class TestArmInverseKinematics(unittest.TestCase):

    def setUp(self):
        self.painter = Painter()
        self.r = RobotModel(Painter())

    def assertArmAnglesEqual(self, arm_angles, is_left=True):
        """
        Given shoulder and elbow angles A for arm, calculate the end points P,
        and call inverse kinematics (IK) with P to see if IK returns A exactly.
        :param arm_angles:
        :return:
        """
        # print('\n'*6 + 'aAE', arm_angles)

        arm_angles = to_radians(arm_angles)

        draw_arm = self.r.draw_left_arm if is_left else self.r.draw_right_arm
        expected_shoulder_lateral, expected_shoulder_frontal, expected_elbow_lateral, expected_wrist\
            = draw_arm(arm_angles[0], arm_angles[1], arm_angles[2], draw=False)

        def is_target_behind_body():
            target_x = expected_wrist[0]
            return not is_float_equal(target_x, 0) and target_x < 0

        if is_target_behind_body():
            return

        shoulder_lateral_radians, shoulder_frontal_radians, elbow_lateral_radians\
            = InverseKinematicsArm(expected_wrist, self.r, self.painter, is_left=is_left).get_angles()

        actual_shoulder_lateral, actual_shoulder_frontal, actual_elbow_lateral, actual_wrist\
            = draw_arm(shoulder_lateral_radians, shoulder_frontal_radians, elbow_lateral_radians, draw=False)

        for i in [0, 1, 2]:  # x, y, z
            self.assertAlmostEqual(expected_wrist[i], actual_wrist[i])

    def test_arm_initial_position(self):
        self.assertArmAnglesEqual((0, 0, 0), is_left=True)
        self.assertArmAnglesEqual((0, 0, 0), is_left=False)

    # expected to fail... because shoulder lateral should always be 0 when the shoulder frontal and elbow lateral are 0
    # def test_shoulder_lateral(self):
    #     for angle in angle_range(-90, 90):
    #         self.assertArmAnglesEqual((angle, 0, 0))

    def test_shoulder_frontal(self):
        for angle in angle_range(-90, 90):
            self.assertArmAnglesEqual((0, angle, 0), is_left=True)
            self.assertArmAnglesEqual((0, angle, 0), is_left=False)

    def test_elbow(self):
        for angle in angle_range(-180, 0):
            self.assertArmAnglesEqual((0, 0, angle), is_left=True)
            self.assertArmAnglesEqual((0, 0, angle), is_left=False)

    def test_multi_DoF_together(self, step=10):
        for shoulder_frontal in angle_range(0, 90, step=step):
            print(shoulder_frontal)
            for shoulder_lateral in angle_range(-90, 90, step=step):
                # print(shoulder_frontal, shoulder_lateral)
                for elbow_lateral in angle_range(-180, 0, step=step):
                    # print(shoulder_frontal, shoulder_lateral, elbow_lateral)
                    self.assertArmAnglesEqual((shoulder_frontal, shoulder_lateral, elbow_lateral), is_left=True)
                    # print(-shoulder_frontal, -shoulder_lateral, -elbow_lateral)
                    self.assertArmAnglesEqual((-shoulder_frontal, -shoulder_lateral, -elbow_lateral), is_left=False)


if __name__ == '__main__':
    unittest.main()