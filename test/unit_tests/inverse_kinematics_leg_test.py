import unittest

from robot_model import RobotModel
from util.painter import Painter
from util.angle import to_radians
from inverse_kinematics.lower_limp import InverseKinematicsLeg
from util.point import Point
from util.transformations import translate_3d_point
from util.angle import angle_range
from robot_spec.length_data import foot_height


class TestLegInverseKinematics(unittest.TestCase):

    def setUp(self):
        self.painter = Painter()
        self.r = RobotModel(Painter())

    def assertLowerLimpAnglesEqual(self, leg_angles, is_left=True):
        """
        Given shoulder and elbow angles A for arm, calculate the end points P,
        and call inverse kinematics (IK) with P to see if IK returns A exactly.
        :param arm_angles:
        :return:
        """
        print('\n'*6 + 'aAE', leg_angles)

        lower_limp_angles = to_radians(leg_angles) + (0, 0)

        draw_lower_limp = self.r.draw_left_lower_limp if is_left else self.r.draw_right_lower_limp

        expected_hip_transversal, expected_hip_frontal, expected_hip_lateral, \
        expected_knee_lateral, \
        expected_ankle_lateral, expected_ankle_frontal, \
        expected_foot_center \
            = draw_lower_limp(*lower_limp_angles, color='r-')
        expected_ankle = Point(expected_ankle_frontal.vertex)

        target = translate_3d_point(expected_ankle, dz=-foot_height)

        hip_transversal_radians = lower_limp_angles[0]
        hip_frontal_radians, hip_lateral_radians, knee_lateral_radians, ankle_lateral_radians, ankle_frontal_radians \
            = InverseKinematicsLeg(hip_transversal_radians, target, self.r, self.painter, is_left).get_angles()

        actual_hip_transversal, actual_hip_frontal, actual_hip_lateral, \
        actual_knee_lateral, \
        actual_ankle_lateral, actual_ankle_frontal, \
        actual_foot_center \
            = draw_lower_limp(hip_transversal_radians, hip_frontal_radians, hip_lateral_radians, knee_lateral_radians, ankle_lateral_radians, ankle_frontal_radians, draw=False)
        actual_ankle =Point(actual_ankle_frontal.vertex)
        actual_foot_center = Point(actual_foot_center)

        self.assertTrue(actual_ankle.float_equals(expected_ankle), [actual_ankle, expected_ankle])
        self.assertTrue(actual_ankle.translate(dz=-foot_height).float_equals(actual_foot_center))

    def test_leg_initial_position(self):
        self.assertLowerLimpAnglesEqual((0, 0, 0, 0), is_left=True)
        self.assertLowerLimpAnglesEqual((0, 0, 0, 0), is_left=False)

    def test_hip_frontal(self):
        for angle in angle_range(0, 90):
            self.assertLowerLimpAnglesEqual((0, angle, 0, 0), is_left=True)
        for angle in angle_range(0, -90):
            self.assertLowerLimpAnglesEqual((0, angle, 0, 0), is_left=False)

    def test_hip_lateral(self):
        for angle in angle_range(0, -90):
            self.assertLowerLimpAnglesEqual((0, 0, angle, 0), is_left=True)
        for angle in angle_range(0, 90):
            self.assertLowerLimpAnglesEqual((0, 0, angle, 0), is_left=False)

    def test_knee_lateral(self):
        for angle in angle_range(0, 180):
            self.assertLowerLimpAnglesEqual((0, 0, 0, angle), is_left=True)
        for angle in angle_range(0, -180):
            self.assertLowerLimpAnglesEqual((0, 0, 0, angle), is_left=False)

    def test_all_left(self):
        step = 30
        
        for hip_transversal in angle_range(0, -90, step=step):
            for hip_frontal in angle_range(0, 90, step=step):
                for hip_lateral in angle_range(0, -90, step=step):
                    for knee_lateral in angle_range(0, 180, step=step):
                        self.assertLowerLimpAnglesEqual((hip_transversal, hip_frontal, hip_lateral, knee_lateral), is_left=True)

    def test_all_right(self):
        step = 30

        for hip_transversal in angle_range(0, -90, step=step):
            for hip_frontal in angle_range(0, -90, step=step):
                for hip_lateral in angle_range(0, 90, step=step):
                    for knee_lateral in angle_range(0, -180, step=step):
                        self.assertLowerLimpAnglesEqual((hip_transversal, hip_frontal, hip_lateral, knee_lateral), is_left=False)

if __name__ == '__main__':
    unittest.main()