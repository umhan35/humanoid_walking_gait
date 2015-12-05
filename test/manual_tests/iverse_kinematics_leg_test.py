__author__ = 'zhao'
from inverse_kinematics.lower_limp import InverseKinematicsLeg
from util.painter import Painter
from util.point import Point
from robot_model import RobotModel

is_left = False
# target_position = Point((-8.0, 7.25, -44.699999999999989))
target_position = Point((-8.0, 7.25, -44.7))

painter = Painter()
robot_model = RobotModel(painter)



r_hip_transversal_radians = 0
r_hip_frontal_radians, r_hip_lateral_radians, r_knee_lateral_radians, r_ankle_lateral_radians, r_ankle_frontal_radians\
    = InverseKinematicsLeg(is_left=is_left, hip_transversal_radians=r_hip_transversal_radians, target_position=target_position, robot_model=robot_model, painter=painter).get_angles()
# r_hip_frontal_radians, r_hip_lateral_radians, r_knee_lateral_radians, r_ankle_lateral_radians, r_ankle_frontal_radians = [0] * 5
# painter.draw_point(target_position)

robot_model.draw_neck_and_head()
robot_model.draw_torso()
robot_model.draw_left_arm()
robot_model.draw_right_arm()
robot_model.draw_left_lower_limp()
robot_model.draw_right_lower_limp(right_hip_transversal_radians=r_hip_transversal_radians, right_hip_frontal_radians=r_hip_frontal_radians, right_hip_lateral_radians=r_hip_lateral_radians, right_knee_lateral_radians=r_knee_lateral_radians, right_ankle_lateral_radians=r_ankle_lateral_radians, right_ankle_frontal_radians=r_ankle_frontal_radians)

robot_model.show()
