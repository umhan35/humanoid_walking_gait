from math import radians
from joint.joint_angle import *
from robot_client_server.robot_clients import RealRobotClient, VirtualRobotClient
from util.pi import half_pi


def actuate_to_natural_pose(real_robot_client):

    real_robot_client.actuate_neck([0, 0])
    real_robot_client.actuate_left_arm([0, half_pi, -half_pi])
    real_robot_client.actuate_right_arm([0, -half_pi, half_pi])
    real_robot_client.actuate_left_lower_limp([0, LeftHipFrontal().outward(1).radians, 0, 0, 0, 0])
    real_robot_client.actuate_right_lower_limp([0, RightHipFrontal().outward(1).radians, 0, 0, 0, 0])


def f():
    # c._send_data('Arash PowerOn')
    c._actuate(['RightKneeLateral'], [RightKneeLateral().forward(10).radians])
    pass


c = RealRobotClient()
actuate_to_natural_pose(c)
# f()
