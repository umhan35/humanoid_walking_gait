from numpy import pi

from util.painter import Painter
from robot_model import RobotModel
from robot_client_server.read_state_parser import ReadStateParser
from robot_client_server.robot_clients import RealRobotClient


def ticks_to_radiance(n):
    # n ticks / 4095 = x radiance / 2π

    ticks_for_360_degrees = 4095
    return (n / ticks_for_360_degrees) * (2 * pi)


def forward_kinematics(robot_name=None, read_state_string=None):

    assert robot_name is not None or read_state_string is not None

    if read_state_string is None:
        read_state_string = RealRobotClient().read_state()
    assert 'nan' not in read_state_string

    r = RobotModel(Painter())

    angles = ReadStateParser(read_state_string).get_all_joint_radians()

    r.draw_neck_and_head(neck_transversal_radians=angles['neck_transversal'], neck_lateral_radians=angles['neck_lateral'])
    r.draw_left_arm(left_shoulder_lateral_radians=angles['left_shoulder_lateral'], left_shoulder_frontal_radians=angles['left_shoulder_frontal'], left_elbow_lateral_radians=angles['left_elbow_lateral'])
    r.draw_right_arm(right_shoulder_lateral_radians=angles['right_shoulder_lateral'], right_shoulder_frontal_radians=angles['right_shoulder_frontal'], right_elbow_lateral_radians=angles['right_elbow_lateral'])
    r.draw_left_lower_limp(left_hip_transversal_radians=angles['left_hip_transversal'], left_hip_frontal_radians=angles['left_hip_frontal'], left_hip_lateral_radians=angles['left_hip_lateral'], left_knee_lateral_radians=angles['left_knee_lateral'], left_ankle_lateral_radians=angles['left_ankle_lateral'], left_ankle_frontal_radians=angles['left_ankle_frontal'])
    r.draw_right_lower_limp(right_hip_transversal_radians=angles['right_hip_transversal'], right_hip_frontal_radians=angles['right_hip_frontal'], right_hip_lateral_radians=angles['right_hip_lateral'], right_knee_lateral_radians=angles['right_knee_lateral'], right_ankle_lateral_radians=angles['right_ankle_lateral'], right_ankle_frontal_radians=angles['right_ankle_frontal'])

    r.draw_torso()
    r.draw_coordinates()  # draw coordinates lastly because we want them on top layer
    r.show()

if __name__ == '__main__':
    # forward_kinematics(name='Arash')
    forward_kinematics(read_state_string="Ok&Arash&LeftShoulderLateral:angle=-0.171713 speed=0 load=0&LeftShoulderFrontal:angle=0.957297 speed=0 load=0&LeftElbowLateral:angle=-0.730082 speed=0 load=0&RightShoulderLateral:angle=0.0767918 speed=0 load=0&RightShoulderFrontal:angle=-1.09057 speed=0 load=0&RightElbowLateral:angle=0.897471 speed=0 load=0&NeckTransversal:angle=1.51567 speed=0 load=0&NeckLateral:angle=0.409666 speed=0 load=0&LeftHipTransversal:angle=0.138151 speed=0 load=0&LeftHipFrontal:angle=-0.029053 speed=0 load=0&LeftHipLateral:angle=-1.52162 speed=0 load=0&LeftKneeLateral:angle=-0.187053 speed=0 load=0&LeftAnkleLateral:angle=-0.253014 speed=0 load=0&LeftAnkleFrontal:angle=-0.025985 speed=0 load=0&RightHipTransversal:angle=0.167297 speed=0 load=0&RightHipFrontal:angle=-0.0152471 speed=0 load=0&RightHipLateral:angle=1.39141 speed=0 load=0&RightKneeLateral:angle=0.265471 speed=0 load=0&RightAnkleLateral:angle=1.6706 speed=0 load=0&RightAnkleFrontal:angle=-0.0152471 speed=0 load=0")
    # forward_kinematics(read_state_string="Arash&Ok&LeftShoulderLateral:angle=0.504772 speed=0 load=0&LeftShoulderFrontal:angle=1.15671 speed=0 load=0&LeftElbowLateral:angle=-1.61672 speed=0 load=0&RightShoulderLateral:angle=-0.297499 speed=0 load=0&RightShoulderFrontal:angle=-1.31299 speed=0 load=0&RightElbowLateral:angle=1.65986 speed=0 load=0&NeckTransversal:angle=9.27448e-05 speed=0 load=0&NeckLateral:angle=9.27448e-05 speed=0 load=0&LeftHipTransversal:angle=-0.23614 speed=0 load=0&LeftHipFrontal:angle=-0.0336549 speed=0 load=0&LeftHipLateral:angle=-1.44032 speed=0 load=0&LeftKneeLateral:angle=-0.194723 speed=0 load=0&LeftAnkleLateral:angle=-1.64894 speed=0 load=0&LeftAnkleFrontal:angle=-0.00757718 speed=0 load=0&RightHipTransversal:angle=0.627491 speed=0 load=0&RightHipFrontal:angle=-0.0167811 speed=0 load=0&RightHipLateral:angle=1.42209 speed=0 load=0&RightKneeLateral:angle=0.234792 speed=0 load=0&RightAnkleLateral:angle=1.65986 speed=0 load=0&RightAnkleFrontal:angle=-0.0167811 speed=0 load=0")