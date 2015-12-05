import socketserver
from threading import Thread

from matplotlib.animation import FuncAnimation
from center_of_mass import compute_center_of_mass_given_joints

from util.painter import Painter
from robot_model import RobotModel
from robot_client_server.robot_server_message import RobotServerMessage


class RobotMemory:
    left_lower_limp_angles = [0, 0, 0, 0, 0, 0]
    right_lower_limp_angles = [0, 0, 0, 0, 0, 0]

    left_arm_joints = []
    right_arm_joints = []
    torso_corner_positions = []
    left_lower_limp_joints = []
    right_lower_limp_joints = []


class VirtualRobotRequestHandler(socketserver.BaseRequestHandler):

    lower_limp_angle_name_index_map = {'HipTransversal': 0, 'HipFrontal': 1, 'HipLateral': 2, 'KneeLateral': 3, 'AnkleLateral': 4, 'AnkleFrontal': 5}

    def handle(self):
        message = self.request[0].strip()
        client_host = self.client_address[0]

        # print(client_host, message)
        self.set_lower_limp_angle_from_message(message)

    def set_lower_limp_angle_from_message(self, message):
        robot_server_message = RobotServerMessage(message)
        angle_index = self.lower_limp_angle_name_index_map[robot_server_message.undirected_joint_name]

        if robot_server_message.is_joint_left:
            angles = RobotMemory.left_lower_limp_angles
        else:
            angles = RobotMemory.right_lower_limp_angles

        angles[angle_index] = robot_server_message.joint_angle


def start_robot_server_thread():

    robot_server_host = "localhost"
    robot_server_port = 1313

    udp_server = socketserver.UDPServer((robot_server_host, robot_server_port), VirtualRobotRequestHandler)
    Thread(target=udp_server.serve_forever).start()

if __name__ == '__main__':
    start_robot_server_thread()

    painter = Painter(azimuth=-30, elevation=60)
    robot_model = RobotModel(painter)

    # robot_model.draw_initial_pose()
    robot_model.draw_neck_and_head()
    RobotMemory.left_arm_joints = robot_model.draw_left_arm()
    RobotMemory.right_arm_joints = robot_model.draw_right_arm()
    RobotMemory.torso_corner_positions = robot_model.draw_torso()

    def func(i):
        # Note that draw_left/right_lower_limp() is called two times.
        #   The param `draw` is false for the first time, which makes the func only do math work.
        #   If we want super-high performance, refactor those 2 funcs.

        RobotMemory.left_lower_limp_joints = robot_model.draw_left_lower_limp(*RobotMemory.left_lower_limp_angles, draw=False)
        RobotMemory.right_lower_limp_joints = robot_model.draw_right_lower_limp(*RobotMemory.right_lower_limp_angles, draw=False)

        left_lower_limp_lines = robot_model.draw_left_lower_limp(*RobotMemory.left_lower_limp_angles, get_lines=True)
        right_lower_limp_lines = robot_model.draw_right_lower_limp(*RobotMemory.right_lower_limp_angles, get_lines=True)

        center_of_mass = compute_center_of_mass_given_joints(
                                                         RobotMemory.left_arm_joints[1], RobotMemory.left_arm_joints[2], RobotMemory.left_arm_joints[3],
                                                         RobotMemory.right_arm_joints[1], RobotMemory.right_arm_joints[2], RobotMemory.right_arm_joints[3],
                                                         RobotMemory.left_lower_limp_joints[0], RobotMemory.left_lower_limp_joints[3], RobotMemory.left_lower_limp_joints[6],
                                                         RobotMemory.right_lower_limp_joints[0], RobotMemory.right_lower_limp_joints[3], RobotMemory.right_lower_limp_joints[6],
                                                         *RobotMemory.torso_corner_positions)

        plotted_center_of_mass = painter.draw_point(center_of_mass)

        return list(left_lower_limp_lines) + list(right_lower_limp_lines) + list(plotted_center_of_mass)

    FuncAnimation(painter.fig, init_func=lambda: [], func=func, blit=True, interval=100, save_count=1)

    painter.show()
