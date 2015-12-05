import socket


class RobotClient:

    DEBUG = False

    def __init__(self, host, robot_name):
        self.host = host
        self.port = 1313

        if self.DEBUG: print('Connecting to {} robot robot_client_server ({})...'.format(robot_name, host))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((self.host, self.port))

    def close(self):
        self.sock.close()

    def _send_data(self, data):
        if self.DEBUG: print('Sending data: ' + repr(data))
        self.sock.sendall(data.encode())

    def send_and_receive_data(self, data):
        self._send_data(data)
        ret = self.sock.recv(1024)
        ret = repr(ret)
        if self.DEBUG: print('Received data: ', ret)
        return repr(ret)

    def cli(self):
        while True:
            data = input('> ')
            self._send_data(data)

    def read_state(self):
        return self.send_and_receive_data('Arash ReadState')

    def _actuate(self, actuator_names, actuator_angle_radians, speed=10):
        assert type(actuator_angle_radians) == list
        assert len(actuator_angle_radians) == len(actuator_names)

        for i, radians in enumerate(actuator_angle_radians):
            actuator_name = actuator_names[i]
            self._send_data(str(ActuatorCommand(actuator_name, radians, speed)))

    def actuate_neck(self, neck_radians_list, speed=10):
        neck_actuator_names = ["NeckTransversal", "NeckLateral"]
        self._actuate(neck_actuator_names, neck_radians_list, speed)

    def actuate_left_arm(self, left_arm_radians_list, speed=10):
        left_arm_actuator_names = ["LeftShoulderLateral", "LeftShoulderFrontal", "LeftElbowLateral"]
        self._actuate(left_arm_actuator_names, left_arm_radians_list, speed)

    def actuate_right_arm(self, right_arm_radians_list, speed=10):
        right_arm_actuator_names = ["RightShoulderLateral", "RightShoulderFrontal", "RightElbowLateral"]
        self._actuate(right_arm_actuator_names, right_arm_radians_list, speed)

    def actuate_left_lower_limp(self, left_lower_limp_radians_list, speed=10):
        left_lower_limp_actuator_names = ["LeftHipTransversal", "LeftHipFrontal", "LeftHipLateral", "LeftKneeLateral", "LeftAnkleLateral", "LeftAnkleFrontal"]
        self._actuate(left_lower_limp_actuator_names, left_lower_limp_radians_list, speed)

    def actuate_right_lower_limp(self, right_lower_limp_radians_list, speed=10):
        right_lower_limp_actuator_names = ["RightHipTransversal", "RightHipFrontal", "RightHipLateral", "RightKneeLateral", "RightAnkleLateral", "RightAnkleFrontal"]
        self._actuate(right_lower_limp_actuator_names, right_lower_limp_radians_list, speed)


class VirtualRobotClient(RobotClient):

    def __init__(self):
        super().__init__(host="localhost", robot_name='Virtual')


class RealRobotClient(RobotClient):

    def __init__(self):
        super().__init__(host='10.10.31.1', robot_name='Autman')


class ActuatorCommand:

    def __init__(self, name, angle, speed=10):
        self.name = name
        self.angle = str(angle)
        self.speed = speed

    def __str__(self):
        return 'Arash Actuator {} Position Angle {} Speed {}\n'.format(self.name, self.angle, self.speed)