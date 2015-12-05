

class RobotServerMessage:

    def __init__(self, message):
        self._raw_message = message

        message_tokens = str(message).split(' ')

        self._received_joint_name = message_tokens[2]

        self.is_joint_left = 'Left' in self._received_joint_name

        if self.is_joint_left:
            self.undirected_joint_name = self._received_joint_name.replace('Left', '')
        else:
            self.undirected_joint_name = self._received_joint_name.replace('Right', '')

        self.joint_angle = float(message_tokens[5])


if __name__ == '__main__':
    msg = RobotServerMessage('Arash Actuator LeftAnkleLateral Position Angle 0.26965686740571804 Speed 20\n')