
class ReadStateParser:

    def __init__(self, state_string):
        self.state = state_string

    def get_all_joint_radians(self):

        current_all_joint_angles = {}

        for joint_state in self.state.split('&')[2:]:
            joint_name = joint_state.split(':')[0]
            joint_params = joint_state.split(':')[1]
            angle_param = joint_params.split(' ')[0]
            angle_in_radians = angle_param.split('=')[1]

            joint_name = convert_from_CamelCase_to_underscore(joint_name)

            current_all_joint_angles[joint_name] = float(angle_in_radians)

        return current_all_joint_angles

import re

def convert_from_CamelCase_to_underscore(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()