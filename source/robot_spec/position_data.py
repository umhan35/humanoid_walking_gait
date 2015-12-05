from robot_spec.length_data import *
from util.transformations import translate_3d_point

head_initial = (0, 0, torso_height + neck_length + head_height)
head_forward_bowed_90 = (head_height, 0, torso_height + neck_length)
head_backward_bowed_90 = (head_height, 0, torso_height + neck_length)
head_left_bowed_90 = (0, head_height, torso_height + neck_length)
head_right_bowed_90 = (0, -head_height, torso_height + neck_length)

left_shoulder_position = (0, half_torso_width, torso_height)
left_outer_shoulder_position = translate_3d_point(left_shoulder_position, dy=shoulder_length)

right_shoulder_position = (0, -half_torso_width, torso_height)
right_outer_shoulder_position = translate_3d_point(right_shoulder_position, dy=-shoulder_length)

left_hip_top_position = (0, half_torso_width, 0)
left_hip_bottom_position = (0, half_torso_width, -hip_length)

right_hip_top_position = (0, -half_torso_width, 0)
right_hip_bottom_position = (0, -half_torso_width, -hip_length)

initial_left_foot_position = (0, half_torso_width, initial_foot_z)
initial_right_foot_position = (0, -half_torso_width, initial_foot_z)
def get_initial_foot_position(is_left):
    if is_left:
        return initial_left_foot_position
    else:
        return initial_right_foot_position