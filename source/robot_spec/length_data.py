# all in cm (centimeter)

# torso
torso_width = 14.5
torso_height = 15.8
half_torso_width = torso_width / 2



# head
neck_length = 2.8
head_height = 8.5
head_width = 9.4
half_head_width = head_width / 2



# arms
shoulder_length = 2
upper_arm_length = 14.6
lower_arm_length = 29
arm_length = upper_arm_length + lower_arm_length
lower_and_upper_arm_difference = abs(lower_arm_length - upper_arm_length)

outer_shoulder_x_offset = 0
outer_shoulder_y_offset = half_torso_width + shoulder_length
outer_shoulder_z_offset = torso_height
shoulder_y_offset = half_torso_width
shoulder_z_offset = torso_height



# lower limps
hip_length = 6
upper_leg_length = 17.5
lower_leg_length = 20.5
leg_length = upper_leg_length + lower_leg_length
foot_height = 3.7
foot_width = 11
foot_length = 18
initial_hip_height = hip_length + leg_length + foot_height
initial_foot_z = -initial_hip_height
half_foot_length = foot_length / 2
half_foot_width = foot_width / 2

initial_left_leg_y = half_torso_width
initial_right_leg_y = -half_torso_width
def get_initial_leg_y(is_left):
    if is_left:
        return initial_left_leg_y
    else:
        return initial_right_leg_y
