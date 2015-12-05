from geometric_link import GeometricLink
from robot_spec import mass_data
from statistics import mean


def compute_center_of_mass_given_joints(
                                        left_shoulder_lateral, left_elbow_lateral, left_wrist,
                                        right_shoulder_lateral, right_elbow_lateral, right_wrist,
                                        left_hip_transversal, left_knee_lateral, left_foot_center,
                                        right_hip_transversal, right_knee_lateral, right_foot_center,
                                        right_shoulder_position, left_shoulder_position, left_hip_top_position, right_hip_top_position):

    left_upper_arm_link = GeometricLink(left_shoulder_lateral.vertex, left_elbow_lateral.vertex, mass_data.upper_arm)
    left_lower_arm_link = GeometricLink(left_elbow_lateral.vertex, left_wrist, mass_data.lower_arm)

    right_upper_arm_link = GeometricLink(right_shoulder_lateral.vertex, right_elbow_lateral.vertex, mass_data.upper_arm)
    right_lower_arm_link = GeometricLink(right_elbow_lateral.vertex, right_wrist, mass_data.lower_arm)

    left_upper_leg_link = GeometricLink(left_hip_transversal.vertex, left_knee_lateral.vertex, mass_data.upper_leg)
    left_lower_leg_link = GeometricLink(left_knee_lateral.vertex, left_foot_center, mass_data.loweer_leg)

    right_upper_leg_link = GeometricLink(right_hip_transversal.vertex, right_knee_lateral.vertex, mass_data.upper_leg)
    right_lower_leg_link = GeometricLink(right_knee_lateral.vertex, right_foot_center, mass_data.loweer_leg)

    def get_torso_position_x_mass():
        torso_four_points = (right_shoulder_position, left_shoulder_position, left_hip_top_position, right_hip_top_position)
        x = mean([each[0] for each in torso_four_points])
        y = mean([each[1] for each in torso_four_points])
        z = mean([each[2] for each in torso_four_points])
        return [x, y, z]

    position_x_mass_list = [get_torso_position_x_mass()] + [each.position_x_mass for each in [left_upper_arm_link, left_lower_arm_link, right_upper_arm_link, right_lower_arm_link, left_upper_leg_link, left_lower_leg_link, right_upper_leg_link, right_lower_leg_link]]
    # print(position_x_mass_list)

    center_of_mass_x = sum([each[0] for each in position_x_mass_list]) / mass_data.total
    center_of_mass_y = sum([each[1] for each in position_x_mass_list]) / mass_data.total
    # center_of_mass_z = sum([each[2] for each in position_x_mass_list]) / mass_data.total

    left_foot_z = left_foot_center[2]
    right_foot_z = right_foot_center[2]
    center_of_mass_z = min(left_foot_z, right_foot_z)

    center_of_mass = (center_of_mass_x, center_of_mass_y, center_of_mass_z)

    return center_of_mass


def compute_center_of_mass_from_read_state_string(angles_from_read_state_string, painter, robot_model, draw_robot=False, draw_center_of_mass=False):
    angles = angles_from_read_state_string

    if draw_robot:
        robot_model.draw_neck_and_head(neck_transversal_radians=angles['neck_transversal'], neck_lateral_radians=angles['neck_lateral'])
    left_shoulder_lateral, left_shoulder_frontal, left_elbow_lateral, left_wrist\
        = robot_model.draw_left_arm(left_shoulder_lateral_radians=angles['left_shoulder_lateral'], left_shoulder_frontal_radians=angles['left_shoulder_frontal'], left_elbow_lateral_radians=angles['left_elbow_lateral'], draw=draw_robot)
    right_shoulder_lateral, right_shoulder_frontal, right_elbow_lateral, right_wrist\
        = robot_model.draw_right_arm(right_shoulder_lateral_radians=angles['right_shoulder_lateral'], right_shoulder_frontal_radians=angles['right_shoulder_frontal'], right_elbow_lateral_radians=angles['right_elbow_lateral'], draw=draw_robot)
    left_hip_transversal, left_hip_frontal, left_hip_lateral, left_knee_lateral, left_ankle_lateral, left_ankle_frontal, left_foot_center\
        = robot_model.draw_left_lower_limp(left_hip_transversal_radians=angles['left_hip_transversal'], left_hip_frontal_radians=angles['left_hip_frontal'], left_hip_lateral_radians=angles['left_hip_lateral'], left_knee_lateral_radians=angles['left_knee_lateral'], left_ankle_lateral_radians=angles['left_ankle_lateral'], left_ankle_frontal_radians=angles['left_ankle_frontal'], draw=draw_robot)
    right_hip_transversal, right_hip_frontal, right_hip_lateral, right_knee_lateral, right_ankle_lateral, right_ankle_frontal, right_foot_center\
        = robot_model.draw_right_lower_limp(right_hip_transversal_radians=angles['right_hip_transversal'], right_hip_frontal_radians=angles['right_hip_frontal'], right_hip_lateral_radians=angles['right_hip_lateral'], right_knee_lateral_radians=angles['right_knee_lateral'], right_ankle_lateral_radians=angles['right_ankle_lateral'], right_ankle_frontal_radians=angles['right_ankle_frontal'], draw=draw_robot)
    right_shoulder_position, left_shoulder_position, left_hip_top_position, right_hip_top_position\
        = robot_model.draw_torso(draw=draw_robot)

    center_of_mass = compute_center_of_mass_given_joints(
                                                        left_shoulder_lateral, left_elbow_lateral, left_wrist,
                                                        right_shoulder_lateral, right_elbow_lateral, right_wrist,
                                                        left_hip_transversal, left_knee_lateral, left_foot_center,
                                                        right_hip_transversal, right_knee_lateral, right_foot_center,
                                                        right_shoulder_position, left_shoulder_position, left_hip_top_position, right_hip_top_position)

    if draw_center_of_mass:
        painter.draw_point(center_of_mass)

    return center_of_mass