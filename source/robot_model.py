import numpy as np

from util.pi import pi, half_pi
from util.transformations import get_translate_matrix, get_rotate_matrix_along_axis, apply_matrix_to_origin
from joint import Joint
from robot_spec.length_data import *
from robot_spec.position_data import right_shoulder_position, left_shoulder_position, left_hip_top_position, right_hip_top_position


class RobotModel:

    def __init__(self, painter):
        self._painter = painter

        self.last_drew_right_lower_limp = {}

    @staticmethod
    def _get_torso_corner_transformation(translation_vector, rotate_along, rotate_radians):
        r = get_rotate_matrix_along_axis(rotate_along, rotate_radians)
        t = get_translate_matrix(*translation_vector)
        transformation = t.dot(r)

        return transformation

    def draw_link(self, joint, color='c-'):
        return self._painter.draw_line(joint.link[0].vertex, joint.link[1].vertex, color=color)

    def _draw_head(self, head_t):
        head_bottom = apply_matrix_to_origin(head_t)

        head_top_t = head_t.dot(get_translate_matrix(0, head_height, 0))
        head_top = apply_matrix_to_origin(head_top_t)
        self._painter.draw_line(head_bottom, head_top)

        head_left_t = head_top_t.dot(get_translate_matrix(0, 0, half_head_width))
        head_left = apply_matrix_to_origin(head_left_t)
        head_right_t = head_top_t.dot(get_translate_matrix(0, 0, -half_head_width))
        head_right = apply_matrix_to_origin(head_right_t)
        self._painter.draw_line(head_left, head_right)

    def draw_neck_and_head(self, neck_transversal_radians=0, neck_lateral_radians=0):

        neck_transversal = Joint(dh_params=(0 + neck_transversal_radians, neck_length, 0, half_pi),
                                 initial_transformation=get_translate_matrix(0, 0, torso_height))

        neck_lateral = Joint(prev_joint=neck_transversal, dh_params=(-neck_lateral_radians, 0, 0, 0))
        self.draw_link(neck_transversal)

        self._draw_head(head_t=neck_lateral.link_t)

    def draw_right_arm(self, right_shoulder_lateral_radians=0, right_shoulder_frontal_radians=0, right_elbow_lateral_radians=0, color='c-', draw=True):

        right_shoulder_lateral = Joint(dh_params=(half_pi + right_shoulder_lateral_radians, shoulder_length, 0, half_pi),
                                       initial_transformation=self._get_torso_corner_transformation(right_shoulder_position, 'x', half_pi))

        right_shoulder_frontal = Joint(prev_joint=right_shoulder_lateral, dh_params=(half_pi - right_shoulder_frontal_radians, 0, upper_arm_length, -half_pi))

        right_elbow_lateral = Joint(prev_joint=right_shoulder_frontal, dh_params=(-right_elbow_lateral_radians, 0, 0, 0))

        right_wrist_t = right_elbow_lateral.link_t.dot(get_translate_matrix(lower_arm_length, 0, 0))
        right_wrist = apply_matrix_to_origin(right_wrist_t)

        if draw:
            self.draw_link(right_shoulder_lateral, color=color)
            self.draw_link(right_shoulder_frontal, color=color)
            self._painter.draw_line(right_elbow_lateral.vertex, right_wrist, color=color)

        return right_shoulder_lateral, right_shoulder_frontal, right_elbow_lateral, right_wrist

    def draw_left_arm(self, left_shoulder_lateral_radians=0, left_shoulder_frontal_radians=0, left_elbow_lateral_radians=0, color='c-', draw=True):

        left_shoulder_lateral = Joint(dh_params=(-half_pi + left_shoulder_lateral_radians, shoulder_length, 0, -half_pi),
                                      initial_transformation=self._get_torso_corner_transformation(left_shoulder_position, 'x', -half_pi))

        left_shoulder_frontal = Joint(prev_joint=left_shoulder_lateral, dh_params=(half_pi - left_shoulder_frontal_radians, 0, -upper_arm_length, -half_pi))

        left_elbow_lateral = Joint(prev_joint=left_shoulder_frontal, dh_params=(-left_elbow_lateral_radians, 0, 0, 0))

        left_wrist_t = left_elbow_lateral.link_t.dot(get_translate_matrix(-lower_arm_length, 0, 0))
        left_wrist = apply_matrix_to_origin(left_wrist_t)

        if draw:
            self.draw_link(left_shoulder_lateral, color)
            self.draw_link(left_shoulder_frontal, color)
            self._painter.draw_line(left_elbow_lateral.vertex, left_wrist, color=color)

        return left_shoulder_lateral, left_shoulder_frontal, left_elbow_lateral, left_wrist

    def _draw_foot(self, ankle_t, color='c-', draw=True, get_lines=False):

        ankle = apply_matrix_to_origin(ankle_t)

        foot_center_t = ankle_t.dot(get_translate_matrix(0, 0, -foot_height))
        foot_center = apply_matrix_to_origin(foot_center_t)

        if draw or get_lines:
            foot_right_t = foot_center_t.dot(get_translate_matrix(-half_foot_width, 0, 0))
            foot_right = apply_matrix_to_origin(foot_right_t)
            foot_left_t = foot_center_t.dot(get_translate_matrix(half_foot_width, 0, 0))
            foot_left = apply_matrix_to_origin(foot_left_t)

            foot_front_t = foot_center_t.dot(get_translate_matrix(0, -half_foot_length, 0))
            foot_front = apply_matrix_to_origin(foot_front_t)
            foot_back_t = foot_center_t.dot(get_translate_matrix(0, half_foot_length, 0))
            foot_back = apply_matrix_to_origin(foot_back_t)

            ankle_foot_links = self._painter.draw_line(ankle, foot_center, color=color),\
                                        self._painter.draw_line(foot_right, foot_left, color=color),\
                                        self._painter.draw_line(foot_front, foot_back, color=color)

        if get_lines:
            return ankle_foot_links
        else:
            return foot_center

    def get_draw_lower_limp_function(self, is_left):
        if is_left:
            return self.draw_left_lower_limp
        else:
            return self.draw_right_lower_limp

    def draw_right_lower_limp(self,
                              right_hip_transversal_radians=0, right_hip_frontal_radians=0, right_hip_lateral_radians=0, right_knee_lateral_radians=0, right_ankle_lateral_radians=0, right_ankle_frontal_radians=0,
                              color='c-', draw=True, get_lines=False):

        right_hip_transversal = Joint(dh_params=(-half_pi + right_hip_transversal_radians, hip_length, 0, -half_pi),
                                      initial_transformation=self._get_torso_corner_transformation(right_hip_top_position, 'x', -pi))

        hip_frontal = Joint(prev_joint=right_hip_transversal, dh_params=(-half_pi - right_hip_frontal_radians, 0, 0, -half_pi))
        hip_lateral = Joint(prev_joint=hip_frontal, dh_params=(0 - right_hip_lateral_radians, 0, upper_leg_length, 0))

        knee_lateral = Joint(prev_joint=hip_lateral, dh_params=(0 - right_knee_lateral_radians, 0, lower_leg_length, 0))

        ankle_lateral = Joint(prev_joint=knee_lateral, dh_params=(0 + right_ankle_lateral_radians, 0, 0, half_pi))
        ankle_frontal = Joint(prev_joint=ankle_lateral, dh_params=(half_pi + right_ankle_frontal_radians, 0, 0, -half_pi))

        if draw or get_lines:
            leg_links = self.draw_link(right_hip_transversal, color),\
                        self.draw_link(hip_lateral, color),\
                        self.draw_link(knee_lateral, color)

        if get_lines:
            ankle_foot_links = self._draw_foot(ankle_t=ankle_frontal.link_t, color=color, draw=draw, get_lines=get_lines)
            return tuple(list(leg_links) + list(ankle_foot_links))
        else:
            foot_center = self._draw_foot(ankle_t=ankle_frontal.link_t, color=color, draw=draw)
            return right_hip_transversal, hip_frontal, hip_lateral, knee_lateral, ankle_lateral, ankle_frontal, foot_center

    def draw_left_lower_limp(self,
                             left_hip_transversal_radians=0, left_hip_frontal_radians=0, left_hip_lateral_radians=0, left_knee_lateral_radians=0, left_ankle_lateral_radians=0, left_ankle_frontal_radians=0,
                             color='c-', draw=True, get_lines=False):

        left_hip_transversal = Joint(dh_params=(-half_pi + left_hip_transversal_radians, hip_length, 0, -half_pi),
                                     initial_transformation=self._get_torso_corner_transformation(left_hip_top_position, 'x', -pi))

        hip_frontal = Joint(prev_joint=left_hip_transversal, dh_params=(-half_pi - left_hip_frontal_radians, 0, 0, -half_pi))
        hip_lateral = Joint(prev_joint=hip_frontal, dh_params=(0 + left_hip_lateral_radians, 0, upper_leg_length, 0))

        knee_lateral = Joint(prev_joint=hip_lateral, dh_params=(0 + left_knee_lateral_radians, 0, lower_leg_length, 0))

        ankle_lateral = Joint(prev_joint=knee_lateral, dh_params=(0 - left_ankle_lateral_radians, 0, 0, half_pi))
        ankle_frontal = Joint(prev_joint=ankle_lateral, dh_params=(half_pi + left_ankle_frontal_radians, 0, 0, -half_pi))

        if draw or get_lines:
            leg_links = self.draw_link(left_hip_transversal, color),\
                        self.draw_link(hip_lateral, color),\
                        self.draw_link(knee_lateral, color)

        if get_lines:
            ankle_foot_links = self._draw_foot(ankle_t=ankle_frontal.link_t, color=color, draw=draw, get_lines=get_lines)
            return tuple(list(leg_links) + list(ankle_foot_links))
        else:
            foot_center = self._draw_foot(ankle_t=ankle_frontal.link_t, color=color, draw=draw)
            return left_hip_transversal, hip_frontal, hip_lateral, knee_lateral, ankle_lateral, ankle_frontal, foot_center

    def draw_torso(self, draw=True):
        torso = np.array([right_shoulder_position,
                          left_shoulder_position,
                          left_hip_top_position,
                          right_hip_top_position,
                          right_shoulder_position])

        if draw:
            self._painter.draw_with_points(torso, color='c-')

        return right_shoulder_position, left_shoulder_position, left_hip_top_position, right_hip_top_position

    def draw_initial_pose(self):
        self.draw_neck_and_head()
        self.draw_torso()
        self.draw_left_arm()
        self.draw_right_arm()
        self.draw_left_lower_limp()
        self.draw_right_lower_limp()

    def draw_coordinates(self):
        self._painter.draw_global_coordinates()

        for j in Joint.all_joints:
            self._painter.draw_coordinates(j.t)

    def show(self):
        self._painter.show()