from matplotlib.animation import FuncAnimation

from robot_spec.length_data import initial_hip_height
from util.point import Point
from robot_spec.position_data import initial_left_foot_position, initial_right_foot_position, get_initial_foot_position
from util.pi import half_pi
from walking.composition.robot_walking_lower_limp_kinematics_engine import RobotWalkingLowerLimpKinematicsEngine


class VirtualRobotWalkingStatic(RobotWalkingLowerLimpKinematicsEngine):

    frames = 40
    intervals = 10

    def __init__(self, hip_height, torso_angle, lift_off_slope, step_height, step_length, put_down_slope, walking_frequency):
        super().__init__(hip_height, torso_angle, lift_off_slope, step_height, step_length, put_down_slope, walking_frequency)

    def draw_natural_pose(self):
        self.robot_model.draw_neck_and_head()
        self.robot_model.draw_torso()
        self.robot_model.draw_left_arm(left_shoulder_frontal_radians=half_pi)
        self.robot_model.draw_right_arm(right_shoulder_frontal_radians=-half_pi)
        self.robot_model.draw_left_lower_limp()
        self.robot_model.draw_right_lower_limp()

    def draw_robot(self, left_lower_limp_angles, right_lower_limp_angles):

        self.robot_model.draw_left_lower_limp(*left_lower_limp_angles)
        self.robot_model.draw_right_lower_limp(*right_lower_limp_angles)
        return


        # self.saved_links = None

        def animate_func(i):
            # if self.saved_links is not None:
            #     for each in self.saved_links:
            #         each.set_data([], [])

            current_left_lower_limp_angles = [(each / VirtualRobotWalkingStatic.frames * i) for each in left_lower_limp_angles]
            current_right_lower_limp_angles = [(each / VirtualRobotWalkingStatic.frames * i) for each in right_lower_limp_angles]

            links = self.robot_model.draw_left_lower_limp(*current_left_lower_limp_angles, draw=False, get_lines=True)\
                   + self.robot_model.draw_right_lower_limp(*current_right_lower_limp_angles, draw=False, get_lines=True)

            # self.saved_links = links
            return links

        FuncAnimation(fig=self.painter.fig, init_func=lambda: [], func=animate_func, frames=VirtualRobotWalkingStatic.frames, interval=VirtualRobotWalkingStatic.intervals, blit=True, repeat=False)

    def draw_some_points(self):

        self.draw_natural_pose()

        self.painter.draw_point((0, 0, 0))
        self.painter.draw_point((0, 0, -(initial_hip_height-self.hip_height)))

        self.painter.draw_point(initial_left_foot_position)
        self.painter.draw_point(initial_right_foot_position)

    def walk(self):
        self.draw_some_points()

        # self.draw_robot(*self.get_squat_with_tilted_torso_lower_limp_angles())

        lower_limps_angle_set_for_lift_up_front_position, lower_limps_angle_set_for_lift_up_front_forward_position, lower_limps_angle_set_for_put_down_position\
            = self.get_feet_positions_set__with__both_tilted_torso_and_hip_offset_effect(is_swing_leg_left=True, step_length=self.step_length/2, current_position_relative_to_initial_foot_position=Point((0, 0, 0)))
        self.draw_robot(*lower_limps_angle_set_for_lift_up_front_position)
        self.draw_robot(*lower_limps_angle_set_for_lift_up_front_forward_position)
        self.draw_robot(*lower_limps_angle_set_for_put_down_position)

        # lower_limps_angle_set_for_lift_up_front_position, lower_limps_angle_set_for_lift_up_front_forward_position, lower_limps_angle_set_for_put_down_position\
        #     = self.get_lower_limps_angles_set__with__both_tilted_torso_and_hip_offset_effect(is_swing_leg_left=False, step_length=self.step_length, current_position_relative_to_initial_foot_position=Point((-self.step_length/2, 0, 0)))
        # self.draw_robot(*lower_limps_angle_set_for_lift_up_front_position)
        # self.draw_robot(*lower_limps_angle_set_for_lift_up_front_forward_position)
        # self.draw_robot(*lower_limps_angle_set_for_put_down_position)

        is_swing_leg_left = True
        initial_position = Point(get_initial_foot_position(is_left=is_swing_leg_left))
        lift_up_front_position, lift_up_front_forward_position, put_down_position = self.get_walk_trajectory_three_positions(is_swing_leg_left=True, step_length=self.step_length / 2, current_position_relative_to_initial_foot_position=Point((0, 0, 0)))

        self.painter.draw_line(initial_position, lift_up_front_position, color='b-')
        self.painter.draw_line(lift_up_front_position, lift_up_front_forward_position, color='b-')
        self.painter.draw_line(lift_up_front_forward_position, put_down_position, color='b-')

        self.painter.show()