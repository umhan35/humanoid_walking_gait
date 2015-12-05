from matplotlib.animation import FuncAnimation

from robot_spec.length_data import initial_hip_height
from util.point import Point
from robot_spec.position_data import initial_left_foot_position, initial_right_foot_position, get_initial_foot_position
from util.pi import half_pi
from walking.composition.robot_walking_lower_limp_kinematics_engine import RobotWalkingLowerLimpKinematicsEngine
from walking.helpers.step_timer import StepTimer
from walking.helpers.bezier_curves import LineSegment


class VirtualRobotWalkingAnimated(RobotWalkingLowerLimpKinematicsEngine):

    frames = 40
    intervals = 50

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

            current_left_lower_limp_angles = [(each / self.frames * i) for each in left_lower_limp_angles]
            current_right_lower_limp_angles = [(each / self.frames * i) for each in right_lower_limp_angles]

            links = self.robot_model.draw_left_lower_limp(*current_left_lower_limp_angles, draw=False, get_lines=True)\
                   + self.robot_model.draw_right_lower_limp(*current_right_lower_limp_angles, draw=False, get_lines=True)

            # self.saved_links = links
            return links

        FuncAnimation(fig=self.painter.fig, init_func=lambda: [], func=animate_func, frames=VirtualRobotWalkingStatic.frames, interval=VirtualRobotWalkingStatic.intervals, blit=True, repeat=False)

    def draw_some_points(self):

        self.painter.draw_point((0, 0, 0))
        self.painter.draw_point((0, 0, -(initial_hip_height-self.hip_height)))

        self.painter.draw_point(initial_left_foot_position)
        self.painter.draw_point(initial_right_foot_position)

    def _take_one_step(self, is_swing_leg_left, lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section):

        print('walking_frequency', self.walking_frequency)

        step_timer = StepTimer(self.walking_frequency)

        def animate_func(i):

            print('i', i)

            fraction = i / step_timer.max_count
            print('fraction', fraction)
            p1, p2, p3 = lift_up_front_trajectory_section.get_point_at(fraction), lift_up_front_forward_trajectory_section.get_point_at(fraction), put_down_trajectory_section.get_point_at(fraction)
            p4, p5 = LineSegment(p1, p2).get_point_at(fraction), LineSegment(p2, p3).get_point_at(fraction)
            p_final = LineSegment(p4, p5).get_point_at(fraction)

            current_left_lower_limp_angles, current_right_lower_limp_angles = self.get_lower_limps_angle_set__with__both_tilted_torso_and_hip_offset_effect(p_final, is_swing_leg_left)

            self.painter.draw_line(p1, p2)
            self.painter.draw_line(p2, p3)

            links = self.robot_model.draw_left_lower_limp(*current_left_lower_limp_angles, draw=False, get_lines=True)\
                   + self.robot_model.draw_right_lower_limp(*current_right_lower_limp_angles, draw=False, get_lines=True)

            return links

        FuncAnimation(fig=self.painter.fig, init_func=lambda: [], func=animate_func, frames=step_timer.max_count, interval=self.intervals, blit=True, repeat=False)

    def walk(self):
        self.draw_natural_pose()
        # self.draw_some_points()

        is_swing_leg_left = True
        initial_position = Point(get_initial_foot_position(is_left=is_swing_leg_left))
        lift_up_front_position, lift_up_front_forward_position, put_down_position = self.get_walk_trajectory_three_positions(is_swing_leg_left=True, step_length=self.step_length / 2, current_position_relative_to_initial_foot_position=Point((0, 0, 0)))

        lift_up_front_trajectory_section = LineSegment(initial_position, lift_up_front_position)
        lift_up_front_forward_trajectory_section = LineSegment(lift_up_front_position, lift_up_front_forward_position)
        put_down_trajectory_section = LineSegment(lift_up_front_forward_position, put_down_position)


        print('initial_position', initial_position)
        print('lift_up_front_position', lift_up_front_position)
        print('lift_up_front_forward_position', lift_up_front_forward_position)
        print('put_down_position', put_down_position)
        self.painter.draw_point(initial_position, color='r.')
        self.painter.draw_point(lift_up_front_position, color='r.')
        self.painter.draw_point(lift_up_front_forward_position, color='r.')
        self.painter.draw_point(put_down_position, color='r.')

        self.painter.draw_line(initial_position, lift_up_front_position, color='b-')
        self.painter.draw_line(lift_up_front_position, lift_up_front_forward_position, color='b-')
        self.painter.draw_line(lift_up_front_forward_position, put_down_position, color='b-')


        self._take_one_step(is_swing_leg_left, lift_up_front_trajectory_section, lift_up_front_forward_trajectory_section, put_down_trajectory_section)

        self.painter.show()