from util.painter import Painter
from util.point import Point
from robot_model import RobotModel
from robot_spec.position_data import *
from inverse_kinematics.arm import InverseKinematicsArm
from inverse_kinematics.lower_limp import InverseKinematicsLeg
from inverse_kinematics.head import move_head
from robot_client_server.robot_clients import RealRobotClient
from joint import *

painter = Painter(azimuth=0, elevation=50)

r = RobotModel(painter)


def inverse_kinematics(head_target_position, l_arm_target_position, r_arm_target_position, l_leg_target_position, l_hip_transversal_radians, r_leg_target_position, r_hip_transversal_radians, do_actuate):

    # move kinematic chains
    neck_transversal_radians, neck_lateral_radians\
        = move_head(head_target_position, painter)
    l_shoulder_lateral_radians, l_shoulder_frontal_radians, l_elbow_lateral_radians\
        = InverseKinematicsArm(l_arm_target_position, r, painter, is_left=True).get_angles()
    r_shoulder_lateral_radians, r_shoulder_frontal_radians, r_elbow_lateral_radians\
        = InverseKinematicsArm(r_arm_target_position, r, painter, is_left=False).get_angles()
    l_hip_frontal_radians, l_hip_lateral_radians, l_knee_lateral_radians, l_ankle_lateral_radians, l_ankle_frontal_radians\
        = InverseKinematicsLeg(is_left=True, l_hip_transversal_radians, l_leg_target_position, r, painter).get_angles()
    r_hip_frontal_radians, r_hip_lateral_radians, r_knee_lateral_radians, r_ankle_lateral_radians, r_ankle_frontal_radians\
        = InverseKinematicsLeg(is_left=False, r_hip_transversal_radians, r_leg_target_position, r, painter).get_angles()

    # print('lll', to_degrees([l_hip_transversal_radians, l_hip_frontal_radians, l_hip_lateral_radians, l_knee_lateral_radians, l_ankle_lateral_radians, l_ankle_frontal_radians]))
    # print('rll', to_degrees([r_hip_transversal_radians, r_hip_frontal_radians, r_hip_lateral_radians, r_knee_lateral_radians, r_ankle_lateral_radians, r_ankle_frontal_radians]))

    if do_actuate:
        c = RealRobotClient()
        # c.actuate_left_arm([l_shoulder_lateral_radians, l_shoulder_frontal_radians, l_elbow_lateral_radians])
        # c.actuate_right_arm([r_shoulder_lateral_radians, r_shoulder_frontal_radians, r_elbow_lateral_radians])
        c.actuate_left_lower_limp([l_hip_transversal_radians, l_hip_frontal_radians, l_hip_lateral_radians, l_knee_lateral_radians, l_ankle_lateral_radians, l_ankle_frontal_radians])
        c.actuate_right_lower_limp([r_hip_transversal_radians, r_hip_frontal_radians, r_hip_lateral_radians, r_knee_lateral_radians, r_ankle_lateral_radians, r_ankle_frontal_radians])
    else:
        # actual drawing work
        r.draw_neck_and_head(neck_transversal_radians=neck_transversal_radians, neck_lateral_radians=neck_lateral_radians)
        r.draw_left_arm(left_shoulder_lateral_radians=l_shoulder_lateral_radians, left_shoulder_frontal_radians=l_shoulder_frontal_radians, left_elbow_lateral_radians=l_elbow_lateral_radians)
        r.draw_right_arm(right_shoulder_lateral_radians=r_shoulder_lateral_radians, right_shoulder_frontal_radians=r_shoulder_frontal_radians, right_elbow_lateral_radians=r_elbow_lateral_radians)
        r.draw_left_lower_limp(left_hip_transversal_radians=l_hip_transversal_radians, left_hip_frontal_radians=l_hip_frontal_radians, left_hip_lateral_radians=l_hip_lateral_radians, left_knee_lateral_radians=l_knee_lateral_radians, left_ankle_lateral_radians=l_ankle_lateral_radians, left_ankle_frontal_radians=l_ankle_frontal_radians)
        r.draw_right_lower_limp(right_hip_transversal_radians=r_hip_transversal_radians, right_hip_frontal_radians=r_hip_frontal_radians, right_hip_lateral_radians=r_hip_lateral_radians, right_knee_lateral_radians=r_knee_lateral_radians, right_ankle_lateral_radians=r_ankle_lateral_radians, right_ankle_frontal_radians=r_ankle_frontal_radians)
        r.draw_torso()
        # r.draw_coordinates()  # draw coordinates lastly because we want them on top layer
        r.show()


def main():

    head_target_position = head_initial

    def get_arm_position(arm_angles, is_left):
        draw_arm = r.draw_left_arm if is_left else r.draw_right_arm

        shoulder_lateral, shoulder_frontal, elbow_lateral, wrist\
            = draw_arm(*arm_angles, draw=False, color='b-')

        return wrist

    # l_arm_position = get_arm_position((-0.58282, 1.19506, -1.49554), is_left=True)
    # l_arm_position = (32, 20, 18)
    # l_arm_position = (32, 15, 18)
    # l_arm_position = (32, 15, 23)
    # l_arm_position = get_arm_position((0, -30, -180), is_left=True)

    l_arm_position = get_arm_position((0, 0, 0), is_left=True)
    r_arm_position = get_arm_position((0, 0, 0), is_left=False)




    def get_leg_position(left_leg_angles, is_left):
        if is_left:
            draw_lower_limp = r.draw_left_lower_limp
        else:
            draw_lower_limp = r.draw_right_lower_limp
        left_hip_transversal, hip_frontal, hip_lateral, knee_lateral, ankle_lateral, ankle_frontal, foot_center\
            = draw_lower_limp(radians(left_leg_angles[0]), radians(left_leg_angles[1]), radians(left_leg_angles[2]), radians(left_leg_angles[3]), 0, 0, color='b--', draw=False)

        print('pd', left_leg_angles)
        return translate_3d_point(ankle_frontal.vertex, dz=-foot_height)

    def get_left_leg_target_position():
        # return get_leg_position((l_hip_transversal_degrees, 0, -0, -0), is_left=True)
        initial_foot_position = Point(initial_left_foot_position)

        up = initial_foot_position.translate(dz=2*foot_height)
        _front_up = up.translate(dx=2*foot_height)
        _back_up = up.translate(dx=-2*foot_height)
        up_out = up.translate(dy=2*foot_height)
        out_down = up_out.translate(dz=-foot_height)
        front_out_up = up_out.translate(dx=2*foot_height)
        _back_out_up = up_out.translate(dx=-2*foot_height)
        front_out_down = out_down.translate(dx=2*foot_height)
        _back_out_down = out_down.translate(dx=-2*foot_height)

        for p in [initial_foot_position, up, _front_up, _back_up, up_out, out_down, front_out_up, _back_out_up, front_out_down, _back_out_down]:
            painter.draw_point(p)

        return _back_up




    l_hip_transversal_degrees = 0
    l_leg_target_position = get_left_leg_target_position()

    def get_right_leg_target_position():
        # return get_leg_position((r_hip_transversal_degrees, RightHipFrontal().outward(20).degrees, 0, 0), is_left=False)

        initial_foot_position = Point(initial_right_foot_position)

        up = initial_foot_position.translate(dz=2*foot_height)
        _front_up = up.translate(dx=2*foot_height)
        _back_up = up.translate(dx=-2*foot_height)
        up_out = up.translate(dy=-2*foot_height)
        out_down = up_out.translate(dz=-foot_height)
        front_out_up = up_out.translate(dx=2*foot_height)
        _back_out_up = up_out.translate(dx=-2*foot_height)
        front_out_down = out_down.translate(dx=2*foot_height)
        _back_out_down = out_down.translate(dx=-2*foot_height)

        for p in [initial_foot_position, up, _front_up, _back_up, up_out, out_down, front_out_up, _back_out_up, front_out_down, _back_out_down]:
            painter.draw_point(p)

        # right...
        return _back_up

    r_hip_transversal_degrees = 0
    r_leg_target_position = get_right_leg_target_position()




    inverse_kinematics(head_target_position,
                       l_arm_position, r_arm_position,
                       l_leg_target_position, radians(l_hip_transversal_degrees),
                       r_leg_target_position, radians(r_hip_transversal_degrees),
                       do_actuate=True)

if __name__ == '__main__':
    main()