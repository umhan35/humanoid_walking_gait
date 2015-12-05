from util.painter import Painter
from robot_model import RobotModel
from joint.joint_angle import *

r = RobotModel(Painter())

r.draw_neck_and_head()
r.draw_left_arm()
# r.draw_left_arm(radians(-30), radians(60), radians(-30))
# r._painter.draw_point(r.draw_left_arm(radians(0), radians(-60), radians(0), color='b-')[-1])
r.draw_right_arm()
# r.draw_right_arm(radians(0), radians(30), radians(160), color='b-')
# r._painter.draw_point(r.draw_right_arm(radians(0), radians(60), radians(0), color='b-')[-1])

# r.draw_left_lower_limp()
# r.draw_right_lower_limp(right_knee_lateral_radians=radians(150))

r.draw_left_lower_limp(
    # LeftHipTransversal().outward(0).radians,
    radians(-0),
    LeftHipFrontal().outward(0).angle,
    LeftHipLateral().forward(0).radians,
    LeftKneeLateral().backward(0).radians,
    # radians(150),
    LeftAnkleLateral().forward(0).radians,
    LeftAnkleFrontal().outward(-0).radians)

r.draw_right_lower_limp(
    RightHipTransversal().inward(0).radians,
    # radians(-0),
    RightHipFrontal().outward(0).angle,
    RightHipLateral().forward(0).radians,
    RightKneeLateral().backward(0).radians,
    RightAnkleLateral().forward(0).radians,
    RightAnkleFrontal().outward(-0).radians)


r.draw_torso()
r.draw_coordinates()
r.show()
# r._painter.show(block=False)
# r.draw_left_lower_limp(radians(-0), radians(0), radians(-30), radians(0), 0, 0)
# r.draw_left_lower_limp(radians(-0), radians(0), radians(-90), radians(0), 0, 0)
# r._painter.show(block=True)