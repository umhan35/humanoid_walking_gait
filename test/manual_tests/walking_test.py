from math import radians
from robot_client_server.robot_clients import VirtualRobotClient, RealRobotClient

from robot_spec.length_data import initial_hip_height
from walking import *


# step height should < 6
max_step_height = 6
# step length should < 10
max_step_length = 10

robot_server = VirtualRobotClient()
# robot_server = RealRobotClient()

# VirtualRobotWalkingStatic(
# VirtualRobotWalkingAnimated(

# RobotPiecewiseConstantInterpolationWalking(
# RobotLinearInterpolationWalking(
# RobotBezierCurveInterpolationWalking(
# RobotSplineInterpolationWalking(
#
#     hip_height=initial_hip_height-3,
#
#     torso_angle=radians(0),
#     # torso_angle=radians(12),
#
#     # lift_off_slope=0,
#     lift_off_slope=2,
#
#     # step_height=max_step_height/2,
#     # step_height=max_step_height,
#     step_height=4,
#
#     # step_length=max_step_length/3,
#     step_length=20,
#
#     # put_down_slope=0,
#     put_down_slope=2,
#
#     duration_of_rest=0.5,
#
#     walking_frequency=1/2,
#
#     robot_server=robot_server
#
# ).walk(sleep_time_after_squat=1)


# stable
walking_params = {
    "hip_height": initial_hip_height-3,
    "torso_angle": radians(12),
    "lift_off_slope": 2,
    "step_height": 2,
    "step_length": 4,
    "put_down_slope": 2,
    "duration_of_rest": 0,
    "walking_frequency": 1/0.2,
}


# RobotPiecewiseConstantInterpolationWalking(
# # RobotLinearInterpolationWalking(
# # RobotSplineInterpolationWalking(
# # RobotBezierCurveInterpolationWalking(
#
#     hip_height=initial_hip_height-7,
#
#     # torso_angle=radians(0),
#     torso_angle=radians(12),
#
#     # lift_off_slope=0,
#     lift_off_slope=2,
#
#     step_height=2,
#     # step_height=4,
#
#     step_length=10,
#
#     # put_down_slope=0,
#     put_down_slope=2,
#
#     duration_of_rest=0,
#
#     # walking_frequency=1/0.2,
#     walking_frequency=1/0.2,
#
#     robot_server=robot_server
#
# ).walk(sleep_time_after_squat=2)



RobotPiecewiseConstantInterpolationWalking(
# RobotLinearInterpolationWalking(
# RobotSplineInterpolationWalking(
# RobotBezierCurveInterpolationWalking(

    hip_height=initial_hip_height-2,

    torso_angle=radians(0),
    # torso_angle=radians(12),

    # lift_off_slope=0,
    lift_off_slope=0.4,

    step_height=1,
    # step_height=4,

    step_length=6,

    # put_down_slope=0,
    put_down_slope=0.4,

    duration_of_rest=0,

    # walking_frequency=1/0.2,
    walking_frequency=1/5,

    robot_server=robot_server

).walk(sleep_time_after_squat=1)