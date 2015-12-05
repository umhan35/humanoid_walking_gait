# Humanoid Walking Gait

This code was written during <a href="https://docs.google.com/document/d/1mPWBGmQyCL091ZmEOl5VE-HMGXpkN_5cPioKWj16Hgo/pub">the Humanoid Robotics class</a> taught by <a href="http://www.cs.umanitoba.ca/~jacky/">Dr. Jacky Baltes</a>. It contains:
- an implementation of forward and inverse kinematics for a teen-sized humanoid (<a href="https://docs.google.com/document/d/1DC1oZSwRnXxOZyOl3Uws5lCSxL411XOLhUKwFWYq-7w/pub">Assignment 1</a>)
- an implementation of a parameterized walking engine by applying bézier curve for the humanoid (<a href="https://docs.google.com/document/d/1xn-Vz_1iDdZaxjkU-f0krdVOTQELIBOWTNv0RW_a9qY/pub">Assignment 2</a> & <a href="https://docs.google.com/document/d/16tFkeojwIg4bChwQuD_m7U0C73lbdEWQNqBkyzHhOx0/pub">Assignment 3</a>)

It's written in `python3`. `matplotlib` and some other packages are used.

## How to run

You can run `walking_test.py` to test the walking gait. In line 13 and 14 of that file, you can uncomment or comment out:

```
robot_server = VirtualRobotClient()
# robot_server = RealRobotClient()
```

to either test on real robot or virtual robot. For the virual robot, you need to run `virtual_robot_server.py` to open the virtual robot.
