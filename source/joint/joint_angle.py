from math import radians

# things in this file helps you to get angles so you don't have to remember minus sign -> outwards or inwards...


class JointAngle:

    def __init__(self):
        self.angle = 0

    @property
    def radians(self):
        return radians(self.angle)


class JointFrontalAngle(JointAngle):

    def __init__(self, outward_coefficient):
        assert abs(outward_coefficient) == 1
        self.outward_coefficient = outward_coefficient
        super().__init__()

    def outward(self, value):
        self.angle += self.outward_coefficient * value
        return self

    def inward(self, value):
        self.angle += -self.outward_coefficient * value
        return self


class JointLateralAngle(JointAngle):

    def __init__(self, forward_coefficient):
        assert abs(forward_coefficient) == 1
        self.forward_coefficient = forward_coefficient
        super().__init__()

    def forward(self, value):
        self.angle += self.forward_coefficient * value
        return self

    def backward(self, value):
        self.angle += -self.forward_coefficient * value
        return self

# Uses JointFrontalAngle because HipTransversal join is also outward and inward
HipTransversal = JointFrontalAngle

LeftHipTransversal = lambda: HipTransversal(outward_coefficient=-1)
RightHipTransversal = lambda: HipTransversal(outward_coefficient=-1)

LeftHipFrontal = lambda: JointFrontalAngle(outward_coefficient=-1)
RightHipFrontal = lambda: JointFrontalAngle(outward_coefficient=1)

LeftHipLateral = lambda: JointLateralAngle(forward_coefficient=-1)
RightHipLateral = lambda: JointLateralAngle(forward_coefficient=1)

LeftKneeLateral = lambda: JointLateralAngle(forward_coefficient=-1)
RightKneeLateral = lambda: JointLateralAngle(forward_coefficient=1)

LeftAnkleLateral = lambda: JointLateralAngle(forward_coefficient=1)
RightAnkleLateral = lambda: JointLateralAngle(forward_coefficient=-1)

LeftAnkleFrontal = lambda: JointFrontalAngle(outward_coefficient=1)
RightAnkleFrontal = lambda: JointFrontalAngle(outward_coefficient=-1)