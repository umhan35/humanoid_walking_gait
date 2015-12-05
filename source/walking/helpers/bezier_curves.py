from util.point import Point


def get_interpolated_position(fraction, first_trajectory_section, second_trajectory_section, third_trajectory_section):

    p1, p2, p3 = first_trajectory_section.get_point_at(fraction), second_trajectory_section.get_point_at(fraction), third_trajectory_section.get_point_at(fraction)
    p4, p5 = LineSegment(p1, p2).get_point_at(fraction), LineSegment(p2, p3).get_point_at(fraction)
    p_final = LineSegment(p4, p5).get_point_at(fraction)

    return p_final


class LineSegment:

    def __init__(self, initial, end):
        self.initial = Point(initial)
        self.end = Point(end)

        self._mid_point = None
        self._length = None

    def get_point_at(self, fraction):
        x = self.initial.x + (self.end.x - self.initial.x) * fraction
        y = self.initial.y + (self.end.y - self.initial.y) * fraction
        z = self.initial.z + (self.end.z - self.initial.z) * fraction

        return Point((x, y, z))
    
    def has_point(self, p):
        if self.initial.x <= p.x <= self.end.x\
            and self.initial.y <= p.y <= self.end.y\
            and self.initial.z <= p.x <= self.end.x:
            return True
        else:
            return False

    @property
    def mid_point(self):
        if self._mid_point is None:
            self._mid_point = self.get_point_at(0.5)

        return self._mid_point

    @property
    def length(self):
        if self._length is None:
            self._length = self.initial.distance_to(self.end)

        return self._length