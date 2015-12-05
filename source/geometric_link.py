from walking.helpers.bezier_curves import LineSegment


class GeometricLink:

    def __init__(self, start_joint_position, end_joint_position, link_mass):
        self.start_joint_position = start_joint_position
        self.end_joint_position = end_joint_position
        self.mass = link_mass

        self._position_x_mass = None

    @property
    def position_x_mass(self):
        """
        In the method name, both position and mass are in center.
        returns x, y, z results (will be cached)
        """
        if self._position_x_mass is None:
            position = LineSegment(self.start_joint_position, self.end_joint_position).mid_point
            self._position_x_mass = [each_dimension * self.mass for each_dimension in position]

        return self._position_x_mass