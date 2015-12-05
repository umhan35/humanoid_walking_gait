from util.geometry import distance_between_2_vertices_in_3d
from util.float import is_float_equal


class Point(tuple):
    """
    3D point
    """

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def z(self):
        return self[2]

    def translate(self, dx=0, dy=0, dz=0):
        """
        it don't modify original x,y,z
        """
        return Point((self.x + dx, self.y + dy, self.z + dz))

    def distance_to(self, point_b):
        return distance_between_2_vertices_in_3d(self, point_b)

    def float_equals(self, point_b):
        return is_float_equal(self.x, point_b.x)\
           and is_float_equal(self.y, point_b.y)\
           and is_float_equal(self.z, point_b.z)

    def to_2d(self):
        return Point((self.x, self.y))