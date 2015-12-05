from util.dh import dh
from util.transformations import apply_matrix_to_origin


class Joint(object):

    all_joints = []

    def __init__(self, prev_joint=None, dh_params=None, initial_transformation=None):
        """
        :param initial_transformation: used for the first joint in a link to transform relative to global coordinate origin
        """

        if prev_joint is not None:
            prev_joint._next_joint = self
            self._prev_joint = prev_joint

        self._next_joint = None
        self._dh_params = dh_params
        self._initial_transformation = initial_transformation  # transformation
        self._saved_transformation = None
        self._link_t = None
        self._vertex = None

        self.all_joints.append(self)

    @property
    def link_t(self):

        if self._link_t is None:
            self._link_t = self.t.dot(dh(*self._dh_params))

        return self._link_t

    @property
    def t(self):
        if self._saved_transformation is not None:
            return self._saved_transformation

        if self._initial_transformation is not None:
            self._saved_transformation = self._initial_transformation
        else:
            self._saved_transformation = self._prev_joint.link_t

        return self._saved_transformation

    @property
    def vertex(self):
        if self._vertex is None:
            self._vertex = apply_matrix_to_origin(self.t)

        return self._vertex

    @property
    def link(self):
        return self, self._next_joint