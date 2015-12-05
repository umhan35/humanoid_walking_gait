from robot_spec.length_data import get_initial_leg_y, initial_foot_z


class WalkTrajectory:

    def __init__(self, hip_offset, lift_off_slope, step_height, step_length, put_down_slope):
        self.hip_offset = hip_offset
        self.lift_off_slope = lift_off_slope
        self.step_height = step_height
        self.step_length = step_length
        self.put_down_slope = put_down_slope

        self.lift_up_x_side_length = self._calculate_lift_up_x_side_length()
        self.put_down_x_side_length = self._calculate_put_down_x_side_length()

        assert self.lift_up_x_side_length + self.put_down_x_side_length <= self.step_length, '{} + {} > {}'.format(self.lift_up_x_side_length, self.put_down_x_side_length, self.step_length)

    def get_lift_front_position(self, foot_position_relative_to, is_left):

        lift_front_x = self.lift_up_x_side_length
        lift_front_y = get_initial_leg_y(is_left)
        lift_front_z = initial_foot_z + self.hip_offset

        lift_front_position = foot_position_relative_to.translate(dx=lift_front_x, dy=lift_front_y, dz=lift_front_z)

        return lift_front_position

    def get_lift_up_front_forward_position(self, lift_up_front_position):
        return lift_up_front_position.translate(dx=self.step_length-self.lift_up_x_side_length-self.put_down_x_side_length)

    def get_put_down_position(self, lift_up_front_forward_position):
        return lift_up_front_forward_position.translate(dx=self.put_down_x_side_length, dz=-self.hip_offset)

    def _calculate_lift_up_x_side_length(self):

        if self.lift_off_slope == 0:
            length = 0
        else:
            length = self.step_height / self.lift_off_slope

        return length

    def _calculate_put_down_x_side_length(self):

        if self.put_down_slope == 0:
            length = 0
        else:
            length = self.step_height / self.put_down_slope

        return length

    @staticmethod
    def project_to_opposite_ground(position, is_left):
        x = -position.x
        y = get_initial_leg_y(is_left)
        z = initial_foot_z  # didn't add effect yet

        return x, y, z
