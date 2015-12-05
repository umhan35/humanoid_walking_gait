
class StepTimer:

    n_times_per_second = 40

    DEBUG = False

    def __init__(self, walking_frequency):

        # walking_frequency is n steps per second
        self.single_step_time = 1 / walking_frequency
        if self.DEBUG: print('single_step_time', self.single_step_time)

        # if n_times_per_second is 20,
        #   frequency would be 0.05 second - 50ms
        self.frequency = 1 / self.n_times_per_second
        if self.DEBUG: print('timer_frequency', self.frequency)

        assert self.single_step_time >= 2 * self.frequency, 'In order to lift and put down foot, single_step_time must be more than 2 step_time_frequenct'
        self.max_count = self.single_step_time / self.frequency
        if self.DEBUG: print('max_count', self.max_count)
#
# class StepTimer:
#
#     n_times_per_second = 20
#
#     def __init__(self, walking_frequency):
#
#         # walking_frequency is n steps per second
#         self.single_step_time = 1 / walking_frequency
#         print('single_step_time', self.single_step_time)
#
#          # the timer runs 20 times per second
#         self.frequency = 1 / self.n_times_per_second
#         print('timer_frequency', self.frequency)
#
#         self.max_count = int(self.single_step_time / self.frequency)
#         self.current_count = 0
#
#     def increase_count(self):
#         self.current_count += 1