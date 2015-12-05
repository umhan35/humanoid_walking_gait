import time
import math


class PerformanceTimer:
    """
    it uses time.perf_counter()
    """

    def __init__(self, interval, duration):
        self.interval = interval
        self.duration = duration

        self.start_time = None
        self.last_recorded_time = None

    def start(self):
        self.start_time = time.perf_counter()
        self.last_recorded_time = self.start_time

    def record_time(self):
        self.last_recorded_time = time.perf_counter()

    def elapsed_since_last_recorded_time(self):
        return time.perf_counter() - self.last_recorded_time

    def elapsed_since_beginning(self):
        return time.perf_counter() - self.start_time

    def sleep_until_next_interval(self):
        return time.sleep(self.interval - self.elapsed_since_last_recorded_time())

    def next_count(self):
        return math.ceil(self.elapsed_since_beginning() / self.interval)

    def is_done(self):
        return self.elapsed_since_beginning() >= self.duration

    def restart(self):
        self.start()