"""
timer.py

Copyright 2018. All Rights Reserved.

Created: September 11, 2018
Authors: Toki Migimatsu
"""

import time

class Timer:

    def __init__(self, frequency):
        self.dt = 1. / frequency
        self._initialized = False
        self._num_iters = 0
        self._t_start = 0.
        self._t_next = 0.

    @property
    def frequency(self):
        return 1. / self.dt

    @frequency.setter
    def frequency(self, new_frequency):
        self.dt = 1. / new_frequency

    def time(self):
        return time.time()

    def time_elapsed(self):
        return time.time() - self._t_start

    def time_sim(self):
        return self._t_next - self._t_start

    @property
    def num_iterations(self):
        return self._num_iters

    def reset(self):
        self._t_start = time.time()
        self._t_next = self._t_start + self.dt
        self._num_iters = 0

    def sleep(self):
        self._num_iters += 1
        if not self._initialized:
            self._initialized = True
            self._t_start = time.time()
            self._t_next = self._t_start + self.dt
            return

        t_curr = time.time()
        if (t_curr < self._t_next):
            time.sleep(self._t_next - t_curr)
        self._t_next += self.dt

