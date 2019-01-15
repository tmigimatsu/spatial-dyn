"""
timer.py

Copyright 2018. All Rights Reserved.

Created: September 11, 2018
Authors: Toki Migimatsu
"""

import time

##
# @ingroup py_utils
#
# Timer utility class for use in control loops.
#
# @see C++: SpatialDyn::Timer
class Timer:

    ##
    # Constructor that sets the loop frequency to 1000 Hz by default.
    #
    # @param frequency float
    # @see C++: SpatialDyn::Timer()
    def __init__(self, frequency = 1000.):
        ##
        # Timer loop time interval in seconds.
        #
        # @return float
        # @see C++: SpatialDyn::dt()
        self.dt = 1. / frequency

        ## @cond
        self._initialized = False
        self._num_iters = 0
        self._t_start = 0.
        self._t_next = 0.
        ## @endcond

    ##
    # Timer loop frequency in Hz.
    #
    # @return float
    # @see C++: SpatialDyn::freq()
    @property
    def freq(self):
        return 1. / self.dt

    @freq.setter
    def freq(self, frequency):
        self.dt = 1. / frequency

    ##
    # Current CPU time since epoch in seconds.
    #
    # @return float
    # @see C++: SpatialDyn::Timer::time()
    def time(self):
        return time.time()

    ##
    # Current CPU time since last timer reset in seconds.
    #
    # @return float
    # @see C++: SpatialDyn::Timer::time_elapsed()
    def time_elapsed(self):
        return time.time() - self._t_start

    ##
    # Simulation time since last timer reset in seconds.
    #
    # @return float
    # @see C++: SpatialDyn::Timer::time_elapsed()
    def time_sim(self):
        return self._t_next - self._t_start

    ##
    # Number of loop iterations since last timer reset.
    #
    # @see C++: SpatialDyn::Timer::num_iters()
    @property
    def num_iters(self):
        return self._num_iters

    ##
    # Reset timer.
    #
    # @see C++: SpatialDyn::Timer::Reset()
    def reset(self):
        self._num_iters = 0
        self._t_start = time.time()
        self._t_next = self._t_start + self.dt

    ##
    # Wait for the next timer loop.
    #
    # @see C++: SpatialDyn::Timer::Sleep()
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

