from __future__ import division, print_function
import numpy as np


class PhaseVariable(object):
    def __init__(self):
        self.step_vectorized = np.vectorize(self.step, otypes=[float])
        self.reset()

    def step(self, dt):
        """
        Solve the canonical system at next time step t+dt.
        Parameters
        ----------
        dt : float
            Time step.
        """
        self.s += (-self.s) * dt
        return self.s

    def rollout(self, t):
        """
        Solve the canonical system.
        Parameters
        ----------
        t : array_like
            Time points for which to evaluate the integral.
        """
        self.reset()
        return self.step_vectorized(np.gradient(t))

    def reset(self):
        self.s = 1.0
