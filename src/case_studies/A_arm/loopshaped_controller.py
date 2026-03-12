# 3rd-Party
import numpy as np

# local (controlbook)
from . import params as P
from .loopshaping import controller, prefilter
from ..common import ControllerBase, loopshaping_tools as ls


class ArmControllerLoopshaped(ControllerBase):
    def __init__(self):
        assert prefilter is not None  # make type checkers happy

        # convert TFs to digital filters
        self.prefilter = ls.DigitalFilter(prefilter, P.ts)
        self.controller = ls.DigitalFilter(controller, P.ts)

    def update_with_state(self, r, x):
        y = x[:1]
        u, _ = self.update_with_measurement(r, y)
        return u

    def update_with_measurement(self, r, y):
        theta_r = r[0]
        theta = y[0]

        # This controller does not use velocity information, so we don't need
        # to estimate it; just return None for that part of the state estimate.
        # Instead, this controller uses a history of previous theta values
        # internally in its digital filters.
        xhat = np.array([theta, None], dtype=np.float64)

        # prefilter the reference
        theta_r_filtered = self.prefilter.update(theta_r)

        # filtered error signal
        error = theta_r_filtered - theta

        # update controller
        tau_tilde = self.controller.update(error)

        # compute feedback linearization torque tau_fl
        tau_fl = P.m * P.g * (P.ell / 2.0) * np.cos(theta)

        # compute total torque
        tau = tau_fl + tau_tilde
        u_unsat = np.array([tau])
        u = np.clip(u_unsat, -P.tau_max, P.tau_max)

        return u, xhat
