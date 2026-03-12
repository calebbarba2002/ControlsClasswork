# 3rd-Party
import numpy as np

# local (controlbook)
from . import params as P
from .loopshaping import controller_inner, controller_outer, prefilter
from ..common import ControllerBase, loopshaping_tools as ls


class CartPendulumLoopshapedController(ControllerBase):
    def __init__(self):
        assert prefilter is not None  # make type checkers happy

        # convert TFs to digital filters
        self.prefilter = ls.DigitalFilter(prefilter, P.ts)
        self.controller_inner = ls.DigitalFilter(controller_inner, P.ts)
        self.controller_outer = ls.DigitalFilter(controller_outer, P.ts)

        self.F_eq = 0.0

    def update_with_state(self, r, x):
        y = x[:2]
        u, _ = self.update_with_measurement(r, y)
        return u

    def update_with_measurement(self, r, y):
        z_r = r[0]
        z, theta = y

        # This controller does not use velocity information, so we don't need
        # to estimate it; just return None for that part of the state estimate.
        # Instead, this controller uses a history of previous y values
        # internally in its digital filters.
        xhat = np.array([z, theta, None, None], dtype=np.float64)

        # prefilter the reference
        z_r_filtered = self.prefilter.update(z_r)

        error_z = z_r_filtered - z
        theta_r = self.controller_outer.update(error_z)

        error_theta = theta_r - theta
        F_tilde = self.controller_inner.update(error_theta)
        F = F_tilde + self.F_eq

        u_unsat = np.array([F])
        u = np.clip(u_unsat, -P.force_max, P.force_max)

        return u, xhat
