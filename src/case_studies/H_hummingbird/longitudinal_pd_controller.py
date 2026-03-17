import numpy as np
from .. import common
from . import params as P

class LongitudinalPDController(common.ControllerBase):
    def __init__(self):
        self.kp = 2.0
        self.kd = 0.7

    def update_with_state(self, r, x):
        theta_ref = r[1]
        theta = x[1]
        thetadot = x[4]

        error = theta_ref - theta

        F_tilde = self.kp * error - self.kd * thetadot

        # gravity compensation
        F_fl = (P.m1 * P.ell1 + P.m2 * P.ell2) * P.g / P.ellT * np.cos(theta)

        F = F_fl + F_tilde

        tau = 0.0  # REQUIRED by lab

        #convert to motor forces
        f = P.mixer @ np.array([F, tau])

        # convert forces → PWM
        km = P.g * (P.m1 * P.ell1 + P.m2 * P.ell2) / P.ellT
        u = f / km

        return u
