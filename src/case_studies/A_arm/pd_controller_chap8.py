# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common
import case_studies.A_arm.params as P


class ArmControllerPD(common.ControllerBase):
    def __init__(self, use_feedback_linearization: bool = True):
        # tuning parameters
        tr = 0.8  # original (part a)
        tr = 0.48  # tuned to saturate for single time step (part b)
        zeta = 0.707

        # system parameters
        b0 = P.tf_num[-1]
        a1, a0 = P.tf_den[-2:]

        # find gains
        wn = 2.2 / tr
        alpha0 = wn**2
        alpha1 = 2 * zeta * wn
        self.kp = (alpha0 - a0) / b0
        self.kd = (alpha1 - a1) / b0
        print(f"{self.kp = :.2f}, {self.kd = :.3f}")

        self.tau_eq = P.tau_eq
        self.use_feedback_linearization = use_feedback_linearization

    def update_with_state(self, r, x):
        # unpack references and states
        theta_ref = r[0]
        theta, thetadot = x

        # theta (modified) PD
        error = theta_ref - theta
        tau_tilde = self.kp * error - self.kd * thetadot

        if self.use_feedback_linearization:
            tau_fl = P.m * P.g * P.ell / 2 * np.cos(theta)
            tau = tau_tilde + tau_fl
        else:  # use equilibrium (Jacobian linearization)
            # NOTE: likely not work well (or at all) for large angles
            tau = tau_tilde + self.tau_eq

        u_unsat = np.array([tau])
        u = self.saturate(u_unsat, u_max=P.tau_max)
        return u
