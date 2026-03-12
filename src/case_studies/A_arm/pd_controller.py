# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common
import case_studies.A_arm.params as P


class ArmControllerPD(common.ControllerBase):
    def __init__(self, use_feedback_linearization: bool = True):
        # tuning parameters
        p1 = -3.0
        p2 = -4.0

        # system parameters
        b0 = P.tf_num[-1]
        a1, a0 = P.tf_den[-2:]

        # desired characteristic equation parameters
        # des_CE: s^2 + alpha1*s + alpha0 = s^2 + (a1 + b0*kd)s + (a0 + b0*kp)
        des_CE = np.poly([p1, p2])
        alpha1, alpha0 = des_CE[-2:]

        # find gains
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

        u = np.array([tau])
        return u
