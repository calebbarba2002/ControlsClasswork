# 3rd-party
import numpy as np

# local (controlbook)
from . import params as P
from .. import common


class ArmControllerPID(common.ControllerBase):
    def __init__(self):
        # tuning parameters
        tr = 0.6
        zeta = 0.9
        self.ki = 0.2

        # system parameters
        b0 = P.tf_num[-1]
        a1, a0 = P.tf_den[-2:]

        # find gains
        wn = 0.5 * np.pi / (tr * np.sqrt(1 - zeta**2))
        alpha0 = wn**2
        alpha1 = 2 * zeta * wn
        self.kp = (alpha0 - a0) / b0
        self.kd = (alpha1 - a1) / b0
        print(f"{self.kp = :.3f}, {self.ki = :.3f}, {self.kd = :.3f}")

        self.tau_eq = P.tau_eq

        # variables for dirty derivative
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.ts) / (2 * self.sigma + P.ts)
        self.thetadot_hat = P.thetadot0  # estimated derivative of theta
        self.theta_prev = P.theta0

        # variables for integrator
        self.error_prev = 0.0
        self.error_integral = 0.0

    def update_with_measurement(self, r, y):
        theta = y[0]
        theta_ref = r[0]

        # dirty derivative to estimate thetadot
        theta_diff = (theta - self.theta_prev) / P.ts
        self.thetadot_hat = self.beta * self.thetadot_hat + (1 - self.beta) * theta_diff
        self.theta_prev = theta

        # compute input from partially estimated state
        xhat = np.array([theta, self.thetadot_hat])

        # integrate error
        error = theta_ref - theta
        if (
            abs(self.thetadot_hat) < 0.08
        ):  # anti-windup: only integrate if thetadot is small
            self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # theta (modified) PID
        tau_tilde = (
            self.kp * error
            + self.ki * self.error_integral
            - self.kd * self.thetadot_hat
        )
        tau_fl = P.m * P.g * P.ell / 2 * np.cos(theta)
        tau = tau_tilde + tau_fl
        u_unsat = np.array([tau])
        u = self.saturate(u_unsat, u_max=P.tau_max)

        return u, xhat
