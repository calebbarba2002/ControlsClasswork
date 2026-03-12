# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class SatelliteSSController(ControllerBase):
    def __init__(self):
        # tuning parameters
        tr_theta = 2.0
        zeta_theta = 0.9
        M = 3  # time separation factor between inner and outer loop
        tr_phi = tr_theta * M
        zeta_phi = 0.9

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(P.A, P.B)) != 4:
            raise ValueError("System not controllable")

        # compute gains
        wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta_theta**2))
        theta_char_poly = [1, 2 * zeta_theta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_phi = 0.5 * np.pi / (tr_phi * np.sqrt(1 - zeta_phi**2))
        phi_char_poly = [1, 2 * zeta_phi * wn_phi, wn_phi**2]
        phi_poles = np.roots(phi_char_poly)

        des_poles = np.hstack([theta_poles, phi_poles])

        self.K = cnt.place(P.A, P.B, des_poles)
        self.kr = -1.0 / (P.Cr @ np.linalg.inv(P.A - P.B @ self.K) @ P.B)
        print("des_poles:", des_poles)
        print("K:", self.K)
        print("kr:", self.kr)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        # dirty derivative variables
        sigma = 0.05  # cutoff freq for dirty derivative
        self.beta = (2 * sigma - P.ts) / (2 * sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0
        self.phidot_hat = P.phidot0
        self.phi_prev = P.phi0

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # compute state feedback control
        u_tilde = -self.K @ x_tilde + self.kr @ r_tilde

        # convert back to original variables
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.torque_max)
        return u
