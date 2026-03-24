import numpy as np
from ..common import ControllerBase
from . import params as P


class FullPDController(ControllerBase):
    def __init__(self):
        self.kp_theta = P.kp_theta
        self.kd_theta = P.kd_theta

        self.kp_phi = P.kp_phi
        self.kd_phi = P.kd_phi

        self.kp_psi = P.kp_psi
        self.kd_psi = P.kd_psi

        self.Fe = P.Fe

        self.sigma = P.sigma
        self.beta = (2.0 * self.sigma - P.ts) / (2.0 * self.sigma + P.ts)

        self.phi_d1 = P.phi0
        self.theta_d1 = P.theta0
        self.psi_d1 = P.psi0

        self.phidot = P.phidot0
        self.thetadot = P.thetadot0
        self.psidot = P.psidot0

    def update_with_measurement(self, r, y):
        phi = y[0]
        theta = y[1]
        psi = y[2]

        theta_ref = r[0]
        psi_ref = r[1]

        self.phidot = self.beta * self.phidot + (1.0 - self.beta) * ((phi - self.phi_d1) / P.ts)
        self.thetadot = self.beta * self.thetadot + (1.0 - self.beta) * ((theta - self.theta_d1) / P.ts)
        self.psidot = self.beta * self.psidot + (1.0 - self.beta) * ((psi - self.psi_d1) / P.ts)
        
        self.phi_d1 = phi
        self.theta_d1 = theta
        self.psi_d1 = psi

        theta_error = theta_ref - theta
        F_tilde = self.kp_theta * theta_error - self.kd_theta * self.thetadot
        F_fl = self.Fe * np.cos(theta)
        F = F_fl + F_tilde

        psi_error = psi_ref - psi
        phi_ref = self.kp_psi * psi_error - self.kd_psi * self.psidot

        phi_error = phi_ref - phi
        tau = self.kp_phi * phi_error - self.kd_phi * self.phidot

        return np.array([F, tau])

