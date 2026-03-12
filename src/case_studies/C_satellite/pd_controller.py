# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common
import case_studies.C_satellite.params as P


class SatelliteControllerPD(common.ControllerBase):
    def __init__(self):
        # tuning parameters
        tr_theta = 1.5  # original already saturated, so increased rise time.
        zeta_theta = 0.9
        M = 10  # time separation factor between inner and outer loop
        tr_phi = tr_theta * M
        zeta_phi = 0.9

        # system parameters
        a1_inner, a0_inner = P.tf_inner_den[-2:]
        b0_inner = P.tf_inner_num[-1]

        # Inner loop
        wn_theta = 0.5 * np.pi / (tr_theta * np.sqrt(1 - zeta_theta**2))
        alpha0_inner = wn_theta**2
        alpha1_inner = 2 * zeta_theta * wn_theta
        self.kp_theta = (alpha0_inner - a0_inner) / b0_inner
        self.kd_theta = (alpha1_inner - a1_inner) / b0_inner
        print(f"Inner loop (theta): kp = {self.kp_theta:.3f}, kd = {self.kd_theta:.3f}")

        # DC gain of inner loop
        DC_theta = (b0_inner * self.kp_theta) / (a0_inner + b0_inner * self.kp_theta)
        print(f"{DC_theta = }")  # a0 is 0, so DC_gain is 1

        # Outer loop
        wn_phi = 0.5 * np.pi / (tr_phi * np.sqrt(1 - zeta_phi**2))
        AA = DC_theta * np.array(
            [
                [P.k, -P.b * wn_phi**2],
                [P.b, P.k - P.b * 2 * zeta_phi * wn_phi],
            ]
        )
        bb = np.array([-P.k + P.Jp * wn_phi**2, -P.b + P.Jp * 2 * zeta_phi * wn_phi])
        self.kp_phi, self.kd_phi = np.linalg.solve(AA, bb)
        print(f"Outer loop (phi): kp = {self.kp_phi:.3f}, kd = {self.kd_phi:.3f}")

        # DC gain of outer loop
        DC_phi = P.k * DC_theta * self.kp_phi / (P.k + P.k * DC_theta * self.kp_phi)
        print(f"{DC_phi = :.3f}")

        self.tau_eq = P.u_eq[0]

    def update_with_state(self, r, x):
        # unpack references and states
        phi_ref = r[1]
        theta, phi, thetadot, phidot = x

        # outer loop (modified) PD
        error_phi = phi_ref - phi
        theta_ref_tilde = self.kp_phi * error_phi - self.kd_phi * phidot
        theta_ref_ff = phi_ref  # feedforward term because outer loop DC gain isn't 1
        theta_ref = theta_ref_tilde + theta_ref_ff
        r[0] = theta_ref  # if you want to visualize the "reference" angle

        # inner loop (modified) PD
        error_theta = theta_ref - theta
        tau_tilde = self.kp_theta * error_theta - self.kd_theta * thetadot
        tau = tau_tilde + self.tau_eq
        u_unsat = np.array([tau])
        u = self.saturate(u_unsat, u_max=P.torque_max)
        return u
