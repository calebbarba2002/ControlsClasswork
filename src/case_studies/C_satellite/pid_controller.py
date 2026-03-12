# 3rd-party
import numpy as np

# local (controlbook)
from case_studies import common
import case_studies.C_satellite.params as P


class SatelliteControllerPID(common.ControllerBase):
    def __init__(self):
        # tuning parameters
        tr_theta = 1.0
        zeta_theta = 0.9
        M = 10  # time separation factor between inner and outer loop
        tr_phi = tr_theta * M
        zeta_phi = 0.9
        self.ki_phi = 0.20

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
        print(f"{DC_theta = :.1f}")  # a0 is 0, so DC_gain is 1

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
        print(
            f"Outer loop (phi): kp = {self.kp_phi:.3f}, ki = {self.ki_phi}, "
            f"kd = {self.kd_phi:.3f}"
        )

        # DC gain of outer loop
        DC_phi = P.k * DC_theta * self.kp_phi / (P.k + P.k * DC_theta * self.kp_phi)
        print(f"{DC_phi = :.3f}")

        self.tau_eq = P.u_eq[0]
        self.theta_max = np.radians(30)

        # dirty derivative variables
        self.sigma = 0.05
        self.beta = (2 * self.sigma - P.ts) / (2 * self.sigma + P.ts)
        self.thetadot_hat = P.thetadot0
        self.theta_prev = P.theta0
        self.phidot_hat = P.phidot0
        self.phi_prev = P.phi0

        # integrator variables
        self.error_phi_prev = 0.0
        self.integral_phi_error = 0.0

    def update_with_measurement(self, r, y):
        # unpack references and states
        theta, phi = y
        phi_ref = r[1]

        # dirty derivative to estimate thetadot
        theta_diff = (theta - self.theta_prev) / P.ts
        self.thetadot_hat = self.beta * self.thetadot_hat + (1 - self.beta) * theta_diff
        self.theta_prev = theta

        # dirty derivative to estimate phidot
        phi_diff = (phi - self.phi_prev) / P.ts
        self.phidot_hat = self.beta * self.phidot_hat + (1 - self.beta) * phi_diff
        self.phi_prev = phi

        # compute input from partially estimated state
        xhat = np.array([theta, phi, self.thetadot_hat, self.phidot_hat])

        # outer loop (modified) PD
        error_phi = phi_ref - phi
        # if abs(phidot) < 0.08:  # anti-windup: only integrate if phidot is small
        #     self.error_phi_integral += P.ts * (error_phi + self.error_phi_prev) / 2
        self.integral_phi_error += P.ts * (error_phi + self.error_phi_prev) / 2
        self.error_phi_prev = error_phi

        theta_ref_unsat = (
            self.kp_phi * error_phi
            + self.ki_phi * self.integral_phi_error
            - self.kd_phi * self.phidot_hat  # no feed-forward term with integrator
        )
        theta_ref = self.saturate(theta_ref_unsat, self.theta_max)
        r[0] = theta_ref  # if you want to visualize the "reference" angle

        # integrator anti-windup
        if self.ki_phi != 0.0:
            self.integral_phi_error += (
                P.ts / self.ki_phi * (theta_ref - theta_ref_unsat)
            )

        # inner loop (modified) PD
        error_theta = theta_ref - theta
        tau_tilde = self.kp_theta * error_theta - self.kd_theta * self.thetadot_hat
        tau = tau_tilde + self.tau_eq
        u_unsat = np.array([tau])
        u = self.saturate(u_unsat, u_max=P.torque_max)

        return u, xhat
