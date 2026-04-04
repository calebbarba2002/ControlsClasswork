import numpy as np
from ..common import ControllerBase
from . import params as P


class HummingbirdControllerPID(ControllerBase):
    def __init__(self):
        self.kp_theta = P.kp_theta
        self.kd_theta = P.kd_theta

        self.kp_phi = P.kp_phi
        self.kd_phi = P.kd_phi

        self.kp_psi = P.kp_psi
        self.kd_psi = P.kd_psi

        # Integral gains 
        self.ki_theta = 0.05
        self.ki_psi = 0.02

        self.Fe = P.Fe
        self.sigma = P.sigma
        self.beta = (2.0 * self.sigma - P.ts) / (2.0 * self.sigma + P.ts)

        #  Dirty derivative states 
        self.phi_d1 = P.phi0
        self.theta_d1 = P.theta0
        self.psi_d1 = P.psi0

        self.phidot = P.phidot0
        self.thetadot = P.thetadot0
        self.psidot = P.psidot0

        #  Integrator states 
        self.theta_error_integral = 0.0
        self.theta_error_prev = 0.0

        self.psi_error_integral = 0.0
        self.psi_error_prev = 0.0

        #  Saturation limits for anti-windup 
        self.F_max = 2 * P.unmixer[0, 0] * 10.0
        self.tau_max = 0.3

    def update_with_measurement(self, r, y):
        phi = y[0]
        theta = y[1]
        psi = y[2]

        theta_ref = r[0]
        psi_ref = r[1]

        #  Dirty derivatives 
        self.phidot = (self.beta * self.phidot
                       + (1.0 - self.beta) * ((phi - self.phi_d1) / P.ts))
        self.thetadot = (self.beta * self.thetadot
                         + (1.0 - self.beta) * ((theta - self.theta_d1) / P.ts))
        self.psidot = (self.beta * self.psidot
                       + (1.0 - self.beta) * ((psi - self.psi_d1) / P.ts))

        self.phi_d1 = phi
        self.theta_d1 = theta
        self.psi_d1 = psi

        # 
        # LONGITUDINAL CONTROL (theta) with integrator
        # 
        theta_error = theta_ref - theta

        self.theta_error_integral += (P.ts / 2.0) * (theta_error + self.theta_error_prev)
        self.theta_error_prev = theta_error

        F_tilde = (self.kp_theta * theta_error
                   - self.kd_theta * self.thetadot
                   + self.ki_theta * self.theta_error_integral)

        F_fl = self.Fe * np.cos(theta)
        F_unsat = F_fl + F_tilde
        F = np.clip(F_unsat, 0.0, self.F_max)

        # anti-windup
        if F != F_unsat:
            self.theta_error_integral -= (P.ts / 2.0) * (theta_error + self.theta_error_prev)

        # 
        # LATERAL CONTROL - outer loop (psi) with integrator
        # 
        psi_error = psi_ref - psi

        self.psi_error_integral += (P.ts / 2.0) * (psi_error + self.psi_error_prev)
        self.psi_error_prev = psi_error

        phi_ref = (self.kp_psi * psi_error
                   - self.kd_psi * self.psidot
                   + self.ki_psi * self.psi_error_integral)

        # 
        # LATERAL CONTROL - inner loop (phi), PD only
        # 
        phi_error = phi_ref - phi
        tau_unsat = self.kp_phi * phi_error - self.kd_phi * self.phidot
        tau = np.clip(tau_unsat, -self.tau_max, self.tau_max)

        # anti-windup on psi integrator if tau saturates
        if tau != tau_unsat:
            self.psi_error_integral -= (P.ts / 2.0) * (psi_error + self.psi_error_prev)

        xhat = np.array([phi, theta, psi, self.phidot, self.thetadot, self.psidot])
        return np.array([F, tau]), xhat

    def update_with_state(self, r, x):
        phi = x[0]
        theta = x[1]
        psi = x[2]
        phidot = x[3]
        thetadot = x[4]
        psidot = x[5]

        theta_ref = r[0]
        psi_ref = r[1]

        theta_error = theta_ref - theta
        self.theta_error_integral += (P.ts / 2.0) * (theta_error + self.theta_error_prev)
        self.theta_error_prev = theta_error

        F_tilde = (self.kp_theta * theta_error
                   - self.kd_theta * thetadot
                   + self.ki_theta * self.theta_error_integral)
        F_fl = self.Fe * np.cos(theta)
        F_unsat = F_fl + F_tilde
        F = np.clip(F_unsat, 0.0, self.F_max)

        if F != F_unsat:
            self.theta_error_integral -= (P.ts / 2.0) * (theta_error + self.theta_error_prev)

        psi_error = psi_ref - psi
        self.psi_error_integral += (P.ts / 2.0) * (psi_error + self.psi_error_prev)
        self.psi_error_prev = psi_error

        phi_ref = (self.kp_psi * psi_error
                   - self.kd_psi * psidot
                   + self.ki_psi * self.psi_error_integral)

        phi_error = phi_ref - phi
        tau_unsat = self.kp_phi * phi_error - self.kd_phi * phidot
        tau = np.clip(tau_unsat, -self.tau_max, self.tau_max)

        if tau != tau_unsat:
            self.psi_error_integral -= (P.ts / 2.0) * (psi_error + self.psi_error_prev)

        xhat = x
        return np.array([F, tau]), xhat

# import numpy as np
# from ..common import ControllerBase
# from . import params as P
# from case_studies import common


# class HummingbirdControllerPID(ControllerBase):
#     def __init__(self):
#         # --- Same PD gains as before ---
#         self.kp_theta = P.kp_theta
#         self.kd_theta = P.kd_theta

#         self.kp_phi = P.kp_phi
#         self.kd_phi = P.kd_phi

#         self.kp_psi = P.kp_psi
#         self.kd_psi = P.kd_psi

#         # --- Integral gains (tune these) ---
#         self.ki_theta = 0.05   # longitudinal integrator
#         self.ki_psi = 0.08    # outer loop lateral integrator

#         self.Fe = P.Fe
#         self.sigma = P.sigma
#         self.beta = (2.0 * self.sigma - P.ts) / (2.0 * self.sigma + P.ts)

#         # --- Dirty derivative states ---
#         self.phi_d1 = P.phi0
#         self.theta_d1 = P.theta0
#         self.psi_d1 = P.psi0

#         self.phidot = P.phidot0
#         self.thetadot = P.thetadot0
#         self.psidot = P.psidot0

#         # --- Integrator states ---
#         self.theta_error_integral = 0.0
#         self.theta_error_prev = 0.0

#         self.psi_error_integral = 0.0
#         self.psi_error_prev = 0.0

#         # --- Saturation limits for anti-windup ---
#         self.F_max = 2 * P.unmixer[0, 0] * 10.0   # approx max force
#         self.tau_max = 0.3                      # approx max torque

#     def update_with_measurement(self, r, y):
#         phi = y[0]
#         theta = y[1]
#         psi = y[2]

#         theta_ref = r[0]
#         psi_ref = r[1]

#         # --- Dirty derivatives ---
#         self.phidot = (self.beta * self.phidot
#                        + (1.0 - self.beta) * ((phi - self.phi_d1) / P.ts))
#         self.thetadot = (self.beta * self.thetadot
#                          + (1.0 - self.beta) * ((theta - self.theta_d1) / P.ts))
#         self.psidot = (self.beta * self.psidot
#                        + (1.0 - self.beta) * ((psi - self.psi_d1) / P.ts))

#         self.phi_d1 = phi
#         self.theta_d1 = theta
#         self.psi_d1 = psi

#         # =============================================
#         # LONGITUDINAL CONTROL (theta) with integrator
#         # =============================================
#         theta_error = theta_ref - theta

#         # trapezoidal integration
#         self.theta_error_integral += (P.ts / 2.0) * (theta_error + self.theta_error_prev)
#         self.theta_error_prev = theta_error

#         F_tilde = (self.kp_theta * theta_error
#                    - self.kd_theta * self.thetadot
#                    + self.ki_theta * self.theta_error_integral)

#         F_fl = self.Fe * np.cos(theta)
#         F_unsat = F_fl + F_tilde
#         F = np.clip(F_unsat, 0.0, self.F_max)

#         # anti-windup: freeze integrator if saturated
#         if F != F_unsat:
#             self.theta_error_integral -= (P.ts / 2.0) * (theta_error + self.theta_error_prev)

#         # =============================================
#         # LATERAL CONTROL - outer loop (psi) with integrator
#         # =============================================
#         psi_error = psi_ref - psi

#         # trapezoidal integration
#         self.psi_error_integral += (P.ts / 2.0) * (psi_error + self.psi_error_prev)
#         self.psi_error_prev = psi_error

#         phi_ref = (self.kp_psi * psi_error
#                    - self.kd_psi * self.psidot
#                    + self.ki_psi * self.psi_error_integral)

#         # =============================================
#         # LATERAL CONTROL - inner loop (phi), PD only
#         # =============================================
#         phi_error = phi_ref - phi
#         tau_unsat = self.kp_phi * phi_error - self.kd_phi * self.phidot
#         tau = np.clip(tau_unsat, -self.tau_max, self.tau_max)

#         # anti-windup on psi integrator if tau saturates
#         if tau != tau_unsat:
#             self.psi_error_integral -= (P.ts / 2.0) * (psi_error + self.psi_error_prev)

#         xhat = np.array([y[0], y[1], y[2], self.phidot, self.thetadot, self.psidot])
#         return np.array([F, tau]), xhat

#     def update_with_state(self, r, x):
#         phi = x[0]
#         theta = x[1]
#         psi = x[2]
#         phidot = x[3]
#         thetadot = x[4]
#         psidot = x[5]

#         theta_ref = common.SignalGenerator(amplitude=0.0, frequency=0.05)
#         psi_ref = common.SignalGenerator(amplitude=np.radians(30), frequency=0.05)

#         theta_error = theta_ref - theta
#         self.theta_error_integral += (P.ts / 2.0) * (theta_error + self.theta_error_prev)
#         self.theta_error_prev = theta_error

#         F_tilde = (self.kp_theta * theta_error
#                    - self.kd_theta * thetadot
#                    + self.ki_theta * self.theta_error_integral)
#         F_fl = self.Fe * np.cos(theta)
#         F_unsat = F_fl + F_tilde
#         F = np.clip(F_unsat, 0.0, self.F_max)

#         if F != F_unsat:
#             self.theta_error_integral -= (P.ts / 2.0) * (theta_error + self.theta_error_prev)

#         psi_error = psi_ref - psi
#         self.psi_error_integral += (P.ts / 2.0) * (psi_error + self.psi_error_prev)
#         self.psi_error_prev = psi_error

#         phi_ref = (self.kp_psi * psi_error
#                    - self.kd_psi * psidot
#                    + self.ki_psi * self.psi_error_integral)

#         phi_error = phi_ref - phi
#         tau_unsat = self.kp_phi * phi_error - self.kd_phi * phidot
#         tau = np.clip(tau_unsat, -self.tau_max, self.tau_max)

#         if tau != tau_unsat:
#             self.psi_error_integral -= (P.ts / 2.0) * (psi_error + self.psi_error_prev)

#         xhat = x 
#         return np.array([F, tau]), xhat