# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class BlockbeamSSIOController(ControllerBase):
    def __init__(self):
        # --- Controller tuning (same as E.12 SSI) ---
        # --- Controller tuning ---
        tr_theta = 0.3      # slightly slower than 0.2, reduces initial spike
        zeta_theta = 0.95   # keep this
        tr_z = 1.5          # faster z tracking (was 2.5)
        zeta_z = 0.95       # CRITICAL — increase from 0.5, eliminates the dip
        integrator_pole = [-8.0]  # slow it down slightly from -5.0

        # --- Augmented system for integral control ---
        A1 = np.block([[P.A, np.zeros((4, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            raise ValueError("Augmented system not controllable")

        # compute controller poles
        wn_theta = 2.2 / tr_theta
        theta_char_poly = [1, 2 * zeta_theta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_z = 2.2 / tr_z
        z_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
        z_poles = np.roots(z_char_poly)

        des_poles = np.hstack([theta_poles, z_poles, integrator_pole])

        self.K1 = cnt.place(A1, B1, des_poles)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]
        print("K:", self.K, "ki:", self.ki)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        # integrator variables
        self.error_prev = 0.0
        self.error_integral = 0.0

        # --- Observer tuning (M times faster than controller) ---
        M_obs = 8.0
        tr_theta_obs = tr_theta / M_obs
        zeta_theta_obs = 0.95
        tr_z_obs = tr_z / M_obs
        zeta_z_obs = 0.95

        # check observability
        if np.linalg.matrix_rank(cnt.ctrb(P.A.T, P.Cm.T)) != 4:
            raise ValueError("System not observable")

        # compute observer poles
        wn_theta_obs = 2.2 / tr_theta_obs
        theta_obs_char_poly = [1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2]
        theta_obs_poles = np.roots(theta_obs_char_poly)

        wn_z_obs = 2.2 / tr_z_obs
        z_obs_char_poly = [1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2]
        z_obs_poles = np.roots(z_obs_char_poly)

        obs_poles = np.hstack([theta_obs_poles, z_obs_poles])
        self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T
        print("L^T:", self.L.T)

        # observer state
        self.xhat_tilde = np.zeros(4)
        self.u_prev = np.zeros(1)

    def update_with_measurement(self, r, y):
        # update observer
        xhat = self.observer_rk4_step(y)

        x_tilde = xhat - self.x_eq
        r_tilde = r - self.r_eq

        # integrate error
        error = r - P.Cr @ xhat
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # compute control using estimated state
        x1_tilde = np.hstack((x_tilde, self.error_integral))
        u_tilde = -self.K1 @ x1_tilde

        # saturate
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)

        # save u for observer
        self.u_prev = u

        return u, xhat

    def observer_f(self, xhat, y):
        y_error = y - P.Cm @ xhat
        xhat_tilde = xhat - self.x_eq
        u_tilde = self.u_prev - self.u_eq
        xhat_dot = P.A @ xhat_tilde + P.B @ u_tilde + self.L @ y_error
        return xhat_dot

    def observer_rk4_step(self, y):
        k1 = self.observer_f(self.xhat_tilde, y)
        k2 = self.observer_f(self.xhat_tilde + P.ts / 2 * k1, y)
        k3 = self.observer_f(self.xhat_tilde + P.ts / 2 * k2, y)
        k4 = self.observer_f(self.xhat_tilde + P.ts * k3, y)
        xhat_tilde_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.xhat_tilde += xhat_tilde_dot * P.ts
        xhat = self.xhat_tilde + self.x_eq
        return xhat

    def update_with_state(self, r, x):
        raise NotImplementedError("Use update_with_measurement for observer control.")