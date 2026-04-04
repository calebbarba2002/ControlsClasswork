# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class BlockbeamSSIDOController(ControllerBase):
    def __init__(self):
        # -------------------------------------------------------
        # Same controller tuning as E.13
        # -------------------------------------------------------
        tr_theta = 0.3
        zeta_theta = 0.95
        tr_z = 1.5
        zeta_z = 0.95
        integrator_pole = [-8.0]

        # --- Augmented system for integral control ---
        A1 = np.block([[P.A, np.zeros((4, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            raise ValueError("Augmented system not controllable")

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

        # -------------------------------------------------------
        # E.13 state observer — same as before
        # -------------------------------------------------------
        M_obs = 8.0
        tr_theta_obs = tr_theta / M_obs
        zeta_theta_obs = 0.95
        tr_z_obs = tr_z / M_obs
        zeta_z_obs = 0.95

        if np.linalg.matrix_rank(cnt.ctrb(P.A.T, P.Cm.T)) != 4:
            raise ValueError("System not observable")

        wn_theta_obs = 2.2 / tr_theta_obs
        theta_obs_char_poly = [1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2]
        theta_obs_poles = np.roots(theta_obs_char_poly)

        wn_z_obs = 2.2 / tr_z_obs
        z_obs_char_poly = [1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2]
        z_obs_poles = np.roots(z_obs_char_poly)

        obs_poles = np.hstack([theta_obs_poles, z_obs_poles])
        self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T
        print("L^T:", self.L.T)

        # -------------------------------------------------------
        # Part (b): Disturbance observer
        # Augment the system with a disturbance state d:
        #   xdot = A*x + B*u + B*d
        #   ddot = 0  (constant disturbance model)
        # Augmented system (5 states: 4 original + 1 disturbance):
        #   [xdot]   [A  B] [x]   [B]
        #   [ddot] = [0  0] [d] + [0] * u
        # -------------------------------------------------------
        n = P.A.shape[0]  # 4 states

        A_aug = np.block([
            [P.A,               P.B              ],
            [np.zeros((1, n)),  np.zeros((1, 1)) ]
        ])
        B_aug = np.vstack([P.B, np.zeros((1, 1))])

        # augmented C — still measure z and theta (first 2 states)
        C_aug = np.hstack([P.Cm, np.zeros((P.Cm.shape[0], 1))])

        # disturbance observer poles — faster than state observer
        M_dist = 1.5  # slightly faster than state observer
        tr_theta_dist = tr_theta_obs / M_dist
        tr_z_dist = tr_z_obs / M_dist
        zeta_dist = 0.95

        wn_theta_dist = 2.2 / tr_theta_dist
        theta_dist_poly = [1, 2 * zeta_dist * wn_theta_dist, wn_theta_dist**2]
        theta_dist_poles = np.roots(theta_dist_poly)

        wn_z_dist = 2.2 / tr_z_dist
        z_dist_poly = [1, 2 * zeta_dist * wn_z_dist, wn_z_dist**2]
        z_dist_poles = np.roots(z_dist_poly)

        # 5th pole for disturbance state
        dist_pole = np.array([-wn_theta_dist * 1.5])

        dist_obs_poles = np.hstack([theta_dist_poles, z_dist_poles, dist_pole])
        self.L_aug = cnt.place(A_aug.T, C_aug.T, dist_obs_poles).T
        print("L_aug^T:", self.L_aug.T)

        # observer states
        self.xhat_tilde = np.zeros(4)       # state observer
        self.xhat_aug_tilde = np.zeros(5)   # disturbance observer (4 states + 1 disturbance)
        self.u_prev = np.zeros(1)
        self.dhat = np.zeros(1)             # disturbance estimate

    def update_with_measurement(self, r, y):
        # -------------------------------------------------------
        # Update state observer (same as E.13)
        # -------------------------------------------------------
        xhat = self.observer_rk4_step(y)

        # -------------------------------------------------------
        # Part (b): Update disturbance observer
        # -------------------------------------------------------
        xhat_aug = self.dist_observer_rk4_step(y)
        self.dhat = xhat_aug[4:5]  # extract disturbance estimate

        x_tilde = xhat - self.x_eq
        r_tilde = r - self.r_eq

        # integrate error
        error = r - P.Cr @ xhat
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # Part (b): subtract disturbance estimate to cancel disturbance
        x1_tilde = np.hstack((x_tilde, self.error_integral))
        u_tilde = -self.K1 @ x1_tilde - self.dhat

        # saturate
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)

        # save u for observers
        self.u_prev = u

        return u, xhat

    # -------------------------------------------------------
    # State observer RK4 (same as E.13)
    # -------------------------------------------------------
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

    # -------------------------------------------------------
    # Part (b): Disturbance observer RK4
    # -------------------------------------------------------
    def dist_observer_f(self, xhat_aug, y):
        # split augmented state into x and d parts
        xhat = xhat_aug[:4]
        dhat = xhat_aug[4:5]

        # augmented matrices
        n = P.A.shape[0]
        A_aug = np.block([
            [P.A,               P.B              ],
            [np.zeros((1, n)),  np.zeros((1, 1)) ]
        ])
        B_aug = np.vstack([P.B, np.zeros((1, 1))])
        C_aug = np.hstack([P.Cm, np.zeros((P.Cm.shape[0], 1))])

        xhat_aug_tilde = xhat_aug - np.hstack([self.x_eq, np.zeros(1)])
        u_tilde = self.u_prev - self.u_eq
        y_error = y - P.Cm @ xhat

        xhat_aug_dot = (A_aug @ xhat_aug_tilde
                        + B_aug @ u_tilde
                        + self.L_aug @ y_error)
        return xhat_aug_dot

    def dist_observer_rk4_step(self, y):
        k1 = self.dist_observer_f(self.xhat_aug_tilde, y)
        k2 = self.dist_observer_f(self.xhat_aug_tilde + P.ts / 2 * k1, y)
        k3 = self.dist_observer_f(self.xhat_aug_tilde + P.ts / 2 * k2, y)
        k4 = self.dist_observer_f(self.xhat_aug_tilde + P.ts * k3, y)
        xhat_aug_tilde_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.xhat_aug_tilde += xhat_aug_tilde_dot * P.ts
        xhat_aug = self.xhat_aug_tilde + np.hstack([self.x_eq, np.zeros(1)])
        return xhat_aug

    def update_with_state(self, r, x):
        raise NotImplementedError("Use update_with_measurement for observer control.")