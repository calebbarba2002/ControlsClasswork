# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSIDO(ControllerBase):
    def __init__(self):
        # -------------------------------------------------------
        # Same controller tuning as F.13
        # -------------------------------------------------------
        tr_h = 2.5
        zeta_h = 0.95
        tr_theta = 0.3
        zeta_theta = 0.9
        M = 10.0
        tr_z = tr_theta * M
        zeta_z = 0.9
        integrator_pole_z = [-1.0]
        integrator_pole_h = [-2.0]

        # --- Longitudinal augmented system ---
        A_lon1 = np.block([[P.A_lon, np.zeros((2, 1))], [-P.Cr_lon, np.zeros(1)]])
        B_lon1 = np.vstack((P.B_lon, 0))

        if np.linalg.matrix_rank(cnt.ctrb(A_lon1, B_lon1)) != 3:
            raise ValueError("Longitudinal system not controllable")

        wn_h = 2.2 / tr_h
        h_char_poly = [1, 2 * zeta_h * wn_h, wn_h**2]
        h_poles = np.roots(h_char_poly)
        des_poles_lon = np.hstack([h_poles, integrator_pole_h])

        self.K_lon1 = cnt.place(A_lon1, B_lon1, des_poles_lon)
        self.K_lon = self.K_lon1[:, :2]
        self.ki_lon = self.K_lon1[:, 2:]
        print("K_lon:", self.K_lon, "ki_lon:", self.ki_lon)

        # --- Lateral augmented system ---
        A_lat1 = np.block([[P.A_lat, np.zeros((4, 1))], [-P.Cr_lat, np.zeros(1)]])
        B_lat1 = np.vstack((P.B_lat, 0))

        if np.linalg.matrix_rank(cnt.ctrb(A_lat1, B_lat1)) != 5:
            raise ValueError("Lateral system not controllable")

        wn_theta = 2.2 / tr_theta
        theta_char_poly = [1, 2 * zeta_theta * wn_theta, wn_theta**2]
        theta_poles = np.roots(theta_char_poly)

        wn_z = 2.2 / tr_z
        z_char_poly = [1, 2 * zeta_z * wn_z, wn_z**2]
        z_poles = np.roots(z_char_poly)

        des_poles_lat = np.hstack([theta_poles, z_poles, integrator_pole_z])

        self.K_lat1 = cnt.place(A_lat1, B_lat1, des_poles_lat)
        self.K_lat = self.K_lat1[:, :4]
        self.ki_lat = self.K_lat1[:, 4:]
        print("K_lat:", self.K_lat, "ki_lat:", self.ki_lat)

        # --- Integrator variables ---
        self.error_lon_prev = 0.0
        self.error_lon_integral = 0.0
        self.error_lat_prev = 0.0
        self.error_lat_integral = 0.0

        # --- Linearization points ---
        self.x_lon_eq = P.x_lon_eq
        self.r_lon_eq = P.Cr_lon @ self.x_lon_eq
        self.x_lat_eq = P.x_lat_eq
        self.r_lat_eq = P.Cr_lat @ self.x_lat_eq
        self.u_eq = P.u_eq

        # -------------------------------------------------------
        # F.13 state observers — same as before
        # -------------------------------------------------------
        M_obs = 8.0
        tr_h_obs = tr_h / M_obs
        zeta_h_obs = 0.95
        tr_theta_obs = tr_theta / M_obs
        zeta_theta_obs = 0.95
        tr_z_obs = tr_z / M_obs
        zeta_z_obs = 0.95

        # longitudinal state observer
        if np.linalg.matrix_rank(cnt.ctrb(P.A_lon.T, P.Cm_lon.T)) != 2:
            raise ValueError("Longitudinal system not observable")

        wn_h_obs = 2.2 / tr_h_obs
        h_obs_poly = [1, 2 * zeta_h_obs * wn_h_obs, wn_h_obs**2]
        h_obs_poles = np.roots(h_obs_poly)
        self.L_lon = cnt.place(P.A_lon.T, P.Cm_lon.T, h_obs_poles).T
        print("L_lon^T:", self.L_lon.T)

        # lateral state observer
        if np.linalg.matrix_rank(cnt.ctrb(P.A_lat.T, P.Cm_lat.T)) != 4:
            raise ValueError("Lateral system not observable")

        wn_theta_obs = 2.2 / tr_theta_obs
        theta_obs_poly = [1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2]
        theta_obs_poles = np.roots(theta_obs_poly)

        wn_z_obs = 2.2 / tr_z_obs
        z_obs_poly = [1, 2 * zeta_z_obs * wn_z_obs, wn_z_obs**2]
        z_obs_poles = np.roots(z_obs_poly)

        obs_poles_lat = np.hstack([theta_obs_poles, z_obs_poles])
        self.L_lat = cnt.place(P.A_lat.T, P.Cm_lat.T, obs_poles_lat).T
        print("L_lat^T:", self.L_lat.T)

        # -------------------------------------------------------
        # Part (b): Longitudinal disturbance observer
        # Augment lon system with altitude disturbance d_h:
        #   hdot  = A_lon*x_lon + B_lon*F + B_lon*d_h
        #   d_dot = 0
        # Augmented: 3 states (h, hdot, d_h)
        # -------------------------------------------------------
        n_lon = P.A_lon.shape[0]  # 2

        A_lon_aug = np.block([
            [P.A_lon,               P.B_lon             ],
            [np.zeros((1, n_lon)),  np.zeros((1, 1))    ]
        ])
        B_lon_aug = np.vstack([P.B_lon, np.zeros((1, 1))])
        C_lon_aug = np.hstack([P.Cm_lon, np.zeros((P.Cm_lon.shape[0], 1))])

        # disturbance observer poles — faster than state observer
        M_dist = 1.5
        tr_h_dist = tr_h_obs / M_dist
        zeta_h_dist = 0.95

        wn_h_dist = 2.2 / tr_h_dist
        h_dist_poly = [1, 2 * zeta_h_dist * wn_h_dist, wn_h_dist**2]
        h_dist_poles = np.roots(h_dist_poly)
        dist_lon_pole = np.array([-wn_h_dist * 1.5])  # 3rd pole for d_h state

        dist_obs_poles_lon = np.hstack([h_dist_poles, dist_lon_pole])
        self.L_lon_aug = cnt.place(A_lon_aug.T, C_lon_aug.T, dist_obs_poles_lon).T
        print("L_lon_aug^T:", self.L_lon_aug.T)

        # -------------------------------------------------------
        # Part (b): Lateral disturbance observer
        # Augment lat system with wind disturbance d_z:
        #   zdot  = A_lat*x_lat + B_lat*tau + [1,0,0,0]^T * d_z
        #   d_dot = 0
        # Augmented: 5 states (z, theta, zdot, thetadot, d_z)
        # Note: wind disturbance enters zdot equation directly
        # -------------------------------------------------------
        n_lat = P.A_lat.shape[0]  # 4

        # wind disturbance enters as zdot perturbation — maps to zdot row
        B_lat_dist = np.array([[0], [0], [1], [0]])  # enters zdot equation

        A_lat_aug = np.block([
            [P.A_lat,               B_lat_dist          ],
            [np.zeros((1, n_lat)),  np.zeros((1, 1))    ]
        ])
        B_lat_aug = np.vstack([P.B_lat, np.zeros((1, 1))])
        C_lat_aug = np.hstack([P.Cm_lat, np.zeros((P.Cm_lat.shape[0], 1))])

        M_dist_lat = 1.5
        tr_theta_dist = tr_theta_obs / M_dist_lat
        tr_z_dist = tr_z_obs / M_dist_lat
        zeta_dist_lat = 0.95

        wn_theta_dist = 2.2 / tr_theta_dist
        theta_dist_poly = [1, 2 * zeta_dist_lat * wn_theta_dist, wn_theta_dist**2]
        theta_dist_poles = np.roots(theta_dist_poly)

        wn_z_dist = 2.2 / tr_z_dist
        z_dist_poly = [1, 2 * zeta_dist_lat * wn_z_dist, wn_z_dist**2]
        z_dist_poles = np.roots(z_dist_poly)

        dist_lat_pole = np.array([-wn_theta_dist * 1.5])  # 5th pole for d_z state

        dist_obs_poles_lat = np.hstack([theta_dist_poles, z_dist_poles, dist_lat_pole])
        self.L_lat_aug = cnt.place(A_lat_aug.T, C_lat_aug.T, dist_obs_poles_lat).T
        print("L_lat_aug^T:", self.L_lat_aug.T)

        # --- Observer states ---
        self.xhat_lon_tilde = np.zeros(2)
        self.xhat_lat_tilde = np.zeros(4)
        self.xhat_lon_aug_tilde = np.zeros(3)   # h, hdot, d_h
        self.xhat_lat_aug_tilde = np.zeros(5)   # z, theta, zdot, thetadot, d_z
        self.u_prev = np.zeros(2)               # [fr, fl]
        self.dhat_lon = np.zeros(1)             # altitude disturbance estimate
        self.dhat_lat = np.zeros(1)             # wind disturbance estimate

        # save augmented matrices for RK4
        self.A_lon_aug = A_lon_aug
        self.B_lon_aug = B_lon_aug
        self.C_lon_aug = C_lon_aug
        self.A_lat_aug = A_lat_aug
        self.B_lat_aug = B_lat_aug
        self.C_lat_aug = C_lat_aug

    def update_with_measurement(self, r, y):
        # y = [z, h, theta]
        y_lon = y[[1]]      # h
        y_lat = y[[0, 2]]   # z, theta

        # --- F.13 state observers ---
        xhat_lon = self.observer_lon_rk4_step(y_lon)
        xhat_lat = self.observer_lat_rk4_step(y_lat)

        # -------------------------------------------------------
        # Part (b): disturbance observer updates
        # -------------------------------------------------------
        xhat_lon_aug = self.dist_obs_lon_rk4_step(y_lon)
        xhat_lat_aug = self.dist_obs_lat_rk4_step(y_lat)

        # extract disturbance estimates
        self.dhat_lon = xhat_lon_aug[2:3]   # altitude disturbance
        self.dhat_lat = xhat_lat_aug[4:5]   # wind disturbance

        # --- Longitudinal control with disturbance feedforward ---
        r_lon = r[[1]]
        x_lon_tilde = xhat_lon - self.x_lon_eq
        error_lon = r_lon - P.Cr_lon @ xhat_lon
        self.error_lon_integral += P.ts * (error_lon + self.error_lon_prev) / 2
        self.error_lon_prev = error_lon

        x_lon1_tilde = np.hstack((x_lon_tilde, self.error_lon_integral))
        # Part (b): subtract disturbance estimate to cancel altitude disturbance
        F_tilde = -self.K_lon1 @ x_lon1_tilde - self.dhat_lon

        # --- Lateral control with disturbance feedforward ---
        r_lat = r[[0]]
        x_lat_tilde = xhat_lat - self.x_lat_eq
        error_lat = r_lat - P.Cr_lat @ xhat_lat
        self.error_lat_integral += P.ts * (error_lat + self.error_lat_prev) / 2
        self.error_lat_prev = error_lat

        x_lat1_tilde = np.hstack((x_lat_tilde, self.error_lat_integral))
        # Part (b): subtract disturbance estimate to cancel wind disturbance
        tau_tilde = -self.K_lat1 @ x_lat1_tilde - self.dhat_lat

        # --- Combine and mix ---
        u_FT_tilde = np.hstack([F_tilde, tau_tilde])
        u_tilde = P.mixer @ u_FT_tilde
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.fr_max, u_min=0.0)

        self.u_prev = u

        # reconstruct full xhat [z, h, theta, zdot, hdot, thetadot]
        xhat_full = np.array([
            xhat_lat[0],    # z
            xhat_lon[0],    # h
            xhat_lat[2],    # theta
            xhat_lat[1],    # zdot
            xhat_lon[1],    # hdot
            xhat_lat[3],    # thetadot
        ])

        return u, xhat_full

    # -------------------------------------------------------
    # F.13 state observer RK4 steps (unchanged)
    # -------------------------------------------------------
    def observer_lon_f(self, xhat_lon, y_lon):
        y_error = y_lon - P.Cm_lon @ xhat_lon
        xhat_lon_tilde = xhat_lon - self.x_lon_eq
        F_prev = P.unmixer[0] @ self.u_prev
        u_lon_tilde = F_prev - (P.unmixer[0] @ self.u_eq)
        xhat_dot = (P.A_lon @ xhat_lon_tilde
                    + P.B_lon.flatten() * u_lon_tilde
                    + self.L_lon @ y_error)
        return xhat_dot

    def observer_lon_rk4_step(self, y_lon):
        k1 = self.observer_lon_f(self.xhat_lon_tilde, y_lon)
        k2 = self.observer_lon_f(self.xhat_lon_tilde + P.ts/2 * k1, y_lon)
        k3 = self.observer_lon_f(self.xhat_lon_tilde + P.ts/2 * k2, y_lon)
        k4 = self.observer_lon_f(self.xhat_lon_tilde + P.ts * k3, y_lon)
        self.xhat_lon_tilde += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lon_tilde + self.x_lon_eq

    def observer_lat_f(self, xhat_lat, y_lat):
        y_error = y_lat - P.Cm_lat @ xhat_lat
        xhat_lat_tilde = xhat_lat - self.x_lat_eq
        tau_prev = P.unmixer[1] @ self.u_prev
        u_lat_tilde = tau_prev - (P.unmixer[1] @ self.u_eq)
        xhat_dot = (P.A_lat @ xhat_lat_tilde
                    + P.B_lat.flatten() * u_lat_tilde
                    + self.L_lat @ y_error)
        return xhat_dot

    def observer_lat_rk4_step(self, y_lat):
        k1 = self.observer_lat_f(self.xhat_lat_tilde, y_lat)
        k2 = self.observer_lat_f(self.xhat_lat_tilde + P.ts/2 * k1, y_lat)
        k3 = self.observer_lat_f(self.xhat_lat_tilde + P.ts/2 * k2, y_lat)
        k4 = self.observer_lat_f(self.xhat_lat_tilde + P.ts * k3, y_lat)
        self.xhat_lat_tilde += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lat_tilde + self.x_lat_eq

    # -------------------------------------------------------
    # Part (b): Longitudinal disturbance observer RK4
    # -------------------------------------------------------
    def dist_obs_lon_f(self, xhat_lon_aug, y_lon):
        xhat_lon = xhat_lon_aug[:2]
        y_error = y_lon - P.Cm_lon @ xhat_lon
        xhat_lon_aug_tilde = xhat_lon_aug - np.hstack([self.x_lon_eq, np.zeros(1)])
        F_prev = P.unmixer[0] @ self.u_prev
        u_lon_tilde = F_prev - (P.unmixer[0] @ self.u_eq)
        xhat_aug_dot = (self.A_lon_aug @ xhat_lon_aug_tilde
                        + self.B_lon_aug.flatten() * u_lon_tilde
                        + self.L_lon_aug @ y_error)
        return xhat_aug_dot

    def dist_obs_lon_rk4_step(self, y_lon):
        k1 = self.dist_obs_lon_f(self.xhat_lon_aug_tilde, y_lon)
        k2 = self.dist_obs_lon_f(self.xhat_lon_aug_tilde + P.ts/2 * k1, y_lon)
        k3 = self.dist_obs_lon_f(self.xhat_lon_aug_tilde + P.ts/2 * k2, y_lon)
        k4 = self.dist_obs_lon_f(self.xhat_lon_aug_tilde + P.ts * k3, y_lon)
        self.xhat_lon_aug_tilde += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lon_aug_tilde + np.hstack([self.x_lon_eq, np.zeros(1)])

    # -------------------------------------------------------
    # Part (b): Lateral disturbance observer RK4
    # -------------------------------------------------------
    def dist_obs_lat_f(self, xhat_lat_aug, y_lat):
        xhat_lat = xhat_lat_aug[:4]
        y_error = y_lat - P.Cm_lat @ xhat_lat
        xhat_lat_aug_tilde = xhat_lat_aug - np.hstack([self.x_lat_eq, np.zeros(1)])
        tau_prev = P.unmixer[1] @ self.u_prev
        u_lat_tilde = tau_prev - (P.unmixer[1] @ self.u_eq)
        xhat_aug_dot = (self.A_lat_aug @ xhat_lat_aug_tilde
                        + self.B_lat_aug.flatten() * u_lat_tilde
                        + self.L_lat_aug @ y_error)
        return xhat_aug_dot

    def dist_obs_lat_rk4_step(self, y_lat):
        k1 = self.dist_obs_lat_f(self.xhat_lat_aug_tilde, y_lat)
        k2 = self.dist_obs_lat_f(self.xhat_lat_aug_tilde + P.ts/2 * k1, y_lat)
        k3 = self.dist_obs_lat_f(self.xhat_lat_aug_tilde + P.ts/2 * k2, y_lat)
        k4 = self.dist_obs_lat_f(self.xhat_lat_aug_tilde + P.ts * k3, y_lat)
        self.xhat_lat_aug_tilde += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lat_aug_tilde + np.hstack([self.x_lat_eq, np.zeros(1)])

    def update_with_state(self, r, x):
        raise NotImplementedError("Use update_with_measurement for observer control.")