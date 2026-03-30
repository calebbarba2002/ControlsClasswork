# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSIO(ControllerBase):
    def __init__(self):
        # --- Controller tuning (same as F.12) ---
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

        # --- Observer tuning (M_obs times faster than controller) ---
        M_obs = 8.0
        tr_h_obs = tr_h / M_obs
        zeta_h_obs = 0.95
        tr_theta_obs = tr_theta / M_obs
        zeta_theta_obs = 0.95
        tr_z_obs = tr_z / M_obs
        zeta_z_obs = 0.95

        # --- Longitudinal observer ---
        if np.linalg.matrix_rank(cnt.ctrb(P.A_lon.T, P.Cm_lon.T)) != 2:
            raise ValueError("Longitudinal system not observable")

        wn_h_obs = 2.2 / tr_h_obs
        h_obs_poly = [1, 2 * zeta_h_obs * wn_h_obs, wn_h_obs**2]
        h_obs_poles = np.roots(h_obs_poly)

        self.L_lon = cnt.place(P.A_lon.T, P.Cm_lon.T, h_obs_poles).T
        print("L_lon^T:", self.L_lon.T)

        # --- Lateral observer ---
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

        # --- Observer states ---
        self.xhat_lon_tilde = np.zeros(2)
        self.xhat_lat_tilde = np.zeros(4)
        self.u_prev = np.zeros(2)  # [fr, fl]

    def update_with_measurement(self, r, y):
        # y = [z, h, theta] (from Cm = eye(3,6))
        y_lon = y[[1]]       # h
        y_lat = y[[0, 2]]    # z, theta

        # update observers
        xhat_lon = self.observer_lon_rk4_step(y_lon)
        xhat_lat = self.observer_lat_rk4_step(y_lat)

        # --- Longitudinal control ---
        r_lon = r[[1]]  # h reference
        x_lon_tilde = xhat_lon - self.x_lon_eq
        r_lon_tilde = r_lon - self.r_lon_eq

        error_lon = r_lon - P.Cr_lon @ xhat_lon
        self.error_lon_integral += P.ts * (error_lon + self.error_lon_prev) / 2
        self.error_lon_prev = error_lon

        x_lon1_tilde = np.hstack((x_lon_tilde, self.error_lon_integral))
        F_tilde = -self.K_lon1 @ x_lon1_tilde

        # --- Lateral control ---
        r_lat = r[[0]]  # z reference
        x_lat_tilde = xhat_lat - self.x_lat_eq
        r_lat_tilde = r_lat - self.r_lat_eq

        error_lat = r_lat - P.Cr_lat @ xhat_lat
        self.error_lat_integral += P.ts * (error_lat + self.error_lat_prev) / 2
        self.error_lat_prev = error_lat

        x_lat1_tilde = np.hstack((x_lat_tilde, self.error_lat_integral))
        tau_tilde = -self.K_lat1 @ x_lat1_tilde

        # --- Combine and mix ---
        u_FT_tilde = np.hstack([F_tilde, tau_tilde])
        u_tilde = P.mixer @ u_FT_tilde
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.fr_max, u_min=0.0)

        self.u_prev = u

        # reconstruct full xhat for logging [z, h, theta, zdot, hdot, thetadot]
        xhat_full = np.array([
            xhat_lat[0],   # z
            xhat_lon[0],   # h
            xhat_lat[2],   # theta
            xhat_lat[1],   # zdot
            xhat_lon[1],   # hdot
            xhat_lat[3],   # thetadot
        ])

        return u, xhat_full

    def observer_lon_f(self, xhat_lon, y_lon):
        y_error = y_lon - P.Cm_lon @ xhat_lon
        xhat_lon_tilde = xhat_lon - self.x_lon_eq
        # F from previous u: F = unmixer[0] @ u_prev
        F_prev = P.unmixer[0] @ self.u_prev
        u_lon_tilde = F_prev - (P.unmixer[0] @ self.u_eq)
        xhat_dot = P.A_lon @ xhat_lon_tilde + P.B_lon.flatten() * u_lon_tilde + self.L_lon @ y_error
        return xhat_dot

    def observer_lon_rk4_step(self, y_lon):
        k1 = self.observer_lon_f(self.xhat_lon_tilde, y_lon)
        k2 = self.observer_lon_f(self.xhat_lon_tilde + P.ts / 2 * k1, y_lon)
        k3 = self.observer_lon_f(self.xhat_lon_tilde + P.ts / 2 * k2, y_lon)
        k4 = self.observer_lon_f(self.xhat_lon_tilde + P.ts * k3, y_lon)
        self.xhat_lon_tilde += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lon_tilde + self.x_lon_eq

    def observer_lat_f(self, xhat_lat, y_lat):
        y_error = y_lat - P.Cm_lat @ xhat_lat
        xhat_lat_tilde = xhat_lat - self.x_lat_eq
        # tau from previous u
        tau_prev = P.unmixer[1] @ self.u_prev
        u_lat_tilde = tau_prev - (P.unmixer[1] @ self.u_eq)
        xhat_dot = P.A_lat @ xhat_lat_tilde + P.B_lat.flatten() * u_lat_tilde + self.L_lat @ y_error
        return xhat_dot

    def observer_lat_rk4_step(self, y_lat):
        k1 = self.observer_lat_f(self.xhat_lat_tilde, y_lat)
        k2 = self.observer_lat_f(self.xhat_lat_tilde + P.ts / 2 * k1, y_lat)
        k3 = self.observer_lat_f(self.xhat_lat_tilde + P.ts / 2 * k2, y_lat)
        k4 = self.observer_lat_f(self.xhat_lat_tilde + P.ts * k3, y_lat)
        self.xhat_lat_tilde += P.ts * (k1 + 2*k2 + 2*k3 + k4) / 6
        return self.xhat_lat_tilde + self.x_lat_eq

    def update_with_state(self, r, x):
        raise NotImplementedError("Use update_with_measurement for observer control.")