# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class VTOLControllerSSIDO(ControllerBase):
    def __init__(self):

        # =========================================
        # LONGITUDINAL LQR (h dynamics)
        # =========================================
        A_lon1 = np.block([
            [P.A_lon, np.zeros((2, 1))],
            [-P.Cr_lon, np.zeros((1, 1))]
        ])
        B_lon1 = np.vstack((P.B_lon, [[0]]))

        Q_lon = np.diag([10, 5, 10])   # [h, hdot, integrator]
        R_lon = np.array([[1]])

        K_lon1, _, _ = cnt.lqr(A_lon1, B_lon1, Q_lon, R_lon)
        self.K_lon1 = np.asarray(K_lon1)

        print("K_lon1:", self.K_lon1)
        print("Lon poles:", np.linalg.eigvals(A_lon1 - B_lon1 @ self.K_lon1))

        # =========================================
        # LATERAL LQR (z + theta dynamics)
        # =========================================
        A_lat1 = np.block([
            [P.A_lat, np.zeros((4, 1))],
            [-P.Cr_lat, np.zeros((1, 1))]
        ])
        B_lat1 = np.vstack((P.B_lat, [[0]]))

        Q_lat = np.diag([20, 5, 50, 5, 10])   # [z, zdot, theta, thetadot, integrator]
        R_lat = np.array([[1]])

        K_lat1, _, _ = cnt.lqr(A_lat1, B_lat1, Q_lat, R_lat)
        self.K_lat1 = np.asarray(K_lat1)

        print("K_lat1:", self.K_lat1)
        print("Lat poles:", np.linalg.eigvals(A_lat1 - B_lat1 @ self.K_lat1))

        # =========================================
        # INTEGRATORS
        # =========================================
        self.int_lon = 0.0
        self.int_lat = 0.0
        self.err_lon_prev = 0.0
        self.err_lat_prev = 0.0

        # =========================================
        # EQUILIBRIUM
        # =========================================
        self.x_lon_eq = P.x_lon_eq
        self.r_lon_eq = P.Cr_lon @ self.x_lon_eq

        self.x_lat_eq = P.x_lat_eq
        self.r_lat_eq = P.Cr_lat @ self.x_lat_eq

        self.u_eq = P.u_eq

        # =========================================
        # OBSERVER GAINS (same as SSIO)
        # =========================================

        # --- longitudinal observer ---
        tr_h_obs = 0.3
        zeta_h_obs = 0.95
        wn = 2.2 / tr_h_obs
        h_obs_poly = [1, 2*zeta_h_obs*wn, wn**2]
        h_obs_poles = np.roots(h_obs_poly)

        self.L_lon = cnt.place(P.A_lon.T, P.Cm_lon.T, h_obs_poles).T

        # --- lateral observer ---
        tr_theta_obs = 0.05
        zeta_theta_obs = 0.95
        tr_z_obs = 0.5
        zeta_z_obs = 0.95

        wn_theta = 2.2 / tr_theta_obs
        theta_obs_poly = [1, 2*zeta_theta_obs*wn_theta, wn_theta**2]
        theta_obs_poles = np.roots(theta_obs_poly)

        wn_z = 2.2 / tr_z_obs
        z_obs_poly = [1, 2*zeta_z_obs*wn_z, wn_z**2]
        z_obs_poles = np.roots(z_obs_poly)

        obs_poles_lat = np.hstack([theta_obs_poles, z_obs_poles])
        self.L_lat = cnt.place(P.A_lat.T, P.Cm_lat.T, obs_poles_lat).T

        # =========================================
        # OBSERVER STATES
        # =========================================
        self.xhat_lon_tilde = np.zeros(2)
        self.xhat_lat_tilde = np.zeros(4)

        self.u_prev = np.zeros(2)

    def update_with_state(self, r, x):
        raise NotImplementedError("Use measurement feedback.")

    def update_with_measurement(self, r, y):

        # =========================================
        # SPLIT MEASUREMENTS
        # =========================================
        y_lon = y[[1]]       # h
        y_lat = y[[0, 2]]    # z, theta

        # =========================================
        # OBSERVERS
        # =========================================
        xhat_lon = self.observer_lon_rk4_step(y_lon)
        xhat_lat = self.observer_lat_rk4_step(y_lat)

        # =========================================
        # LONGITUDINAL CONTROL (h)
        # =========================================
        r_lon = r[[1]]

        error_lon = r_lon - P.Cr_lon @ xhat_lon
        self.int_lon += P.ts * (error_lon + self.err_lon_prev) / 2
        self.err_lon_prev = error_lon

        x_lon_tilde = xhat_lon - self.x_lon_eq
        x_lon1 = np.hstack((x_lon_tilde, self.int_lon))

        F_tilde = -self.K_lon1 @ x_lon1

        # =========================================
        # LATERAL CONTROL (z)
        # =========================================
        r_lat = r[[0]]

        error_lat = r_lat - P.Cr_lat @ xhat_lat
        self.int_lat += P.ts * (error_lat + self.err_lat_prev) / 2
        self.err_lat_prev = error_lat

        x_lat_tilde = xhat_lat - self.x_lat_eq
        x_lat1 = np.hstack((x_lat_tilde, self.int_lat))

        tau_tilde = -self.K_lat1 @ x_lat1

        # =========================================
        # MIXING
        # =========================================
        u_FT_tilde = np.hstack([F_tilde, tau_tilde])
        u_tilde = P.mixer @ u_FT_tilde

        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.fr_max, u_min=0.0)

        self.u_prev = u

        # =========================================
        # FULL STATE ESTIMATE (for plotting)
        # =========================================
        xhat_full = np.array([
            xhat_lat[0],
            xhat_lon[0],
            xhat_lat[2],
            xhat_lat[1],
            xhat_lon[1],
            xhat_lat[3],
        ])

        return u, xhat_full

    # =========================================
    # OBSERVERS (unchanged)
    # =========================================
    def observer_lon_f(self, xhat, y):
        y_err = y - P.Cm_lon @ xhat
        x_tilde = xhat - self.x_lon_eq

        F_prev = P.unmixer[0] @ self.u_prev
        u_tilde = F_prev - (P.unmixer[0] @ self.u_eq)

        return P.A_lon @ x_tilde + P.B_lon.flatten()*u_tilde + self.L_lon @ y_err

    def observer_lon_rk4_step(self, y):
        k1 = self.observer_lon_f(self.xhat_lon_tilde, y)
        k2 = self.observer_lon_f(self.xhat_lon_tilde + P.ts/2*k1, y)
        k3 = self.observer_lon_f(self.xhat_lon_tilde + P.ts/2*k2, y)
        k4 = self.observer_lon_f(self.xhat_lon_tilde + P.ts*k3, y)

        self.xhat_lon_tilde += P.ts*(k1 + 2*k2 + 2*k3 + k4)/6
        return self.xhat_lon_tilde + self.x_lon_eq

    def observer_lat_f(self, xhat, y):
        y_err = y - P.Cm_lat @ xhat
        x_tilde = xhat - self.x_lat_eq

        tau_prev = P.unmixer[1] @ self.u_prev
        u_tilde = tau_prev - (P.unmixer[1] @ self.u_eq)

        return P.A_lat @ x_tilde + P.B_lat.flatten()*u_tilde + self.L_lat @ y_err

    def observer_lat_rk4_step(self, y):
        k1 = self.observer_lat_f(self.xhat_lat_tilde, y)
        k2 = self.observer_lat_f(self.xhat_lat_tilde + P.ts/2*k1, y)
        k3 = self.observer_lat_f(self.xhat_lat_tilde + P.ts/2*k2, y)
        k4 = self.observer_lat_f(self.xhat_lat_tilde + P.ts*k3, y)

        self.xhat_lat_tilde += P.ts*(k1 + 2*k2 + 2*k3 + k4)/6
        return self.xhat_lat_tilde + self.x_lat_eq
