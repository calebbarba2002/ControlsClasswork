# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class SatelliteSSIDOController(ControllerBase):
    def __init__(self, separate_integrator=False):
        # tuning parameters
        Q = np.diag([20.0, 20.0, 5.0, 50.0, 50.0])
        R = np.array([[0.1]])

        # augmented system
        A1 = np.block([[P.A, np.zeros((4, 1))], [-P.Cr, np.zeros(1)]])
        B1 = np.vstack((P.B, 0))

        # check controllability
        if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
            raise ValueError("System not controllable")

        # compute gains
        self.K1, *_ = cnt.lqr(A1, B1, Q, R)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]
        if separate_integrator:
            print("K:", self.K)
            print("ki:", self.ki)
        else:
            print("K1:", self.K1)

        # linearization point
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        # integrator variables
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.separate_integrator = separate_integrator

        ####### OBSERVER DESIGN ########
        # observer tuning parameters
        tr_theta_obs = 0.2
        zeta_theta_obs = 0.9
        tr_phi_obs = 0.6
        phieta_phi_obs = 0.9
        disturbance_pole = [-1.0]

        # augmented system
        self.A2 = np.block([[P.A, P.B], [np.zeros((1, 5))]])
        self.B2 = np.block([[P.B], [np.zeros(1)]])
        self.C2 = np.block([P.Cm, np.zeros((2, 1))])

        # check observability
        if np.linalg.matrix_rank(cnt.ctrb(self.A2.T, self.C2.T)) != 5:
            raise ValueError("System not observable")

        # compute observer gain matrix
        wn_theta_obs = 2.2 / tr_theta_obs
        theta_obs_char_poly = [1, 2 * zeta_theta_obs * wn_theta_obs, wn_theta_obs**2]
        theta_obs_poles = np.roots(theta_obs_char_poly)

        wn_phi_obs = 2.2 / tr_phi_obs
        phi_obs_char_poly = [1, 2 * phieta_phi_obs * wn_phi_obs, wn_phi_obs**2]
        phi_obs_poles = np.roots(phi_obs_char_poly)

        obs_poles = np.hstack((theta_obs_poles, phi_obs_poles, disturbance_pole))
        self.L2 = cnt.place(self.A2.T, self.C2.T, obs_poles).T

        # observer variables
        self.x2hat_tilde = np.zeros(5)
        self.u_prev = np.zeros(1)
        self.x2_eq = np.hstack((self.x_eq, [0]))

    def update_with_state(self, r, x):
        # convert to linearization (tilde) variables
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # integrate error
        error = r - P.Cr @ x  # can also use tilde vars (eq subtracts out)
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # compute feedback control
        if self.separate_integrator:
            u_tilde = -self.K @ x_tilde - self.ki @ self.error_integral
        else:
            x1_tilde = np.hstack((x_tilde, self.error_integral))
            u_tilde = -self.K1 @ x1_tilde

        # convert back to original variables
        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.torque_max)

        # save the previous control input for the observer
        self.u_prev = u
        return u

    def update_with_measurement(self, r, y):
        # update the observer with the measurement
        xhat, dhat = self.observer_rk4_step(y)

        # calculate full-state feedback control
        u = self.update_with_state(r, xhat) - dhat
        return u, xhat, dhat

    def observer_f(self, x2hat, y):
        y_error = y - self.C2 @ x2hat  # can also use tilde vars (eq subtracts out)
        x2hat_tilde = x2hat - self.x2_eq
        u_tilde = self.u_prev - self.u_eq
        x2hat_dot = self.A2 @ x2hat_tilde + self.B2 @ u_tilde + self.L2 @ y_error
        return x2hat_dot

    def observer_rk4_step(self, y):
        k1 = self.observer_f(self.x2hat_tilde, y)
        k2 = self.observer_f(self.x2hat_tilde + P.ts / 2 * k1, y)
        k3 = self.observer_f(self.x2hat_tilde + P.ts / 2 * k2, y)
        k4 = self.observer_f(self.x2hat_tilde + P.ts * k3, y)
        x2hat_tilde_dot = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        self.x2hat_tilde += x2hat_tilde_dot * P.ts

        # separate state and disturbance estimates
        xhat_tilde = self.x2hat_tilde[:-1]
        xhat = xhat_tilde + self.x_eq
        dhat = self.x2hat_tilde[-1:]
        return xhat, dhat.copy()
