# 3rd-party
import numpy as np
import control as cnt

# local
from . import params as P
from ..common import ControllerBase


class BlockbeamSSIDOController(ControllerBase):
    def __init__(self, case="baseline"):

        # =========================
        # Augmented system (integrator)
        # =========================
        A1 = np.block([
            [P.A, np.zeros((4, 1))],
            [-P.Cr, np.zeros((1, 1))]
        ])
        B1 = np.vstack((P.B, [[0]]))

        # =========================
        # LQR tuning
        # =========================
        if case == "baseline":
            Q = np.diag([10, 10, 1, 1, 5])
            R = np.array([[1]])

        elif case == "slow":
            Q = np.diag([1, 1, 0.1, 0.1, 1])
            R = np.array([[10]])

        elif case == "aggressive":
            Q = np.diag([100, 100, 10, 10, 20])
            R = np.array([[0.1]])

        else:
            raise ValueError("Invalid case")

        # =========================
        # LQR gain
        # =========================
        self.K1, _, E = cnt.lqr(A1, B1, Q, R)
        self.K = self.K1[:, :4]
        self.ki = self.K1[:, 4:]

        print(f"\nCASE: {case}")
        print("K:", self.K)
        print("ki:", self.ki)
        print("Closed-loop poles:", E)

        # =========================
        # Equilibrium
        # =========================
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.u_eq = P.u_eq

        # =========================
        # Integrator
        # =========================
        self.error_prev = 0.0
        self.error_integral = 0.0

        # =========================
        # Observer (same as before)
        # =========================
        M_obs = 8.0

        tr_theta = 0.3 / M_obs
        zeta_theta = 0.95
        tr_z = 1.5 / M_obs
        zeta_z = 0.95

        wn_theta = 2.2 / tr_theta
        theta_poles = np.roots([1, 2*zeta_theta*wn_theta, wn_theta**2])

        wn_z = 2.2 / tr_z
        z_poles = np.roots([1, 2*zeta_z*wn_z, wn_z**2])

        obs_poles = np.hstack([theta_poles, z_poles])

        self.L = cnt.place(P.A.T, P.Cm.T, obs_poles).T

        self.xhat_tilde = np.zeros(4)
        self.u_prev = np.zeros(1)

    # =========================
    # CONTROLLER UPDATE
    # =========================
    def update_with_measurement(self, r, y):
        xhat = self.observer_rk4_step(y)

        x_tilde = xhat - self.x_eq

        # integrator
        error = r - P.Cr @ xhat
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        x1_tilde = np.hstack((x_tilde, self.error_integral))

        u_tilde = -self.K1 @ x1_tilde

        u_unsat = u_tilde + self.u_eq
        u = self.saturate(u_unsat, u_max=P.force_max)

        self.u_prev = u

        return u, xhat

    # =========================
    # OBSERVER
    # =========================
    def observer_f(self, xhat, y):
        y_error = y - P.Cm @ xhat
        xhat_tilde = xhat - self.x_eq
        u_tilde = self.u_prev - self.u_eq

        return P.A @ xhat_tilde + P.B @ u_tilde + self.L @ y_error

    def observer_rk4_step(self, y):
        k1 = self.observer_f(self.xhat_tilde, y)
        k2 = self.observer_f(self.xhat_tilde + P.ts/2 * k1, y)
        k3 = self.observer_f(self.xhat_tilde + P.ts/2 * k2, y)
        k4 = self.observer_f(self.xhat_tilde + P.ts * k3, y)

        self.xhat_tilde += (k1 + 2*k2 + 2*k3 + k4)/6 * P.ts

        return self.xhat_tilde + self.x_eq
