# 3rd-party
import numpy as np
import control as cnt

# local
import case_studies.D_mass.params as P
from case_studies.common import ControllerBase
from case_studies.control.observer import Observer
from case_studies.control import utils_design


class MassSSIDOController(ControllerBase):
    def __init__(self, case="baseline"):

        # =========================
        # Augmented system (integrator)
        # =========================
        A1 = np.block([
            [P.A, np.zeros((2, 1))],
            [-P.Cr, np.zeros((1, 1))]
        ])
        B1 = np.vstack((P.B, [[0]]))

        # =========================
        # LQR tuning cases
        # =========================
        if case == "baseline":
            Q = np.diag([10, 1, 5])   # [z, zdot, integrator]
            R = np.array([[1]])

        elif case == "slow":
            Q = np.diag([1, 0.1, 1])
            R = np.array([[10]])

        elif case == "aggressive":
            Q = np.diag([100, 10, 20])
            R = np.array([[0.1]])

        else:
            raise ValueError("Invalid case")

        # =========================
        # LQR gain
        # =========================
        self.K1, _, E = cnt.lqr(A1, B1, Q, R)

        self.K = self.K1[:, :2]
        self.ki = self.K1[:, 2:]

        print(f"\nCASE: {case}")
        print("K:", self.K)
        print("ki:", self.ki)
        print("Closed-loop poles:", E)

        # =========================
        # Observer (same as before)
        # =========================
        tr_z = 1.0
        tr_z_obs = tr_z / 10
        zeta_z_obs = 0.9

        obs_poles = utils_design.get_2nd_order_poles(tr_z_obs, zeta_z_obs)
        self.observer = Observer(P.A, P.B, P.Cm, obs_poles, P.ts, print_gain=True)

        # =========================
        # Equilibrium
        # =========================
        self.x_eq = P.x_eq
        self.r_eq = P.Cr @ self.x_eq
        self.y_eq = P.Cm @ self.x_eq
        self.u_eq = P.u_eq

        # =========================
        # Integrator
        # =========================
        self.error_prev = 0.0
        self.error_integral = 0.0

        # =========================
        # Store previous input
        # =========================
        self.u_tilde_prev = np.zeros(1)

    # =========================
    # MAIN UPDATE
    # =========================
    def update_with_measurement(self, r, y):

        # observer update
        y_tilde = y - self.y_eq
        xhat_tilde = self.observer.update(y_tilde, self.u_tilde_prev)
        xhat = xhat_tilde + self.x_eq

        # state error
        x_tilde = xhat - self.x_eq

        # integrator
        error = r - P.Cr @ xhat
        self.error_integral += P.ts * (error + self.error_prev) / 2
        self.error_prev = error

        # augmented state
        x1_tilde = np.hstack((x_tilde, self.error_integral))

        # LQR control
        u_tilde = -self.K1 @ x1_tilde

        # add equilibrium input
        u_unsat = u_tilde + self.u_eq

        # saturate
        u = self.saturate(u_unsat, u_max=P.force_max)

        # save for observer
        self.u_tilde_prev = u - self.u_eq

        return u, xhat

    def update_with_state(self, r, x):
        raise NotImplementedError("Use measurement feedback with observer.")
