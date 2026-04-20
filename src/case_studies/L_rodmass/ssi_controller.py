# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIController(ControllerBase):
    def __init__(self):

        # =========================================
        # DESIGN PARAMETERS
        # =========================================
        tr = 0.5
        zeta = 0.9
        integrator_pole = -10.0

        # =========================================
        # AUGMENTED SYSTEM
        # =========================================
        A1 = np.block([
            [P.A, np.zeros((2,1))],
            [-P.Cr, np.zeros((1,1))]
        ])
        B1 = np.vstack((P.B, 0))

        # =========================================
        # DESIRED POLES
        # =========================================
        wn = 2.2 / tr
        poles = np.roots([1, 2*zeta*wn, wn**2])
        des_poles = np.hstack([poles, integrator_pole])

        # =========================================
        # GAINS
        # =========================================
        K1 = cnt.place(A1, B1, des_poles)
        self.K = K1[:, :2]
        self.ki = K1[:, 2]

        print("K =", self.K)
        print("ki =", self.ki)

        # =========================================
        # STATES
        # =========================================
        self.integrator = 0.0
        self.error_prev = 0.0

        self.x_eq = P.x_eq
        self.u_eq = P.u_eq
        self.r_eq = P.Cr @ self.x_eq

    def update_with_state(self, r, x):

        # =========================================
        # TILDE VARIABLES
        # =========================================
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq
        y_tilde = P.Cr @ x_tilde

        # =========================================
        # INTEGRATOR (CRITICAL FIX)
        # =========================================
        error = r_tilde - y_tilde
        self.integrator += P.ts * error

        # =========================================
        # CONTROL LAW
        # =========================================
        u_tilde = -self.K @ x_tilde - self.ki * self.integrator
        u = u_tilde + self.u_eq

        return np.array([u.item()])
