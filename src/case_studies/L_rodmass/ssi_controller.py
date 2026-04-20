# 3rd-party
import numpy as np
import control as cnt

# local (controlbook)
from . import params as P
from ..common import ControllerBase


class RodMassSSIController(ControllerBase):
    def __init__(self):

        # =========================================
        # 1. DESIGN PARAMETERS (YOU CHANGE THESE)
        # =========================================
        tr = 0.5        # rise time (faster → more aggressive)
        zeta = 0.9      # damping
        integrator_pole = -10.0

        # =========================================
        # 2. AUGMENT SYSTEM (adds integrator)
        # =========================================
        A1 = np.block([
            [P.A, np.zeros((2,1))],
            [-P.Cr, np.zeros((1,1))]
        ])
        B1 = np.vstack((P.B, 0))

        # =========================================
        # 3. DESIRED POLES
        # =========================================
        wn = 2.2 / tr
        des_char_poly = [1, 2*zeta*wn, wn**2]
        poles = np.roots(des_char_poly)

        des_poles = np.hstack([poles, integrator_pole])

        # =========================================
        # 4. PLACE POLES → GET GAINS
        # =========================================
        K1 = cnt.place(A1, B1, des_poles)

        self.K = K1[:, :2]
        self.ki = K1[:, 2]

        print("K =", self.K)
        print("ki =", self.ki)

        # =========================================
        # 5. INTEGRATOR STATE
        # =========================================
        self.integrator = 0.0
        self.error_prev = 0.0

        self.x_eq = P.x_eq
        self.u_eq = P.u_eq
        self.r_eq = P.Cr @ self.x_eq   # reference equilibrium
        self.integrator = 0.0



    def update_with_state(self, r, x):
        # state error
        x_tilde = x - self.x_eq
        r_tilde = r - self.r_eq

        # integrator
        error = r - P.Cr @ x
        self.integrator += P.ts * error

        # control law
        u_tilde = -self.K @ x_tilde - self.ki * self.integrator

        u = u_tilde + self.u_eq

        # CRITICAL: return ONLY u as 1D array
        return np.array([u.item()])

